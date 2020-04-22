// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>


#include "system.h"
#include "io.h"

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "FreeRTOS/timers.h"

#include <altera_avalon_pio_regs.h>

// Keyboard includes
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"

// VGA includes
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"

// LCD defines
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Task Priorities
#define LEDcontroller_priority 3
#define switchPolling_priority 2
#define PRVGADraw_Task_P      4
#define keyboard_task_P		  2
#define fsmControl_task_P	   1
#define stabilityCheck_task_P	   1


// Definition of Queues
static QueueHandle_t keyboardData; 		  // Queue for changing frequency threshold
static QueueHandle_t raw_freq_data;       // Queue for receiving data

// State enum declaration
typedef enum{
	DEFAULT,
	SHEDDING,
	MONITORING,
	LOADING,
	MAINTENANCE,
	NORMAL
}state;

// Boolean declarations
bool PREVstable = true;		// Monitors if stability changes in monitoring state
bool stable = true;			// Stability bool
bool timerFinished = false; // Flag for 500ms timer finish
bool timing = true;			// Flag for timing reaction time
bool allConnected = false;  // Flag for if relay has reonnected all loads

bool load_status[5];		// Load array controlled by relay
bool switch_status[5];		// Switch array controlled only by switches
bool shed_status[5];		// Tracks which loads have been shed


int measurements[5];		// Previous 5 reaction times
double average;				// Average reaction time
int minimum;				// Minimum reaction time
int maximum;				// Maximum reaction time

#define CHECK_BIT(var,pos) ((var) & (1<<(pos))) // macro checks if a specific bit is set

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

// Timer and task handles
TimerHandle_t timer500;
TaskHandle_t Timer_Reset;
TaskHandle_t xHandle;
TaskHandle_t PRVGADraw;


// Definition of Semaphores
xSemaphoreHandle thresholdSemaphore;
xSemaphoreHandle loadStatusSemaphore;
xSemaphoreHandle systemStatusSemaphore;
xSemaphoreHandle measurementSemaphore;
xSemaphoreHandle stableSemaphore;


//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 			//minimum frequency to draw


// GLOBAL VARIABLES
static volatile int keyboard_toggle = 0; // Keyboard debounce3
static volatile int thresholdFreq = 49;
static volatile int thresholdRoc = 60;
state currentState = DEFAULT;

state operationState = NORMAL;

unsigned volatile int reactionStart = 0; //
unsigned volatile int reactionTotal = 0;  //
unsigned volatile int statCount = 0;  //

unsigned volatile int time500 = 0;       // Variable used for 500ms timer for loading/unloading
unsigned volatile int totalTime = 0;     // Total system uptime
unsigned volatile int dummy_value = 0;   // Passed into push button isr but not used as isr only toggles
unsigned volatile int button_value = 0;  // To switch between modes (0 = normal)
unsigned int switch_value = 0;           // Value of switches
char char_test[100];

static volatile int currentFreq;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);
int initISRs(void);


double freq[100], dfreq[100];
int i = 99, j = 0;
Line line_freq, line_roc;


void PRVGADraw_Task(void *pvParameters ){

	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);



	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);


	// Constants drawn to VGA screen
	alt_up_char_buffer_string(char_buf, "System Status:", 9, 41);
	alt_up_char_buffer_string(char_buf, "System Stability:", 7, 43);
	alt_up_char_buffer_string(char_buf, "Total system uptime (s):", 10, 45);
	alt_up_char_buffer_string(char_buf, "Minimum reaction time (ms):", 8, 47);
	alt_up_char_buffer_string(char_buf, "Maximum reaction time (ms):", 8, 49);
	alt_up_char_buffer_string(char_buf, "Average reaction time (ms):", 8, 51);


	alt_up_char_buffer_string(char_buf, "Previous measurements (ms):", 45, 41);
	alt_up_char_buffer_string(char_buf, "1)", 45, 43);
	alt_up_char_buffer_string(char_buf, "2)", 45, 45);
	alt_up_char_buffer_string(char_buf, "3)", 45, 47);
	alt_up_char_buffer_string(char_buf, "4)", 45, 49);
	alt_up_char_buffer_string(char_buf, "5)", 45, 51);
	alt_up_char_buffer_string(char_buf, "Threshold Values:", 30, 54);
	alt_up_char_buffer_string(char_buf, "Freq: ", 25, 56);
	alt_up_char_buffer_string(char_buf, "RoC: ", 50, 56);


	while(1){

		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);
		// DRAWING GRAPH
		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(i+j)%100]) > MIN_FREQ) && ((int)(freq[(i+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
		// UPDATES SYSTEM STATE
		alt_up_char_buffer_string(char_buf, "             ", 25, 41); // Blanks used to clear vga section before updating

		xSemaphoreTake(systemStatusSemaphore, portMAX_DELAY);
		if(operationState == NORMAL){
			if(currentState == DEFAULT){
				alt_up_char_buffer_string(char_buf, "Normal operation", 25, 41);
			}else {

				alt_up_char_buffer_string(char_buf, "Relay monitoring", 25, 41);
			}
		}else{ // in maintenance

			alt_up_char_buffer_string(char_buf, "Maintenance", 25, 41);
		}
		xSemaphoreGive(systemStatusSemaphore);


		// UPDATE SYSTEM STABILITY
		alt_up_char_buffer_string(char_buf, "             ", 25, 43); // Blanks used to clear vga section before updating
		xSemaphoreTake(stableSemaphore, portMAX_DELAY);
		if(stable){
			alt_up_char_buffer_string(char_buf, "Stable", 25, 43);
		}else{
			alt_up_char_buffer_string(char_buf, "Unstable", 25, 43);
		}
		xSemaphoreGive(stableSemaphore);


		// UPDATES TOTAL TIME ON SCREEN
		alt_up_char_buffer_string(char_buf, "             ", 30, 45);
		totalTime = xTaskGetTickCount() / 1000;
		sprintf(char_test,"%1d", totalTime);
		alt_up_char_buffer_string(char_buf, char_test, 30, 45);

		// UPDATES THRESHOLD VALUES

		alt_up_char_buffer_string(char_buf, "       ", 30, 56);
		alt_up_char_buffer_string(char_buf, "             ", 55, 56);

		xSemaphoreTake(thresholdSemaphore, portMAX_DELAY);
		sprintf(char_test,"%1d", thresholdFreq);
		alt_up_char_buffer_string(char_buf, char_test, 30, 56);
		sprintf(char_test,"%1d", thresholdRoc);
		alt_up_char_buffer_string(char_buf, char_test, 55, 56);
		xSemaphoreGive(thresholdSemaphore);

		// UPDATES MEASUREMENTS
		alt_up_char_buffer_string(char_buf, "   ", 35, 47); // min
		alt_up_char_buffer_string(char_buf, "   ", 35, 49); // max
		alt_up_char_buffer_string(char_buf, "   ", 35, 51); // average

		alt_up_char_buffer_string(char_buf, "   ", 50, 43); // 1
		alt_up_char_buffer_string(char_buf, "   ", 50, 45); // 2
		alt_up_char_buffer_string(char_buf, "   ", 50, 47); // 3
		alt_up_char_buffer_string(char_buf, "   ", 50, 49); // 4
		alt_up_char_buffer_string(char_buf, "   ", 50, 51); // 5

		xSemaphoreTake(measurementSemaphore, portMAX_DELAY);
		sprintf(char_test,"%1d", minimum);
		alt_up_char_buffer_string(char_buf, char_test, 35, 47);
		sprintf(char_test,"%1d", maximum);
		alt_up_char_buffer_string(char_buf, char_test, 35, 49);
		sprintf(char_test,"%1f", average);
		alt_up_char_buffer_string(char_buf, char_test, 35, 51);
		sprintf(char_test,"%1d", measurements[0]);
		alt_up_char_buffer_string(char_buf, char_test, 50, 43);
		sprintf(char_test,"%1d", measurements[1]);
		alt_up_char_buffer_string(char_buf, char_test, 50, 45);
		sprintf(char_test,"%1d", measurements[2]);
		alt_up_char_buffer_string(char_buf, char_test, 50, 47);
		sprintf(char_test,"%1d", measurements[3]);
		alt_up_char_buffer_string(char_buf, char_test, 50, 49);
		sprintf(char_test,"%1d", measurements[4]);
		alt_up_char_buffer_string(char_buf, char_test, 50, 51);
		xSemaphoreGive(measurementSemaphore);


		vTaskDelay(5);


	}
}

// Function used to calculate reaction times and related measurements
void shed_stats(){
//	printf("I AM STOPPING THE TIMER\n");
	int stop = xTaskGetTickCount();
	reactionTotal = stop - reactionStart;
//	printf("Start: %d, Stop: %d\n", reactionStart, stop);
//	printf("REACTION TOTAL: %d\n", reactionTotal);
	// Adds new measurement to zeroth position in array for proper output formatting
	int i;
	xSemaphoreTake(measurementSemaphore, portMAX_DELAY);
	for(i = 4; i > 0; i-- ){
		measurements[i] = measurements[i-1];
	}
	measurements[0] = reactionTotal;

	minimum = measurements[0];
	maximum = measurements[0];

	// Calculates min max and average
	int sum = 0;
	int count = 0;
	for(i = 0; i < 5; i++){
		if(measurements[i] != 0){
			count ++;
			sum = sum + measurements[i];
			if(measurements[i] > maximum){
				maximum = measurements[i];
			}
			if(measurements[i] < minimum){
				minimum = measurements[i];
			}
		}

	}
	if(count != 0){
		average = (double)sum / count;

	}
	xSemaphoreGive(measurementSemaphore);

}

// Sheds the highest priority load that is connected
void loadShedding(){
	int i;
	xSemaphoreTake(loadStatusSemaphore, portMAX_DELAY);
	for(i = 0; i < 5; i++){
		if (load_status[i] == true){ // First load found is highest priority
			load_status[i] = false;
			shed_status[i] = true; //
			allConnected = false;
			break;
		}
	}
	if(timing == true){ // If this is the first load being shed, stop the reaction timer
		timing = false;
		shed_stats();
	}
	xSemaphoreGive(loadStatusSemaphore);
}

// Reconnects the highest priority load that had been shed
void loadReconnect(){
	int i;
	xSemaphoreTake(loadStatusSemaphore, portMAX_DELAY);
	for(i = 0; i < 5; i++){
		if ((load_status[i] == false) && (switch_status[i] == true) && (shed_status[i] == true)){ // Only reconnect load that had been shed
			load_status[i] = true;
			shed_status[i] = false;

			break;
		}
	}
	allConnected = true;
	for(i = 0; i < 5; i++){ // Checks if there are still loads that can be reconnected
		if (shed_status[i] == true){
			allConnected = false;

			break;
		}
	}
	xSemaphoreGive(loadStatusSemaphore);
}

void reset500Timer(){
	timerFinished = false;
	xTimerReset(timer500,0);
}

// FSM for main control logic
void fsmControl_task(void *pvParameters){
	while(1){
	// Overall switch to change between normal and maintenance operations
		xSemaphoreTake(systemStatusSemaphore, portMAX_DELAY);
		switch(operationState){
			case NORMAL:
				switch(currentState){ // fsm for relay system
					case(DEFAULT): // Normal operation where relay does not need to intervene
//						printf("Default state\n");

						if(stable){
							break;

						}else{
							currentState = SHEDDING;
							timing = true;
							reactionStart = xTaskGetTickCount();
						}
						break;

					case(SHEDDING): // State sheds a load then moniters
//						printf("Shedding state\n");
						loadShedding();// Shed a load
						reset500Timer();// Start 500ms timer
						currentState = MONITORING;// Go to monitering state
						break;

					case(MONITORING): // Monitering stability before shedding/reconnecting
//						printf("Monitoring state\n");
						if(timerFinished == true){

							if (stable == true){
								currentState = LOADING;

							}else{
								currentState = SHEDDING;
							}

						}
						break;
					case(LOADING): // Reconnects a load then moniters or returns to default
//						printf("Loading state\n");
						loadReconnect();// add a load
						if(allConnected == true){
							currentState = DEFAULT;
						}else{
							currentState = MONITORING;//else = go to monitering and start timer
							reset500Timer();
						}
						break;
				};
				break;
			case MAINTENANCE:
//				printf("Maintenance state\n");
				break;

		};
		xSemaphoreGive(systemStatusSemaphore);
		vTaskDelay(5);

	};




}

// Receives incoming frequency data, calculates RoC and compares against thresholds
void stabilityCheck_task(void *pvParamters){
	double temp = 0;
	while(1){
		while(uxQueueMessagesWaiting( raw_freq_data ) != 0){
			xQueueReceive( raw_freq_data, freq+i, 0 );
			currentFreq = freq[i];
//			printf("%d\n", currentFreq);
			// RoC calculations
			if(i==0){
				dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
			}
			else{
				dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
			}

			if (dfreq[i] > 100.0){
				dfreq[i] = 100.0;
			}
			i =	++i%100; //point to the next data (oldest) to be overwritten
		}
//		printf("FREQ : %d\n",(int)freq[i]);
//		printf("ROC : %d\n", abs((int)dfreq[i]));

		// Comparing against thresholds to check stability of system
		xSemaphoreTake(stableSemaphore, portMAX_DELAY);
		if(thresholdRoc < abs(dfreq[i])||(thresholdFreq < currentFreq)){
			stable = false;
		}else{
			stable = true;
		}

		if(PREVstable != stable){
			printf("RESETING 500 MS TIMER \n");
			reset500Timer();
		}
		PREVstable = stable;
		xSemaphoreGive(stableSemaphore);
	}
}



// Button ISR toggles a variable whenever button is pressed to
// enter and exit the maintenance state
void buttonISR(void* context, alt_u32 id){

	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

	if(button_value == 1){
		button_value = 0;
		operationState = MAINTENANCE;
	}else{
		button_value = 1;
		operationState = NORMAL;
	}
//	printf("Button value: %d", button_value);


}

// Keyboard ISR for keyboard inputs
void keyboardISR(void* context, alt_u32 id){
	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;
	status = decode_scancode (context, &decode_mode , &key , &ascii) ;
	if(keyboard_toggle == 3){ // Used as a sort of debounce, when a key is pressed it counts as 4, so this reduces that to 1
		if ( status == 0 ){
			xQueueSendFromISR(keyboardData, &key, pdFALSE);
			keyboard_toggle = 0;
		}
	}else{
		keyboard_toggle += 1;

	}

}

// Receives keyboard data from queue and alters thresholds accordingly
void keyboard_task(void *pvParameters){
	unsigned char key;
	while(1){
		xQueueReceive(keyboardData, &key, portMAX_DELAY);
		xSemaphoreTake(thresholdSemaphore, portMAX_DELAY);
		if (key == 0x75) { // up arrow increment freq
			thresholdFreq+= 1;
		}
		else if (key == 0x72) { // down arrow decrement freq
			thresholdFreq -= 1;
		}
		else if (key == 0x74) { // right arrow increment roc
			thresholdRoc += 1;
		}
		else if (key == 0x6b) { // left arrow decerement roc
			thresholdRoc -= 1;
		}
		xSemaphoreGive(thresholdSemaphore);

	}
}

// Receive frequency data from board and send into queue
void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR( raw_freq_data, &temp, pdFALSE );

	return;
}

// Green represents loads being switched off (load shedding)
// Red represent loads that are switched on
// In maintenance mode, no loads are shed
void LEDcontroller_task(void *pvParameters){
	while(1){

		int finalred = 0;
		int tmp;
		int finalgreen = 0;
		int i;
		xSemaphoreTake(loadStatusSemaphore, portMAX_DELAY);
		for ( i = 0; i < 5; i++) { // Convert load statuses array into a int to pass to LED functions
			tmp = load_status[4-i];
			finalred |= tmp << (5 - i - 1);
		};

		if (operationState != MAINTENANCE){ // No green if in maintenance
			for ( i = 0; i < 5; i++) {
				tmp = shed_status[4-i]; // Only show green if load was shed, turn off green if switch down
				finalgreen |= tmp << (5 - i - 1);
			};
		}else{
			finalgreen = 0;
		}
		xSemaphoreGive(loadStatusSemaphore);
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, finalred);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, finalgreen);

		vTaskDelay(5);
	}
}


// Task for polling the switches, sets switch statuses and load statuses
void switchPolling_task(void *pvParameters){
	while(1){
		switch_value = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		int i;
		xSemaphoreTake(loadStatusSemaphore, portMAX_DELAY);
		for (i = 0; i < 5; i++) { // update for first 5 switches (only 5 loads)
			if (CHECK_BIT(switch_value, i)) {
				switch_status[i] = true;
				if((currentState == DEFAULT) || (operationState == MAINTENANCE)){ // Can only turn on loads in these states
					load_status[i] = true;
					}
				}
			else {
				switch_status[i] = false;
				load_status[i] = false;
				shed_status[i] = false;
			}
		}
		xSemaphoreGive(loadStatusSemaphore);




		vTaskDelay(5);
	}
}

void vTimer500Callback(xTimerHandle t_timer500){

	timerFinished = true;

}


// Initialise queues and semaphores
int initOSDataStructs(void)
{
	thresholdSemaphore = xSemaphoreCreateMutex();  // mutex for threshold values for freq and RoC
	loadStatusSemaphore = xSemaphoreCreateMutex(); // mutex for load status and shed status array
	measurementSemaphore = xSemaphoreCreateMutex(); // mutex for various measurements displayed
	systemStatusSemaphore = xSemaphoreCreateMutex();
	stableSemaphore = xSemaphoreCreateMutex();

	keyboardData = xQueueCreate(100, sizeof(unsigned char));
	raw_freq_data = xQueueCreate( 100, sizeof(double) );
	timer500 = xTimerCreate("500ms timer", 500, pdTRUE, NULL, vTimer500Callback);


	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(LEDcontroller_task, "LEDcontroller_task", TASK_STACKSIZE, NULL, LEDcontroller_priority, NULL);
	xTaskCreate(switchPolling_task, "switchPolling_task", TASK_STACKSIZE, NULL, switchPolling_priority, NULL);
	xTaskCreate( PRVGADraw_Task, "DrawTsk", configMINIMAL_STACK_SIZE, NULL, PRVGADraw_Task_P, &PRVGADraw );
	xTaskCreate(keyboard_task, "keyboard_task", TASK_STACKSIZE, NULL, keyboard_task_P, NULL);
	xTaskCreate(fsmControl_task, "fsmControl_task", TASK_STACKSIZE, NULL, fsmControl_task_P, NULL);
	xTaskCreate(stabilityCheck_task, "stabilityCheck_task", TASK_STACKSIZE, NULL, stabilityCheck_task_P, NULL);
	return 0;
}

// Initialise IRs
int initISRs(void){
	// SETUP FOR PUSH BUTTON ISR
    // clears the edge capture register
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0);
    // enable interrupts for all buttons
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
    // register the buttons ISR
    alt_irq_register(PUSH_BUTTON_IRQ, (void*)&dummy_value, buttonISR);

    // SETUP FOR KEYBOARD ISR
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
	}
	alt_up_ps2_clear_fifo (ps2_device);
    alt_irq_register(PS2_IRQ, ps2_device, keyboardISR);
    IOWR_8DIRECT(PS2_BASE,4,1);

    // SETUP FOR FREQUENCY RELAY ISR
    alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

    return 0;

}

int main(int argc, char* argv[], char* envp[])
{
	initOSDataStructs();
	initCreateTasks();
	initISRs();
	vTaskStartScheduler();
	for (;;);
	return 0;
}

