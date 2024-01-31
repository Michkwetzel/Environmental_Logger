/*
 * MiniProjectA.cpp
 * 
 * Written by Matthew Terblanche and Michael Wetzel
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "MiniProjectA.h"
#include "CurrentTime.c"

//#define BLYNK_DEBUG
//#define BLYNK_PRINT stdout
#include <BlynkApiWiringPi.h>
#include <BlynkSocket.h>
#include <BlynkOptionsParser.h>

static BlynkTransportSocket _blynkTransport;
BlynkSocket Blynk(_blynkTransport);

static const char *auth, *serv;
static uint16_t port;

#include <BlynkWidgets.h>


unsigned char bufferADC[3];	//buffer to send and receive data in 3 8bit packets
unsigned char bufferDAC[2];     //buffer to send and receive data in 2 8bit packets

bool monitoring = true; // should be set false when stopped

int hoursRTC, minsRTC, secsRTC;		//store RTC time values
int hoursSys = 0;
int minsSys = 0;			//store System time values
int secsSys = 0;
int freq = 1;			// Starts the freq at 1s intervals
long lastInterruptTime = 0;	// Used for button debounce
int vOutBin[10];
bool alarmOn = false;		// Status of alarm
int alarmTimer = 200;		// Counter for alarm. Starts above 180s so the alarm can turn on


double humidity = 0;		// Store the humidity value
int temperature = 0;		// Store the temperature value
int light = 0;			// Store the light value
double vOut = 0;		// Store the output voltage value


int btnState = HIGH;		// Used to toggle button state


long int timerFreq = 1000;	// Used to change timer interval

int timerID1;
int timerID2;
int timerID3;			// Timer IDs
int timerID4;
int timerID5;

BlynkTimer tmr;			// Initiates timer instance

char strSysTime[12];
char strRTCTime[12];
char strTemp[12];		// Used to store output values as strings for formatting
char strHumid[12];
char strVout[12];
char strAlarm[12];



///////////////////////////////////////////////////////////


//	Virtual Pin Numbers		Widget

//		V0	-------------->	Terminal
//		V1	-------------->	System Time display
//		V2	-------------->	Humidity display
//		V3	-------------->	Light display
//		V4	-------------->	Temperature display
//		V5	-------------->	Alarm button


///////////////////////////////////////////////////////////




BLYNK_CONNECTED() {	// Called when Blynk connects

	Blynk.virtualWrite(V5, alarmOn);		// Syncs virtual button state with physical button when Blynk connects

}


BLYNK_WRITE(V5) {	// Called when virtual button is pressed


	alarmOn = false;		// Sets alarm state to false
	Blynk.virtualWrite(V5, LOW);	//Sets virtual button state to LOW

}


void setup() {		// Set up Blynk timers


	tmr.setInterval(500, fetchTime);		// Fetch time every 0.5s

	timerID5 =  tmr.setInterval(timerFreq, outputFormat);		//TimerID5 - calls outputFormat every timerFreq seconds

        tmr.setTimer(1000, [](){	// Timer called once after 1s

        	Blynk.virtualWrite(V0,"clr");   	// Clears terminal widget on app
        
        	Blynk.virtualWrite(V0, "---------------------------------------------------\n"); 
        	Blynk.virtualWrite(V0, "|RTC Time|Sys Timer|Humid|Temp|Light|DAC out|Alarm|\n");		// Prints out table heading to terminal widget 
        	Blynk.virtualWrite(V0, "---------------------------------------------------\n\n"); 

        },1);	// Number of times called


	timerID1 = tmr.setInterval(timerFreq, sendData);		// Send data to Blynk App every timerFreq seconds

        timerID2 = tmr.setInterval(timerFreq, sendToTerminal);		// Print to terminal widget every timerFreq seconds

        timerID3 = tmr.setInterval(timerFreq, sendToConsole);		// Print to pi console every timerFreq seconds

        timerID4 = tmr.setInterval(timerFreq, sendSysTime);		// Send system time to Blynk app every timerFreq seconds

        tmr.setInterval(100, [](){		// Timer called every 0.1 seconds

	        setAlarm(vOut);		// Sets alarm if vOut is outside of specific range

        });


	tmr.setInterval(100, checkPhysicalButton);		// Checks physical button every 0.1 seconds

}


void loop() {
	Blynk.run();		// Start connection to Blynk App

        tmr.run();		// Run timers
}


void outputFormat(){

	sprintf(strSysTime, "%02d:%02d:%02d", hoursSys,minsSys,secsSys);		// Formats system time values as "hh:mm:ss" string

        sprintf(strRTCTime, "%02d:%02d:%02d", hoursRTC,minsRTC,secsRTC);		// Formats RTC time values as "hh:mm:ss" string

        sprintf(strTemp, "%-2dC", temperature);		// Formats temperature values as "19C" string

        sprintf(strHumid, "%-3.1fV", humidity);		// Formats humidity values as "1.3V" string

        sprintf(strVout, "%-3.2fV", vOut);		// Formats output voltage as "1.25V" string

        sprintf(strAlarm, "%1s", (alarmOn == true) ? "*":" ");		// Sets alarm string to "*" if alarmOn is true or " " if alarmOn is false


}

void changeTimer(){


	if (monitoring){		// Only executes if monitoring is set to true

		tmr.deleteTimer(timerID1);
    		tmr.deleteTimer(timerID2);
    		tmr.deleteTimer(timerID3);		// Deletes all timers with timer IDs
    		tmr.deleteTimer(timerID4);
    		tmr.deleteTimer(timerID5);

    		timerID1 = tmr.setInterval(timerFreq, sendData);
    		timerID2 = tmr.setInterval(timerFreq, sendToTerminal);
    		timerID3 = tmr.setInterval(timerFreq, sendToConsole);		// Creates new timers with new timerFreq values
    		timerID4 = tmr.setInterval(timerFreq, sendSysTime);
    		timerID5 = tmr.setInterval(timerFreq, outputFormat); 

	}
}

void checkPhysicalButton() {


	if (digitalRead(DISMISS_ALARM_BTN) == LOW) {		// Execute if physical button is pushed

        	if (btnState != LOW){

            		alarmOn = false;		// If btnState was high set turn alarm off
            		Blynk.virtualWrite(V5,alarmOn);		// Set virtual button to LOW
        	}
	        btnState = LOW;

	} 
	else {

		btnState = HIGH;
    	}
}


void sendSysTime(){

	Blynk.virtualWrite(V1, strSysTime);		// Send system time value to system time display on Blynk App
}

void sendData(){


	Blynk.virtualWrite(V2, strHumid);             // Send humidity value to humidity display on Blynk App
        Blynk.virtualWrite(V3, light);             // Send light value to light display on Blynk App
        Blynk.virtualWrite(V4, strTemp);             // Send temperature value to temperature display on Blynk App

}

void sendToTerminal(){


	Blynk.virtualWrite(V0, "|",strRTCTime," | ",strSysTime, " | ",strHumid," | ",strTemp," | ",light," | ",strVout," |",strAlarm,"|\n");		// Print to terminal widget
        Blynk.virtualWrite(V0, "\n---------------------------------------------------\n\n"); 

}
void sendToConsole(){

	printf("| %02d:%02d:%02d     | %02d:%02d:%02d      | %-3.1f V        | %-2d C     | %-3d     | %-3.2f V      |     %1s     |\n", hoursRTC, minsRTC, secsRTC,hoursSys, minsSys, secsSys, humidity,temperature , light, vOut, 
	(alarmOn  == true) ? "*":" ");
        printf("----------------------------------------------------------------------------------------------\n");		// Print to pi console

}


void stop_start_isr(void){

	//Debounce
        long interruptTime = millis();

        if (interruptTime - lastInterruptTime>200){

		monitor();		// Call monitor function when stop/start button is pushed

	}
    	lastInterruptTime = interruptTime;
}

void dismiss_alarm_isr(void){

	//Debounce
        long interruptTime = millis();

        if (interruptTime - lastInterruptTime>200){

		dismiss_alarm();		// Call dimiss alarm function when dimiss alarm button is pushed

        }
    	lastInterruptTime = interruptTime;
}

void reset_isr(void){

	//Debounce
        long interruptTime = millis();

        if (interruptTime - lastInterruptTime>200){

        	reset();		// Call reset function when reset button is pushed

        }
        lastInterruptTime = interruptTime;
}


void frequency_isr(void){

	//Debounce
        long interruptTime = millis();

        if (interruptTime - lastInterruptTime>200){

		change_frequency();		// Call change frequency function when frequency button is pushed

        }
        lastInterruptTime = interruptTime;
}



void monitor(void){


	if (monitoring) {
    
        	monitoring = false;		// Set monitoring to false if it was true

    		tmr.deleteTimer(timerID1);
    		tmr.deleteTimer(timerID2);		// Delete timers to stop output
    		tmr.deleteTimer(timerID3);    

	}
        else{
    
        	monitoring = true;		// Set monitoring to true if it was false

        	timerID1 = tmr.setInterval(timerFreq, sendData);
        	timerID2 = tmr.setInterval(timerFreq, sendToTerminal);		// Create timers to resume output
    	        timerID3 = tmr.setInterval(timerFreq, sendToConsole);

        }
}


void reset(void){

	Blynk.virtualWrite(V0,"clr");		// Clears terminal widget screen

	Blynk.virtualWrite(V0, "---------------------------------------------------\n"); 
        Blynk.virtualWrite(V0, "|RTC Time|Sys Timer|Humid|Temp|Light|DAC out|Alarm|\n");		// Prints out table heading to terminal widget 
        Blynk.virtualWrite(V0, "---------------------------------------------------\n\n"); 

        printf("\e[1;1H\e[2J");	//Regex - Clears pi console screen

        secsSys = 0;
        minsSys = 0;		// Sets system timer to 0
        hoursSys = 0;
        alarmTimer = 200;		// Resets alarm count
        alarmOn = false;		// Sets alarm to false

        printf("----------------------------------------------------------------------------------------------\n"); 
        printf("|   RTC Time   |   Sys Timer   |   Humidity   |   Temp   |  Light  |   DAC out   |   Alarm   |\n");		// Prints out table heading to pi console 
        printf("----------------------------------------------------------------------------------------------\n"); 

   

}

void change_frequency(void){

	if (freq == 1) {
        	freq = 2;		// If freq was 1 it changes to 2
		timerFreq = freq * 1000;		// updates timerFreq to 2s
        	changeTimer();		// Resets timers with new timerFreq

   		return;
	}

	if (freq == 2){
        	freq = 5;		// If freq was 2 it chnages to 5
		timerFreq = freq * 1000;		// updates timerFreq to 5s
        	changeTimer();

     		return;
	}

	if (freq == 5){
        	freq = 1;		// If freq was 5 it changes back to 1
		timerFreq = freq * 1000;		// updates timerFreq to 1s
        	changeTimer();

       		return;
	}


}


void dismiss_alarm(void){

	alarmOn = false;		// Turns alarm status to off
       	Blynk.virtualWrite(V5,alarmOn);		// Sets virtual button state to LOW
 
}


double humidityVoltage(double value){


	return ((3.3/1023)*value);		// Converts humidity value out of 1023 to voltage out of 3.3V

}


int temperatureCelsius(int value){

	double adcVoltage = ((3.3/1023)*value);		// Converts temperature value out of 1023 to voltage out of 3.3V

	return ((adcVoltage - 0.5) / 0.01);	// Converts temperature voltage to degrees celsius ( 0.5 = OFFSET, 0.01 = Temp Coefficient)

}

void setAlarm(double vOut){

	if (vOut < 0.65 || vOut > 2.65){		// Executes if output voltage is below 0.65V or above 2.65V
		if (alarmTimer > 180 && (monitoring == true)) {		// Executes if alarm was set more than 3min ago and program is currently monitoring
        		alarmOn = true;		// Sets alarm status to true
        		alarmTimer = 0;		// Resets alarm timer 
        		Blynk.virtualWrite(V5,alarmOn);		// Sets virtual button state to HIGH
        	}
       }


}


double dacOUT(double light, double humidityV){

	double vOut = ( (light/1023) * humidityV );		// Divides light value by 1023 and multiplies answer by humidity voltage

	return vOut;		// returns output voltage
}



/*
 * Setup Function. Called once 
 */
int setup_gpio(void){

	//Set up wiring Pi
    	wiringPiSetupGpio();

    	//setting up the led
    	pinMode(ALARM_LED, PWM_OUTPUT);


 	//setting up the buttons
    	pinMode(CHANGE_FREQ_BTN, INPUT);
    	pullUpDnControl(CHANGE_FREQ_BTN, PUD_UP);

    	pinMode(RESET_TIME_BTN, INPUT);
    	pullUpDnControl(RESET_TIME_BTN, PUD_UP);

    	pinMode(STOP_START_BTN, INPUT);
    	pullUpDnControl(STOP_START_BTN, PUD_UP);

    	pinMode(DISMISS_ALARM_BTN, INPUT);
    	pullUpDnControl(DISMISS_ALARM_BTN, PUD_UP);

    	//Attach interrupts to button
    	wiringPiISR(CHANGE_FREQ_BTN, INT_EDGE_FALLING, frequency_isr);
    	wiringPiISR(RESET_TIME_BTN, INT_EDGE_FALLING, reset_isr);
    	wiringPiISR(STOP_START_BTN, INT_EDGE_FALLING, stop_start_isr);
    	wiringPiISR(DISMISS_ALARM_BTN, INT_EDGE_FALLING, dismiss_alarm_isr);

    	//setting up the SPI interface
    	wiringPiSPISetup(SPI_CHAN, SPI_SPEED);


    	return 0;
}


int analogReadADC(int analogChannel){


	bufferADC[0] = 1;
	bufferADC[1] = (8 + analogChannel) << 4;		// Set config bits for ADC
	bufferADC[2] = 0;

	wiringPiSPIDataRW(SPI_CHAN, bufferADC, 3);		// Write config bits to ADC and return digital values

	return (((bufferADC[1] & 3) << 8) + bufferADC[2]);		// Return readable values

}

void fetchTime(void){		// (Using kernel driver for RTC)

	hoursRTC = getHours();		 // Fetch system hours
	minsRTC = getMins();		// Fetch system minutes
	secsRTC = getSecs();		// Fetch system seconds

}

void sysTime(void){

	alarmTimer += freq ;		// Increases alarm timer by freq interval
    	secsSys += freq;		// Increases system seconds by freq interval

	if(secsSys > 59){
        	minsSys += 1;		// If system seconds goes above 59 seconds increase minute by 1
        	secsSys = 0;		// Reset seconds to 0
    	}

    	if(minsSys > 59){
       		hoursSys += 1;		// If system minutes goes above 59 invrease hours by 1
       		minsSys = 0;		// Reset minutes to 9
    	}

}




 void alarmLED(void){		// Function to blink LED

	for(int i=0;i<1024;i++){

        	if(alarmOn){
        		pwmWrite(ALARM_LED,i);		// If alarm is on gradually increase brightness
        		delay(1);
    		}
		else{
			pwmWrite(ALARM_LED,0);		// If alarm is off turn LED off
		}
    	}


	for(int i=1023;i>=0;i--){

        	if(alarmOn){
        		pwmWrite(ALARM_LED,i);		// If alarm if on gradually decrease brightness
        		delay(1);

    		} 
		else{
        		pwmWrite(ALARM_LED,0);		// If alarm is off turn off LED
        	}
   	}

}


void *systemTimeThread(void *threadargs){		// Thread to increase system time even when program is stopped

	while(1){

		sysTime();
        	sleep(freq);
   	}

	pthread_exit(NULL);

}



void *monitorThread(void *threadargs){		// Thread to monitor values


	while(1){


       		humidity = humidityVoltage(analogReadADC(2));		// Set humidity voltage variable
       		temperature = temperatureCelsius(analogReadADC(0));		// Set temperature variable in degrees celsius
       		light = analogReadADC(1);		// Set light variable
       		vOut = dacOUT(analogReadADC(1), humidityVoltage(analogReadADC(2)));		// Set output voltage variable

       		alarmLED();		// Blink alarm LED

    	}
    
    pthread_exit(NULL);
}



int main(int argc, char* argv[]){
    // Call the setup GPIO function
	if(setup_gpio()==-1){
        	printf("Setup error");
	 	return 0;
    	}

	signal(SIGINT, cleanUp);		// Set up keyboard interrupt

     	parse_options(argc, argv, auth, serv, port);		// Fetches variables

     	Blynk.begin(auth, serv, port);		// Connects to blynk.cloud server

     	setup();		// Set up timers


	/* Initialize thread with parameters
     	*/ 
   
    	pthread_attr_t tattr;
    	pthread_t thread_id[NUM_THREADS];
    	int newprio = 99;
    	sched_param param;
    
    	pthread_attr_init (&tattr);
    	pthread_attr_getschedparam (&tattr, &param); /* safe to get existing scheduling param */
    	param.sched_priority = newprio; /* set the priority; others are unchanged */
    	pthread_attr_setschedparam (&tattr, &param); /* setting the new scheduling param */
    

    	pthread_create(&thread_id[0], &tattr, systemTimeThread, (void *)1); /* with new priority specified */
															// Creates two threads to run systemTimeThread and monitorThread at the same time as Blynk
    	pthread_create(&thread_id[1], &tattr, monitorThread, (void *)1); /* with new priority specified */


	printf("----------------------------------------------------------------------------------------------\n"); 
   	printf("|   RTC Time   |   Sys Timer   |   Humidity   |   Temp   |  Light  |   DAC out   |   Alarm   |\n");		// Prints out table header to pi console
   	printf("----------------------------------------------------------------------------------------------\n"); 

    	while(1){

		loop();		// Runs Blynk and timers
	} 

	pthread_exit(NULL);



    	return 0;
}


void cleanUp(int signal) {

	printf("\nProgram Ended\n");
 
	pinMode(ALARM_LED, 0);		//Sets ALARM  LED to input

	exit(0);	//Exits program

}


