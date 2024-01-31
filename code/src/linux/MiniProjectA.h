/*
 * MiniProjectA.cpp
 *
 * Written for MiniProjectA - EEE3096S 2019 by Matthew Terblanche and Michael Wetzel
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

#ifndef MINIPROJECTA_H
#define MINIPROJECTA_H

//Includes
#include <wiringPi.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <iostream>
#include <signal.h> // For keyboard interrupt


//Defines
#define NUM_THREADS 2




//Define buttons
#define CHANGE_FREQ_BTN 17	//BCM 17
#define RESET_TIME_BTN 27	//BCM 27
#define STOP_START_BTN 22	//BCM 22
#define DISMISS_ALARM_BTN 5	//BCM 5

//Define LED
#define ALARM_LED 18       //BCM 18

//SPI Settings
#define SPI_CHAN 0	//CEO
#define SPI_SPEED 1350000	// Hz (18*75ksps)


//Function definitions
void stop_start_isr(void);
void dismiss_alarm_isr(void);
void reset_isr(void);
void frequency_isr(void);

void monitor(void);
void reset(void);
void change_frequency(void);
void dismiss_alarm(void);

int setup_gpio(void);

int analogReadADC(int analogChannel);
double humidityVoltage(double value);
int temperatureCelsius(int value);
double dacOUT(double light, double humidityV);

void fetchTime(void);

void decToBinary(int n);
void sysTime(void);
void setAlarm(double vOut);
void alarmLED(void);

void cleanUp(int signal);

void checkPhysicalButton();

void sendData();
void sendToTerminal();
void sendToConsole();
void changeTimer();
void sendSysTime();
void outputFormat();

int main(int argc, char* argv[]);

#endif
