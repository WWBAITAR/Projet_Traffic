# Real-Time traffic light controller (C + FreeRTOS)
:earth_africa: For the french version:
 - [Français](README.fr.md)

 A real-time embedded system that manages the traffic light using RTOS tasks, written in C.

## Table of contents
[Overview](#overview)

[Hardware and software](#hardware--and--software--used)

[System architecture](#System--architecture)


## Overview 
This project is a traffic-light management system built with **C and FreeRTOS**, to simulate how traffic signals are managed in an intesection of 2 roads.

The project was developped to practice real-time behaviour using **FreeRTOS** in a context where time and resources are crucial aspects of the system. 

### System's states: 

The system can operate in 2 states:
 - A **"Normal"** state where the system works without external intervention (normal traffic light behaviour).
 - A **"Manual"** state where the operator can give the commands manually. This mode is used to test and set up the system's parameters.

 In the normal state, two blocking actions can occur:
  - **A button push by a pedestrian**: In this case, the sequencer will prioritise this action, and will give the priority to the pedestrian (Red Light on)
  - **A car waiting in the red light alone**: In this case, the sensor will send a signal to the sequencer that the car is alone in the intersection and will turn on the Green light. 

## Hardware and software used

### Hardware - A customised kit that includes:

 - STM32F103RB Microcontroler (https://www.st.com/en/microcontrollers-microprocessors/stm32f103rb.html).
 - Red/Yellow/Green LEDs.
 - Push buttons.

### Software: 

 - STM32CUBEMX to auto-generate HAL functions.
 - Keil µVision5 as the IDE.
 - FreeRTOS (Real-Time operating system).

## System architecture

The system uses 3 tasks to manage the traffic light sequence:


| Task Name        | Priority | Function |
|-----------------|----------|---------|
| Controleur     | Medium     | Controls the **[state](#systems-states)** of the system (**Normal** or **Manual**) |
| Command  | Low     | Handles the operator's external commands |
| Lecture BP  | High   | Handles the button push by the pedestrians |



