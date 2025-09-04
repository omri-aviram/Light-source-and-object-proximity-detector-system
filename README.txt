# README.txt

## Project Title
MSP430-Based Object & Light Source Detection System with PC GUI

## Overview
This project implements a modular embedded system on the MSP430G2553 microcontroller.
It integrates sensors, actuators, and a Python GUI for real-time visualization and control.

### Key Features
- Ultrasonic Telemeter (distance measurement)
- LDR-based Light Source Detection (dual sensors)
- Servo Motor Scanning (0°–180° sweep)
- LCD Display (4-bit parallel mode)
- File Mode Execution (scripts & text stored in Flash)
- Calibration Mode for LDR sensors
- PC GUI (Python/PySimpleGUI + matplotlib) for radar-style visualization

## Repository Structure

### Embedded (C, MSP430 CCS)
- main.c → System FSM (state machine). Controls transitions between modes:
  - state0: Idle / Main Menu
  - state1: Object Detector (ultrasonic)
  - state2: Telemeter mode
  - state3: Light sources scan (LDR1, LDR2)
  - state4: Combined Light & Object detection
  - state5: File Mode (script/text execution)
  - state6: LDR Calibration
- api.c / api.h → High-level application APIs for each state.
- halGPIO.c / halGPIO.h → Hardware Abstraction Layer (LCD, Servo, ADC, UART, Flash, ISRs).
- bsp.c / bsp_msp430x2xx.h → Board Support Package (GPIO, timers, ADC, UART init, Flash clock).
- app.h → Application enums, states, and calibration constants.

### PC-Side (Python GUI)
- main.py (projectPT_all_in_one_gui.py):
  - GUI for:
    - Object detection radar (polar plot).
    - Light source detection (dual LDR).
    - Telemeter live mode.
    - LDR calibration.
    - File/script upload & execution.
  - Handles UART communication (pyserial).
  - Protocol:
    - Enter mode ('1'=Object, '2'=Telemeter, '3'=LDR, '6'=Calibration).
    - Start trigger ('s','t','l').
    - MCU sends header (0xFF + nsamp) then triplets:
      - Object Detect: angle, distance (LSB, MSB).
      - LDR: angle, LDR1, LDR2.

## Hardware Connections
- Servo motor: P2.4 (Timer A1 CCR2 PWM)
- Ultrasonic Sensor:
  - Trigger → P2.6 (Timer A0 CCR1)
  - Echo → P2.2 (Timer A1 CCR1 capture)
- LDR sensors:
  - LDR1 → P1.0 (ADC10 A0)
  - LDR2 → P1.3 (ADC10 A3)
- LCD (4-bit interface):
  - Data: P1.4–P1.7
  - RS → P2.3
  - RW → P2.5
  - E  → P2.1
- Pushbuttons: P2.0, P2.7
- UART:
  - TX → P1.2
  - RX → P1.1

## Build & Run Instructions

### Embedded Side (CCS / MSP430)
1. Import project into Code Composer Studio.
2. Select MSP430G2553 device.
3. Compile and flash to LaunchPad.
4. Reset device – LCD will show Main Menu.

### PC Side (Python GUI)
1. Install requirements:
   pip install pyserial PySimpleGUI matplotlib numpy
2. Connect MSP430 LaunchPad via USB (check COM port, default COM7).
3. Run GUI:
   python main.py

## Usage
- From the GUI, select:
  - Objects Detector → Ultrasonic scan & radar plot.
  - Light Sources Detector → LDR scan & fused results.
  - Telemeter → Single-angle live measurement.
  - File Mode → Upload/execute scripts (LCD & Servo commands).
  - Calibration → Fetch/store LUT calibration from MCU.

## Notes
- System uses low-power modes (LPM); wakeup by UART or PushButton.
- No busy polling (except ADC debounce).
- Calibration required for accurate LDR mapping.
- Python GUI supports CSV export.

## Authors
Developed as part of DCS Lab Project using MSP430 and Python GUI.
