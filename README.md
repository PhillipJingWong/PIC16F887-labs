# PIC16F887-labs
Embedded Computing Labs - PIC16F887

Lab 6 — Assembly Programmable LED Display

This lab involved developing an assembly program to control 8 LEDs as a programmable display with three distinct modes:
	1.	Variable PWM — One LED changes brightness  at a configurable speed, between the range of two configurable brightness levels.
	2.	Side-to-side Strobe — A single LED moves across the LEDs back and forth or in one direction, with configurable speed and direction.
	3.	Linear Feedback Shift Register (LFSR) — Displays pseudo-random 8-bit sequences on the LEDs, with configurable update speed.

The active display mode is selected based on an analog input voltage (ADC value on AN0), split into three ranges. Changing the input dynamically switches modes with minimal delay, preserving the previous mode’s state.

Pressing a button enters configuration mode for the current display mode, allowing the user to adjust parameters (speed, brightness, direction, etc.) using the ADC input and button presses. Visual feedback is provided via LEDs.
