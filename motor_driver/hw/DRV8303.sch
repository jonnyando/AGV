EESchema Schematic File Version 2
LIBS:my_power
LIBS:my_passives
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:my_powerICs
LIBS:my_conns
LIBS:my_uSD
LIBS:motor_driver-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L DRV8303 U?
U 1 1 5A917C3F
P 4400 3750
F 0 "U?" H 3975 4975 60  0000 C CNN
F 1 "DRV8303" H 4650 4975 60  0000 C CNN
F 2 "power_ics:HTSSOP-48-_Pitch0.5mm" H 4400 3750 60  0001 C CNN
F 3 "" H 4400 3750 60  0001 C CNN
	1    4400 3750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A917C46
P 4400 5350
F 0 "#PWR?" H 4400 5100 50  0001 C CNN
F 1 "GND" H 4400 5200 50  0000 C CNN
F 2 "" H 4400 5350 50  0001 C CNN
F 3 "" H 4400 5350 50  0001 C CNN
	1    4400 5350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A917C4C
P 2925 3650
F 0 "C?" H 2950 3750 50  0000 L CNN
F 1 "C" H 2950 3550 50  0000 L CNN
F 2 "" H 2963 3500 50  0001 C CNN
F 3 "" H 2925 3650 50  0001 C CNN
	1    2925 3650
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A917C53
P 3350 2900
F 0 "R?" V 3430 2900 50  0000 C CNN
F 1 "R" V 3350 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 3280 2900 50  0001 C CNN
F 3 "" H 3350 2900 50  0001 C CNN
	1    3350 2900
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5A917C5A
P 5200 2700
F 0 "#PWR?" H 5200 2450 50  0001 C CNN
F 1 "GND" H 5200 2550 50  0000 C CNN
F 2 "" H 5200 2700 50  0001 C CNN
F 3 "" H 5200 2700 50  0001 C CNN
	1    5200 2700
	0    -1   -1   0   
$EndComp
$Comp
L VBUS #PWR?
U 1 1 5A917C60
P 5400 5150
F 0 "#PWR?" H 5400 5000 50  0001 C CNN
F 1 "VBUS" H 5400 5300 50  0000 C CNN
F 2 "" H 5400 5150 50  0001 C CNN
F 3 "" H 5400 5150 50  0001 C CNN
	1    5400 5150
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A917C66
P 5275 2975
F 0 "#PWR?" H 5275 2825 50  0001 C CNN
F 1 "+3.3V" H 5275 3115 50  0000 C CNN
F 2 "" H 5275 2975 50  0001 C CNN
F 3 "" H 5275 2975 50  0001 C CNN
	1    5275 2975
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5A917C6C
P 3650 5375
F 0 "#PWR?" H 3650 5125 50  0001 C CNN
F 1 "GNDA" H 3650 5225 50  0000 C CNN
F 2 "" H 3650 5375 50  0001 C CNN
F 3 "" H 3650 5375 50  0001 C CNN
	1    3650 5375
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A917C72
P 3500 5050
F 0 "C?" H 3520 5130 50  0000 L CNN
F 1 "C" H 3520 4970 50  0000 L CNN
F 2 "" H 3538 4900 50  0001 C CNN
F 3 "" H 3500 5050 50  0001 C CNN
	1    3500 5050
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A917C79
P 5475 3600
F 0 "C?" H 5500 3700 50  0000 L CNN
F 1 "C" H 5500 3500 50  0000 L CNN
F 2 "" H 5513 3450 50  0001 C CNN
F 3 "" H 5475 3600 50  0001 C CNN
	1    5475 3600
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5A917C80
P 5475 3100
F 0 "C?" H 5500 3200 50  0000 L CNN
F 1 "C" H 5500 3000 50  0000 L CNN
F 2 "" H 5513 2950 50  0001 C CNN
F 3 "" H 5475 3100 50  0001 C CNN
	1    5475 3100
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5A917C87
P 5475 4100
F 0 "C?" H 5500 4200 50  0000 L CNN
F 1 "C" H 5500 4000 50  0000 L CNN
F 2 "" H 5513 3950 50  0001 C CNN
F 3 "" H 5475 4100 50  0001 C CNN
	1    5475 4100
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5A917C8E
P 5225 5300
F 0 "C?" H 5245 5380 50  0000 L CNN
F 1 "C" H 5245 5220 50  0000 L CNN
F 2 "" H 5263 5150 50  0001 C CNN
F 3 "" H 5225 5300 50  0001 C CNN
	1    5225 5300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A917C95
P 4975 5300
F 0 "C?" H 4995 5380 50  0000 L CNN
F 1 "C" H 4995 5220 50  0000 L CNN
F 2 "" H 5013 5150 50  0001 C CNN
F 3 "" H 4975 5300 50  0001 C CNN
	1    4975 5300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A917C9C
P 5100 5500
F 0 "#PWR?" H 5100 5250 50  0001 C CNN
F 1 "GND" H 5100 5350 50  0000 C CNN
F 2 "" H 5100 5500 50  0001 C CNN
F 3 "" H 5100 5500 50  0001 C CNN
	1    5100 5500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A917CA2
P 3300 4500
F 0 "C?" H 3320 4580 50  0000 L CNN
F 1 "C" H 3320 4420 50  0000 L CNN
F 2 "" H 3338 4350 50  0001 C CNN
F 3 "" H 3300 4500 50  0001 C CNN
	1    3300 4500
	0    1    1    0   
$EndComp
$Comp
L GNDA #PWR?
U 1 1 5A917CA9
P 3100 4500
F 0 "#PWR?" H 3100 4250 50  0001 C CNN
F 1 "GNDA" H 3100 4350 50  0000 C CNN
F 2 "" H 3100 4500 50  0001 C CNN
F 3 "" H 3100 4500 50  0001 C CNN
	1    3100 4500
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5A917CAF
P 3550 3500
F 0 "C?" H 3575 3600 50  0000 L CNN
F 1 "C" H 3575 3400 50  0000 L CNN
F 2 "" H 3588 3350 50  0001 C CNN
F 3 "" H 3550 3500 50  0001 C CNN
	1    3550 3500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5A917CB6
P 3400 3500
F 0 "#PWR?" H 3400 3250 50  0001 C CNN
F 1 "GND" H 3400 3350 50  0000 C CNN
F 2 "" H 3400 3500 50  0001 C CNN
F 3 "" H 3400 3500 50  0001 C CNN
	1    3400 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	2925 3750 2925 3850
Wire Wire Line
	2925 3850 3150 3850
Wire Wire Line
	3150 3850 3150 3700
Wire Wire Line
	3150 3700 3700 3700
Wire Wire Line
	3150 3600 3700 3600
Wire Wire Line
	3150 3600 3150 3450
Wire Wire Line
	3150 3450 2925 3450
Wire Wire Line
	2925 3450 2925 3550
Wire Wire Line
	3025 2900 3200 2900
Wire Wire Line
	3500 2900 3700 2900
Wire Wire Line
	5050 2700 5200 2700
Wire Wire Line
	5125 2700 5125 2900
Wire Wire Line
	5125 2900 5050 2900
Wire Wire Line
	5050 2800 5125 2800
Connection ~ 5125 2800
Connection ~ 5125 2700
Wire Wire Line
	3700 5000 3650 5000
Wire Wire Line
	3650 5000 3650 5375
Wire Wire Line
	4400 5350 4400 5300
Wire Wire Line
	3500 4950 3500 4900
Wire Wire Line
	3500 5150 3500 5200
Connection ~ 3650 5200
Wire Wire Line
	3500 5200 3650 5200
Wire Wire Line
	3500 4900 3700 4900
Wire Wire Line
	5375 4100 5050 4100
Wire Wire Line
	5375 3600 5050 3600
Wire Wire Line
	5375 3100 5050 3100
Wire Wire Line
	5050 3300 5700 3300
Wire Wire Line
	5700 3300 5700 3100
Wire Wire Line
	5700 3100 5575 3100
Wire Wire Line
	5050 3800 5700 3800
Wire Wire Line
	5700 3800 5700 3600
Wire Wire Line
	5700 3600 5575 3600
Wire Wire Line
	5050 4300 5700 4300
Wire Wire Line
	5700 4300 5700 4100
Wire Wire Line
	5700 4100 5575 4100
Wire Wire Line
	5050 3000 5275 3000
Wire Wire Line
	5275 3000 5275 2975
Wire Wire Line
	5100 5000 5050 5000
Wire Wire Line
	4975 5150 5400 5150
Wire Wire Line
	4975 5150 4975 5200
Wire Wire Line
	5225 5200 5225 5150
Connection ~ 5225 5150
Wire Wire Line
	5100 5000 5100 5150
Connection ~ 5100 5150
Wire Wire Line
	4975 5400 4975 5450
Wire Wire Line
	4975 5450 5225 5450
Wire Wire Line
	5225 5450 5225 5400
Wire Wire Line
	5100 5500 5100 5450
Connection ~ 5100 5450
Wire Wire Line
	3100 4500 3200 4500
Wire Wire Line
	3400 4500 3700 4500
Wire Wire Line
	3650 3500 3700 3500
Wire Wire Line
	3400 3500 3450 3500
Text HLabel 3650 3000 0    60   Input ~ 0
nCS
Text HLabel 3650 3100 0    60   Input ~ 0
SDI
Text HLabel 3650 3200 0    60   Input ~ 0
SDO
Text HLabel 3650 3300 0    60   Input ~ 0
SCLK
Text HLabel 3650 3400 0    60   Input ~ 0
DC_CAL
Text HLabel 3650 3800 0    60   Input ~ 0
EN_GATE
Text HLabel 3650 3900 0    60   Input ~ 0
AH
Text HLabel 3650 4000 0    60   Input ~ 0
AL
Text HLabel 3650 4100 0    60   Input ~ 0
BH
Text HLabel 3650 4200 0    60   Input ~ 0
BL
Text HLabel 3650 4300 0    60   Input ~ 0
CH
Text HLabel 3650 4400 0    60   Input ~ 0
CL
Wire Wire Line
	3650 3000 3700 3000
Wire Wire Line
	3650 3100 3700 3100
Wire Wire Line
	3650 3200 3700 3200
Wire Wire Line
	3650 3300 3700 3300
Wire Wire Line
	3650 3400 3700 3400
Wire Wire Line
	3650 3800 3700 3800
Wire Wire Line
	3700 3900 3650 3900
Wire Wire Line
	3650 4000 3700 4000
Wire Wire Line
	3650 4100 3700 4100
Wire Wire Line
	3650 4200 3700 4200
Wire Wire Line
	3650 4300 3700 4300
Wire Wire Line
	3650 4400 3700 4400
Text HLabel 3650 2700 0    60   Input ~ 0
nOCTW
Text HLabel 3650 2800 0    60   Input ~ 0
nFAULT
Wire Wire Line
	3650 2800 3700 2800
Wire Wire Line
	3650 2700 3700 2700
$EndSCHEMATC