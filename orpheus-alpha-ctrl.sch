EESchema Schematic File Version 2
LIBS:power
LIBS:device
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
LIBS:ESP8266
LIBS:amp_ctrl-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L NodeMCU1.0(ESP-12E) U1
U 1 1 59BD7C17
P 4950 3750
F 0 "U1" H 4950 4943 60  0000 C CNN
F 1 "NodeMCU1.0(ESP-12E)" H 4950 4837 60  0000 C CNN
F 2 "ESP8266:NodeMCU1.0(12-E)" H 4950 4731 60  0000 C CNN
F 3 "" H 4350 2900 60  0000 C CNN
	1    4950 3750
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X04 J1
U 1 1 59BD7D4C
P 7850 2550
F 0 "J1" H 7850 2915 50  0000 C CNN
F 1 "CONN_02X04" H 7850 2824 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x04_Pitch2.54mm" H 7850 1350 50  0001 C CNN
F 3 "" H 7850 1350 50  0001 C CNN
	1    7850 2550
	1    0    0    -1  
$EndComp
Text GLabel 7600 2400 0    60   Input ~ 0
VD
Text GLabel 7600 2500 0    60   Input ~ 0
SDI
Text GLabel 7600 2600 0    60   Input ~ 0
CS
Text GLabel 7600 2700 0    60   Input ~ 0
Zcen
Text GLabel 8100 2500 2    60   Input ~ 0
CLK
Text GLabel 8100 2600 2    60   Input ~ 0
SDO
Text GLabel 8100 2700 2    60   Input ~ 0
Mute
$Comp
L GNDD #PWR01
U 1 1 59BD7E78
P 8100 2400
F 0 "#PWR01" H 8100 2150 50  0001 C CNN
F 1 "GNDD" V 8104 2290 50  0000 R CNN
F 2 "" H 8100 2400 50  0001 C CNN
F 3 "" H 8100 2400 50  0001 C CNN
	1    8100 2400
	0    -1   -1   0   
$EndComp
$Comp
L GNDD #PWR02
U 1 1 59BD7F06
P 5750 4350
F 0 "#PWR02" H 5750 4100 50  0001 C CNN
F 1 "GNDD" V 5754 4240 50  0000 R CNN
F 2 "" H 5750 4350 50  0001 C CNN
F 3 "" H 5750 4350 50  0001 C CNN
	1    5750 4350
	0    -1   -1   0   
$EndComp
$Comp
L GNDD #PWR03
U 1 1 59BD7F19
P 5750 3650
F 0 "#PWR03" H 5750 3400 50  0001 C CNN
F 1 "GNDD" V 5754 3540 50  0000 R CNN
F 2 "" H 5750 3650 50  0001 C CNN
F 3 "" H 5750 3650 50  0001 C CNN
	1    5750 3650
	0    -1   -1   0   
$EndComp
$Comp
L GNDD #PWR04
U 1 1 59BD7F28
P 4150 4350
F 0 "#PWR04" H 4150 4100 50  0001 C CNN
F 1 "GNDD" V 4154 4240 50  0000 R CNN
F 2 "" H 4150 4350 50  0001 C CNN
F 3 "" H 4150 4350 50  0001 C CNN
	1    4150 4350
	0    1    1    0   
$EndComp
$Comp
L GNDD #PWR05
U 1 1 59BD7F3B
P 4150 3950
F 0 "#PWR05" H 4150 3700 50  0001 C CNN
F 1 "GNDD" V 4154 3840 50  0000 R CNN
F 2 "" H 4150 3950 50  0001 C CNN
F 3 "" H 4150 3950 50  0001 C CNN
	1    4150 3950
	0    1    1    0   
$EndComp
Text GLabel 4150 4450 0    60   Input ~ 0
VD
Text GLabel 5750 3350 2    60   Input ~ 0
Mute
Text GLabel 5750 3450 2    60   Input ~ 0
Zcen
Text GLabel 5750 3750 2    60   Input ~ 0
CLK
Text GLabel 5750 3850 2    60   Input ~ 0
SDO
Text GLabel 5750 3950 2    60   Input ~ 0
SDI
Text GLabel 5750 4050 2    60   Input ~ 0
CS
Text GLabel 5750 3150 2    60   Input ~ 0
ROT2
Text GLabel 5750 3250 2    60   Input ~ 0
ROT1
Text GLabel 7500 4150 1    60   Input ~ 0
VD
$Comp
L R R1
U 1 1 59BD8132
P 7500 4300
F 0 "R1" H 7570 4346 50  0000 L CNN
F 1 "R" H 7570 4255 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7430 4300 50  0001 C CNN
F 3 "" H 7500 4300 50  0001 C CNN
	1    7500 4300
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 59BD818E
P 7500 4600
F 0 "D1" V 7538 4483 50  0000 R CNN
F 1 "LED" V 7447 4483 50  0000 R CNN
F 2 "LEDs:LED_D4.0mm" H 7500 4600 50  0001 C CNN
F 3 "" H 7500 4600 50  0001 C CNN
	1    7500 4600
	0    -1   -1   0   
$EndComp
$Comp
L GNDD #PWR06
U 1 1 59BD81D9
P 7500 4750
F 0 "#PWR06" H 7500 4500 50  0001 C CNN
F 1 "GNDD" H 7504 4595 50  0000 C CNN
F 2 "" H 7500 4750 50  0001 C CNN
F 3 "" H 7500 4750 50  0001 C CNN
	1    7500 4750
	1    0    0    -1  
$EndComp
NoConn ~ 5750 3050
NoConn ~ 5750 3550
NoConn ~ 5750 4150
NoConn ~ 5750 4250
NoConn ~ 5750 4450
NoConn ~ 4150 4250
NoConn ~ 4150 4150
NoConn ~ 4150 4050
NoConn ~ 4150 3850
NoConn ~ 4150 3750
NoConn ~ 4150 3650
NoConn ~ 4150 3550
NoConn ~ 4150 3450
NoConn ~ 4150 3250
NoConn ~ 4150 3150
NoConn ~ 4150 3050
$Comp
L Screw_Terminal_1x05 J2
U 1 1 59BD86A0
P 8850 2850
F 0 "J2" H 8930 3492 50  0000 C CNN
F 1 "Screw_Terminal_1x05" H 8930 3401 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_Pheonix_MPT-2.54mm_5pol" H 8850 2325 50  0001 C CNN
F 3 "" H 8825 3150 50  0001 C CNN
	1    8850 2850
	1    0    0    -1  
$EndComp
Text GLabel 9050 2450 2    60   Input ~ 0
VD
$Comp
L GNDD #PWR07
U 1 1 59BD86F7
P 9050 2650
F 0 "#PWR07" H 9050 2400 50  0001 C CNN
F 1 "GNDD" V 9054 2540 50  0000 R CNN
F 2 "" H 9050 2650 50  0001 C CNN
F 3 "" H 9050 2650 50  0001 C CNN
	1    9050 2650
	0    -1   -1   0   
$EndComp
$Comp
L GNDD #PWR08
U 1 1 59BD870C
P 9050 3050
F 0 "#PWR08" H 9050 2800 50  0001 C CNN
F 1 "GNDD" V 9054 2940 50  0000 R CNN
F 2 "" H 9050 3050 50  0001 C CNN
F 3 "" H 9050 3050 50  0001 C CNN
	1    9050 3050
	0    -1   -1   0   
$EndComp
Text GLabel 9050 2850 2    60   Input ~ 0
ROT1
Text GLabel 9050 3250 2    60   Input ~ 0
ROT2
$Comp
L CONN_01X03_MALE J3
U 1 1 59BE6A6E
P 8500 4200
F 0 "J3" H 8606 4590 50  0000 C CNN
F 1 "CONN_01X03_MALE" H 8606 4499 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 8500 4400 50  0001 C CNN
F 3 "" H 8500 4400 50  0001 C CNN
	1    8500 4200
	1    0    0    -1  
$EndComp
Text GLabel 8800 4000 2    60   Input ~ 0
VD
$Comp
L GNDD #PWR09
U 1 1 59BE6AA7
P 8800 4200
F 0 "#PWR09" H 8800 3950 50  0001 C CNN
F 1 "GNDD" V 8804 4090 50  0000 R CNN
F 2 "" H 8800 4200 50  0001 C CNN
F 3 "" H 8800 4200 50  0001 C CNN
	1    8800 4200
	0    -1   -1   0   
$EndComp
Text GLabel 8800 4400 2    60   Input ~ 0
NEO
Text GLabel 4150 3350 0    60   Input ~ 0
NEO
$EndSCHEMATC
