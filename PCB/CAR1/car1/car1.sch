EESchema Schematic File Version 4
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
L Connector:Raspberry_Pi_2_3 J?
U 1 1 5D28A729
P 3300 5850
F 0 "J?" H 3300 7328 50  0000 C CNN
F 1 "Raspberry_Pi_2_3" H 3300 7237 50  0000 C CNN
F 2 "" H 3300 5850 50  0001 C CNN
F 3 "https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3bplus_1p0_reduced.pdf" H 3300 5850 50  0001 C CNN
	1    3300 5850
	1    0    0    -1  
$EndComp
$Comp
L arduino:Arduino_Due_Shield XA?
U 1 1 5D28A80B
P 7400 3850
F 0 "XA?" H 7400 1470 60  0000 C CNN
F 1 "Arduino_Due_Shield" H 7400 1364 60  0000 C CNN
F 2 "" H 8100 6600 60  0001 C CNN
F 3 "https://store.arduino.cc/arduino-due" H 8100 6600 60  0001 C CNN
	1    7400 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 5D28AE36
P 4100 3050
F 0 "D?" H 4092 2795 50  0000 C CNN
F 1 "RESER_LMP" H 4092 2886 50  0000 C CNN
F 2 "" H 4100 3050 50  0001 C CNN
F 3 "~" H 4100 3050 50  0001 C CNN
	1    4100 3050
	-1   0    0    1   
$EndComp
$Comp
L Device:LED OK_LMP
U 1 1 5D28AF4B
P 2650 1350
F 0 "OK_LMP" H 2642 1095 50  0000 C CNN
F 1 "LED" H 2642 1186 50  0000 C CNN
F 2 "" H 2650 1350 50  0001 C CNN
F 3 "~" H 2650 1350 50  0001 C CNN
	1    2650 1350
	-1   0    0    1   
$EndComp
$Comp
L user:Ultrasonic U?
U 1 1 5D28B279
P 9700 3900
F 0 "U?" H 9828 3646 50  0000 L CNN
F 1 "Ultrasonic" H 9828 3555 50  0000 L CNN
F 2 "" H 9700 3900 50  0001 C CNN
F 3 "" H 9700 3900 50  0001 C CNN
	1    9700 3900
	1    0    0    -1  
$EndComp
$Comp
L user:Ultrasonic U?
U 1 1 5D28B330
P 9700 4500
F 0 "U?" H 9828 4246 50  0000 L CNN
F 1 "Ultrasonic" H 9828 4155 50  0000 L CNN
F 2 "" H 9700 4500 50  0001 C CNN
F 3 "" H 9700 4500 50  0001 C CNN
	1    9700 4500
	1    0    0    -1  
$EndComp
$Comp
L user:Ultrasonic U?
U 1 1 5D28B386
P 9700 5100
F 0 "U?" H 9828 4846 50  0000 L CNN
F 1 "Ultrasonic" H 9828 4755 50  0000 L CNN
F 2 "" H 9700 5100 50  0001 C CNN
F 3 "" H 9700 5100 50  0001 C CNN
	1    9700 5100
	1    0    0    -1  
$EndComp
$Comp
L user:PowerOffRelay U?
U 1 1 5D28D8E9
P 3100 2650
F 0 "U?" H 3400 2917 50  0000 C CNN
F 1 "PowerOffRelay" H 3400 2826 50  0000 C CNN
F 2 "" H 3400 2800 50  0001 C CNN
F 3 "" H 3400 2800 50  0001 C CNN
	1    3100 2650
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Switching:CRE1S0505S3C U?
U 1 1 5D28E011
P 2050 2850
F 0 "U?" H 2050 3317 50  0000 C CNN
F 1 "CRE1S0505S3C" H 2050 3226 50  0000 C CNN
F 2 "Converter_DCDC:Converter_DCDC_muRata_CRE1xxxxxx3C_THT" H 2050 2450 50  0001 C CNN
F 3 "http://power.murata.com/datasheet?/data/power/ncl/kdc_cre1.pdf" H 2050 2350 50  0001 C CNN
	1    2050 2850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad B+
U 1 1 5D28E2A8
P 950 2200
F 0 "B+" H 1050 2251 50  0000 L CNN
F 1 " " H 1050 2160 50  0000 L CNN
F 2 "" H 950 2200 50  0001 C CNN
F 3 "~" H 950 2200 50  0001 C CNN
	1    950  2200
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad B-
U 1 1 5D28E326
P 650 2200
F 0 "B-" H 750 2251 50  0000 L CNN
F 1 " " H 750 2160 50  0000 L CNN
F 2 "" H 650 2200 50  0001 C CNN
F 3 "~" H 650 2200 50  0001 C CNN
	1    650  2200
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse PI_FUSE
U 1 1 5D28E44D
P 4300 3400
F 0 "PI_FUSE" H 4360 3446 50  0000 L CNN
F 1 "Fuse" H 4360 3355 50  0000 L CNN
F 2 "" V 4230 3400 50  0001 C CNN
F 3 "~" H 4300 3400 50  0001 C CNN
	1    4300 3400
	-1   0    0    1   
$EndComp
$Comp
L Switch:SW_SPST BREAKER
U 1 1 5D28EAF3
P 1200 2650
F 0 "BREAKER" H 1200 2885 50  0000 C CNN
F 1 "SW_SPST" H 1200 2794 50  0000 C CNN
F 2 "" H 1200 2650 50  0001 C CNN
F 3 "" H 1200 2650 50  0001 C CNN
	1    1200 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse MCU_FUSE
U 1 1 5D28EC26
P 4650 3400
F 0 "MCU_FUSE" V 4847 3400 50  0000 C CNN
F 1 "Fuse" V 4756 3400 50  0000 C CNN
F 2 "" V 4580 3400 50  0001 C CNN
F 3 "~" H 4650 3400 50  0001 C CNN
	1    4650 3400
	-1   0    0    1   
$EndComp
$Comp
L RF:NRF24L01_Breakout U?
U 1 1 5D28F8B3
P 9700 1200
F 0 "U?" H 10178 1178 50  0000 L CNN
F 1 "NRF24L01_Breakout" H 10178 1087 50  0000 L CNN
F 2 "RF_Module:nRF24L01_Breakout" H 9850 1800 50  0001 L CIN
F 3 "http://www.nordicsemi.com/eng/content/download/2730/34105/file/nRF24L01_Product_Specification_v2_0.pdf" H 9700 1100 50  0001 C CNN
	1    9700 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 2650 950  2650
Wire Wire Line
	950  2650 950  2300
Wire Wire Line
	1550 3050 650  3050
Wire Wire Line
	650  3050 650  2300
Wire Wire Line
	1400 2650 1550 2650
$Comp
L Device:R R?
U 1 1 5D290A59
P 4400 3050
F 0 "R?" V 4193 3050 50  0000 C CNN
F 1 "330" V 4284 3050 50  0000 C CNN
F 2 "" V 4330 3050 50  0001 C CNN
F 3 "~" H 4400 3050 50  0001 C CNN
	1    4400 3050
	0    1    1    0   
$EndComp
Text GLabel 2550 3050 2    50   Input ~ 0
GND
Text GLabel 1350 3050 1    50   Input ~ 0
GND
Text GLabel 4650 2950 2    50   Input ~ 0
GND
$Comp
L Switch:SW_Push SW?
U 1 1 5D292626
P 4100 2850
F 0 "SW?" H 4100 3135 50  0000 C CNN
F 1 "SW_Push" H 4100 3044 50  0000 C CNN
F 2 "" H 4100 3050 50  0001 C CNN
F 3 "" H 4100 3050 50  0001 C CNN
	1    4100 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2650 2850 2650
Wire Wire Line
	2850 2650 2850 3150
Wire Wire Line
	2850 3150 3100 3150
Wire Wire Line
	2850 2650 2850 2300
Wire Wire Line
	2850 2300 3700 2300
Wire Wire Line
	3700 2300 3700 2650
Connection ~ 2850 2650
Wire Wire Line
	3700 2850 3900 2850
Wire Wire Line
	4550 3050 4600 3050
Wire Wire Line
	4600 3050 4600 2950
Wire Wire Line
	4600 2950 4650 2950
Wire Wire Line
	4600 2950 4600 2850
Wire Wire Line
	4300 2850 4600 2850
Connection ~ 4600 2950
Wire Wire Line
	3700 3250 4300 3250
Wire Wire Line
	4650 3250 4300 3250
Connection ~ 4300 3250
Text GLabel 4650 3550 3    50   Input ~ 0
5v_MCU
Text GLabel 4300 3550 3    50   Input ~ 0
5v_PI
Text GLabel 3100 2850 0    50   Input ~ 0
GND
Text GLabel 3100 2650 0    50   Input ~ 0
D7
Text GLabel 8700 2200 2    50   Input ~ 0
D7
Wire Wire Line
	3700 3050 3950 3050
Text GLabel 3100 3550 0    50   Input ~ 0
ESC1
Text GLabel 3700 3650 2    50   Input ~ 0
ESC2
Wire Wire Line
	7150 1250 7150 1000
Wire Wire Line
	7150 1000 9200 1000
Wire Wire Line
	7250 1250 7250 900 
Wire Wire Line
	7250 900  9200 900 
Wire Wire Line
	7350 1250 7350 1100
Wire Wire Line
	7350 1100 9200 1100
Text GLabel 6100 5600 0    50   Input ~ 0
3v3
Text GLabel 6100 5700 0    50   Input ~ 0
5v_MCU
$Comp
L Connector:Conn_01x04_Female J?
U 1 1 5D296C98
P 10100 3100
F 0 "J?" H 10127 3076 50  0000 L CNN
F 1 "Conn_01x04_Female" H 10127 2985 50  0000 L CNN
F 2 "" H 10100 3100 50  0001 C CNN
F 3 "~" H 10100 3100 50  0001 C CNN
	1    10100 3100
	1    0    0    -1  
$EndComp
$EndSCHEMATC