EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 10050 3800 0    50   ~ 0
Cyfrowe
Text GLabel 1350 2150 0    39   Output ~ 0
OUTPUT_0
Text GLabel 1350 2050 0    39   Output ~ 0
OUTPUT_1
Text GLabel 1350 1950 0    39   Output ~ 0
OUTPUT_2
Text GLabel 1350 1850 0    39   Output ~ 0
OUTPUT_3
Text GLabel 1350 1750 0    39   Output ~ 0
OUTPUT_4
Text GLabel 1350 2250 0    39   BiDi ~ 0
SWDIO
Text GLabel 1350 2350 0    39   BiDi ~ 0
SWDCLK
Text GLabel 6000 3800 3    39   Input ~ 0
NRST
$Comp
L karta_stm-rescue:+3.3V-power #PWR010
U 1 1 5F93A696
P 9300 5550
F 0 "#PWR010" H 9300 5400 50  0001 C CNN
F 1 "+3.3V" H 9315 5723 50  0000 C CNN
F 2 "" H 9300 5550 50  0001 C CNN
F 3 "" H 9300 5550 50  0001 C CNN
	1    9300 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 5550 9300 5550
Text GLabel 8900 5650 2    39   BiDi ~ 0
SWDCLK
$Comp
L karta_stm-rescue:GND-power #PWR011
U 1 1 5F93BC10
P 9300 5750
F 0 "#PWR011" H 9300 5500 50  0001 C CNN
F 1 "GND" H 9305 5577 50  0000 C CNN
F 2 "" H 9300 5750 50  0001 C CNN
F 3 "" H 9300 5750 50  0001 C CNN
	1    9300 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 5750 9300 5750
Text GLabel 8900 5850 2    39   BiDi ~ 0
SWDIO
Text GLabel 8900 5950 2    39   Input ~ 0
NRST
Wire Notes Line
	9800 3700 9800 5000
Text Notes 8850 5300 0    39   ~ 0
SWD
Wire Notes Line
	8400 5200 8400 6050
Wire Notes Line
	8400 6050 9500 6050
Wire Notes Line
	9500 6050 9500 5200
Wire Notes Line
	9500 5200 8400 5200
Text GLabel 5950 7050 0    39   Input ~ 0
zwora
Text GLabel 10300 4500 0    39   Output ~ 0
OUT_0
Text GLabel 10300 4400 0    39   Output ~ 0
OUT_1
Text GLabel 10300 4300 0    39   Output ~ 0
OUT_2
Text GLabel 10300 4200 0    39   Output ~ 0
OUT_3
Text GLabel 10300 4100 0    39   Output ~ 0
OUT_4
$Comp
L karta_stm-rescue:MCP2562-H-SN-Interface_CAN_LIN U2
U 1 1 5F9644A8
P 8550 2000
F 0 "U2" H 8550 2581 50  0000 C CNN
F 1 "MCP2562-H-SN" H 8550 2490 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 8550 1500 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/25167A.pdf" H 8550 2000 50  0001 C CNN
	1    8550 2000
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR014
U 1 1 5F964EFE
P 8550 2600
F 0 "#PWR014" H 8550 2350 50  0001 C CNN
F 1 "GND" H 8555 2427 50  0000 C CNN
F 2 "" H 8550 2600 50  0001 C CNN
F 3 "" H 8550 2600 50  0001 C CNN
	1    8550 2600
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+3.3V-power #PWR012
U 1 1 5F965905
P 7250 2100
F 0 "#PWR012" H 7250 1950 50  0001 C CNN
F 1 "+3.3V" H 7265 2273 50  0000 C CNN
F 2 "" H 7250 2100 50  0001 C CNN
F 3 "" H 7250 2100 50  0001 C CNN
	1    7250 2100
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR013
U 1 1 5F9660D8
P 8550 950
F 0 "#PWR013" H 8550 800 50  0001 C CNN
F 1 "+5V" H 8565 1123 50  0000 C CNN
F 2 "" H 8550 950 50  0001 C CNN
F 3 "" H 8550 950 50  0001 C CNN
	1    8550 950 
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR015
U 1 1 5F966755
P 8850 1300
F 0 "#PWR015" H 8850 1050 50  0001 C CNN
F 1 "GND" H 8855 1127 50  0000 C CNN
F 2 "" H 8850 1300 50  0001 C CNN
F 3 "" H 8850 1300 50  0001 C CNN
	1    8850 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 2400 8550 2450
Wire Wire Line
	8550 2450 8000 2450
Wire Wire Line
	8000 2450 8000 2200
Wire Wire Line
	8000 2200 8050 2200
Connection ~ 8550 2450
Wire Wire Line
	8550 2450 8550 2600
$Comp
L karta_stm-rescue:C-Device C12
U 1 1 5F969979
P 8850 1150
F 0 "C12" H 8965 1196 50  0000 L CNN
F 1 "100nF" H 8965 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8888 1000 50  0001 C CNN
F 3 "~" H 8850 1150 50  0001 C CNN
	1    8850 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 950  8550 1000
Wire Wire Line
	8550 1000 8850 1000
Connection ~ 8550 1000
Wire Wire Line
	8550 1000 8550 1600
$Comp
L karta_stm-rescue:GND-power #PWR016
U 1 1 5F976ACC
P 10050 2000
F 0 "#PWR016" H 10050 1750 50  0001 C CNN
F 1 "GND" H 10055 1827 50  0000 C CNN
F 2 "" H 10050 2000 50  0001 C CNN
F 3 "" H 10050 2000 50  0001 C CNN
	1    10050 2000
	1    0    0    -1  
$EndComp
Text GLabel 10400 1550 2    50   Input ~ 0
CAN_H
Text GLabel 10400 2300 2    50   Input ~ 0
CAN_L
Text GLabel 7850 1800 0    50   Input ~ 0
CAN_TX
Wire Wire Line
	8050 1800 7850 1800
Text GLabel 7850 1900 0    50   Output ~ 0
CAN_RX
Wire Wire Line
	8050 1900 7850 1900
Wire Wire Line
	7250 2100 8050 2100
Text GLabel 9500 3150 0    50   Input ~ 0
CAN_H
Text GLabel 9500 3050 0    50   Input ~ 0
CAN_L
$Comp
L karta_stm-rescue:C-Device C3
U 1 1 5F990AF6
P 6500 3050
F 0 "C3" H 6615 3096 50  0000 L CNN
F 1 "0.1uF" H 6615 3005 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 6538 2900 50  0001 C CNN
F 3 "~" H 6500 3050 50  0001 C CNN
	1    6500 3050
	-1   0    0    1   
$EndComp
$Comp
L karta_stm-rescue:SW_Push-Switch SW1
U 1 1 5F99169C
P 6400 3700
F 0 "SW1" H 6400 3985 50  0000 C CNN
F 1 "SW_Push" H 6400 3894 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 6400 3900 50  0001 C CNN
F 3 "~" H 6400 3900 50  0001 C CNN
	1    6400 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	5550 3250 6000 3250
$Comp
L karta_stm-rescue:R-Device R1
U 1 1 5F997D4A
P 6000 3050
F 0 "R1" V 5793 3050 50  0000 C CNN
F 1 "0R" V 5884 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 5930 3050 50  0001 C CNN
F 3 "~" H 6000 3050 50  0001 C CNN
	1    6000 3050
	0    -1   -1   0   
$EndComp
Connection ~ 6500 2900
$Comp
L karta_stm-rescue:GND-power #PWR03
U 1 1 5F99D731
P 6500 2900
F 0 "#PWR03" H 6500 2650 50  0001 C CNN
F 1 "GND" H 6505 2727 50  0000 C CNN
F 2 "" H 6500 2900 50  0001 C CNN
F 3 "" H 6500 2900 50  0001 C CNN
	1    6500 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	6000 3800 6000 3700
Connection ~ 6000 3700
$Comp
L karta_stm-rescue:+3.3V-power #PWR08
U 1 1 5F9C5C90
P 6350 7050
F 0 "#PWR08" H 6350 6900 50  0001 C CNN
F 1 "+3.3V" H 6365 7223 50  0000 C CNN
F 2 "" H 6350 7050 50  0001 C CNN
F 3 "" H 6350 7050 50  0001 C CNN
	1    6350 7050
	1    0    0    -1  
$EndComp
Text GLabel 1350 3250 0    39   Input ~ 0
zwora
$Comp
L karta_stm-rescue:Crystal_Small-Device Y1
U 1 1 5F9A88FE
P 3100 7050
F 0 "Y1" V 3054 7138 50  0000 L CNN
F 1 "16MHz" V 3145 7138 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_HC49-SD" H 3100 7050 50  0001 C CNN
F 3 "~" H 3100 7050 50  0001 C CNN
	1    3100 7050
	0    -1   1    0   
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR02
U 1 1 5F9ACF14
P 4250 7450
F 0 "#PWR02" H 4250 7200 50  0001 C CNN
F 1 "GND" H 4255 7277 50  0000 C CNN
F 2 "" H 4250 7450 50  0001 C CNN
F 3 "" H 4250 7450 50  0001 C CNN
	1    4250 7450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4250 7450 4250 7400
Wire Wire Line
	4250 7400 3950 7400
Wire Wire Line
	3100 7150 2500 7150
$Comp
L karta_stm-rescue:PESD2CAN,215-PESD2CAN_215 D1
U 1 1 5F9BE300
P 9650 2000
F 0 "D1" H 9650 2367 50  0000 C CNN
F 1 "PESD2CAN,215" H 9650 2276 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9650 2000 50  0001 L BNN
F 3 "NXP" H 9650 2000 50  0001 L BNN
	1    9650 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 1550 9250 1900
Wire Wire Line
	9050 1900 9250 1900
Connection ~ 9250 1900
Wire Wire Line
	9250 2100 9050 2100
Wire Wire Line
	9250 2300 9250 2100
Connection ~ 9250 2100
$Comp
L karta_stm-rescue:R-Device R2
U 1 1 5F9F2A59
P 6100 7050
F 0 "R2" V 5893 7050 50  0000 C CNN
F 1 "10k Ohm" V 5984 7050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6030 7050 50  0001 C CNN
F 3 "~" H 6100 7050 50  0001 C CNN
	1    6100 7050
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 7050 6350 7050
Wire Notes Line
	10950 3700 10950 5000
Wire Notes Line
	9800 3700 10950 3700
$Comp
L karta_stm-rescue:Jumper_2_Bridged-Jumper JP2
U 1 1 5FA6F978
P 10300 1750
F 0 "JP2" V 10254 1808 50  0000 L CNN
F 1 "Jumper_2_Bridged" V 10345 1808 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 10300 1750 50  0001 C CNN
F 3 "~" H 10300 1750 50  0001 C CNN
	1    10300 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	9250 1550 10300 1550
Connection ~ 10300 1550
Wire Wire Line
	10300 1550 10400 1550
$Comp
L karta_stm-rescue:Jumper_2_Bridged-Jumper JP1
U 1 1 5FAB1B9D
P 5950 7250
F 0 "JP1" V 5904 7308 50  0000 L CNN
F 1 "Jumper_2_Bridged" V 5995 7308 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5950 7250 50  0001 C CNN
F 3 "~" H 5950 7250 50  0001 C CNN
	1    5950 7250
	0    1    1    0   
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR07
U 1 1 5FAB29CE
P 5950 7450
F 0 "#PWR07" H 5950 7200 50  0001 C CNN
F 1 "GND" H 5955 7277 50  0000 C CNN
F 2 "" H 5950 7450 50  0001 C CNN
F 3 "" H 5950 7450 50  0001 C CNN
	1    5950 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 7000 4250 7050
Wire Wire Line
	3950 7000 4250 7000
$Comp
L karta_stm-rescue:GND-power #PWR01
U 1 1 5F9AC879
P 4250 7050
F 0 "#PWR01" H 4250 6800 50  0001 C CNN
F 1 "GND" H 4255 6877 50  0000 C CNN
F 2 "" H 4250 7050 50  0001 C CNN
F 3 "" H 4250 7050 50  0001 C CNN
	1    4250 7050
	-1   0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:C-Device C2
U 1 1 5F9AC11A
P 3800 7400
F 0 "C2" V 4052 7400 50  0000 C CNN
F 1 "20pF" V 3961 7400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3838 7250 50  0001 C CNN
F 3 "~" H 3800 7400 50  0001 C CNN
	1    3800 7400
	0    1    -1   0   
$EndComp
$Comp
L karta_stm-rescue:C-Device C1
U 1 1 5F9AB4E1
P 3800 7000
F 0 "C1" V 3548 7000 50  0000 C CNN
F 1 "20pF" V 3639 7000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3838 6850 50  0001 C CNN
F 3 "~" H 3800 7000 50  0001 C CNN
	1    3800 7000
	0    -1   1    0   
$EndComp
Wire Wire Line
	6950 2900 6500 2900
Wire Wire Line
	3100 6950 3650 6950
Wire Wire Line
	3650 6950 3650 7000
Wire Wire Line
	3100 7150 3650 7150
Wire Wire Line
	3650 7150 3650 7400
Connection ~ 3100 7150
$Comp
L karta_stm-rescue:Conn_01x06_Male-Connector J2
U 1 1 5FAB16FB
P 8700 5750
F 0 "J2" H 8808 6131 50  0000 C CNN
F 1 "Conn_01x06_Male" H 8808 6040 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 8700 5750 50  0001 C CNN
F 3 "~" H 8700 5750 50  0001 C CNN
	1    8700 5750
	1    0    0    -1  
$EndComp
Text GLabel 8900 6050 2    39   BiDi ~ 0
SWO
Text GLabel 1350 2950 0    39   BiDi ~ 0
SWO
Wire Wire Line
	9250 2300 10300 2300
$Comp
L karta_stm-rescue:R-Device R3
U 1 1 5F97AB35
P 10300 2100
F 0 "R3" H 10370 2146 50  0000 L CNN
F 1 "120 Ohm" H 10370 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10230 2100 50  0001 C CNN
F 3 "~" H 10300 2100 50  0001 C CNN
	1    10300 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 2250 10300 2300
Connection ~ 10300 2300
Wire Wire Line
	10300 2300 10400 2300
$Comp
L karta_stm-rescue:MountingHole-Mechanical H1
U 1 1 5FC619F5
P 700 7250
F 0 "H1" H 800 7296 50  0000 L CNN
F 1 "MountingHole" H 800 7205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 700 7250 50  0001 C CNN
F 3 "~" H 700 7250 50  0001 C CNN
	1    700  7250
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:MountingHole-Mechanical H4
U 1 1 5FC62609
P 1250 7250
F 0 "H4" H 1350 7296 50  0000 L CNN
F 1 "MountingHole" H 1350 7205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 1250 7250 50  0001 C CNN
F 3 "~" H 1250 7250 50  0001 C CNN
	1    1250 7250
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:MountingHole-Mechanical H2
U 1 1 5FC62ABF
P 800 7550
F 0 "H2" H 900 7596 50  0000 L CNN
F 1 "MountingHole" H 900 7505 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 800 7550 50  0001 C CNN
F 3 "~" H 800 7550 50  0001 C CNN
	1    800  7550
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:MountingHole-Mechanical H3
U 1 1 5FC630E8
P 1200 7550
F 0 "H3" H 1300 7596 50  0000 L CNN
F 1 "MountingHole" H 1300 7505 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 1200 7550 50  0001 C CNN
F 3 "~" H 1200 7550 50  0001 C CNN
	1    1200 7550
	1    0    0    -1  
$EndComp
Text GLabel 10500 5750 2    50   Input ~ 0
UART_TX
Text GLabel 10500 5850 2    50   Output ~ 0
UART_RX
$Comp
L karta_stm-rescue:GND-power #PWR0101
U 1 1 5FC7CFC1
P 10200 5950
F 0 "#PWR0101" H 10200 5700 50  0001 C CNN
F 1 "GND" H 10205 5777 50  0000 C CNN
F 2 "" H 10200 5950 50  0001 C CNN
F 3 "" H 10200 5950 50  0001 C CNN
	1    10200 5950
	1    0    0    -1  
$EndComp
Wire Notes Line
	10950 5000 9800 5000
Wire Notes Line
	9900 5200 9900 6200
Wire Notes Line
	9900 6200 10950 6200
Wire Notes Line
	10950 6200 10950 5200
Wire Notes Line
	10950 5200 9900 5200
Wire Notes Line
	7050 3400 11150 3400
Wire Notes Line
	11150 3400 11150 550 
Wire Notes Line
	11150 550  7050 550 
Wire Notes Line
	7050 550  7050 3400
Wire Notes Line
	6850 7700 5350 7700
Wire Notes Line
	5350 7700 5350 6650
Wire Notes Line
	5350 6650 6850 6650
Wire Notes Line
	6850 6800 6800 6800
Wire Notes Line
	6850 6650 6850 7700
Text Notes 8900 750  0    79   ~ 0
Magistrala CAN
Text Notes 10250 5350 0    79   ~ 0
USART
Text Notes 5900 6750 0    79   ~ 0
Zworka
Text GLabel 1350 2850 0    39   Input ~ 0
INPUT_0
Text GLabel 1350 2750 0    39   Input ~ 0
INPUT_1
Text GLabel 5550 1450 2    39   Input ~ 0
INPUT_2
Text GLabel 5550 1350 2    39   Input ~ 0
INPUT_3
Text GLabel 5550 2250 2    39   Output ~ 0
DIODE_0
Text GLabel 2500 7550 0    39   Output ~ 0
DIODE_0
$Comp
L Device:D D?
U 1 1 6094085E
P 2800 7550
AR Path="/5F916EBE/6094085E" Ref="D?"  Part="1" 
AR Path="/5F905B56/6094085E" Ref="D11"  Part="1" 
F 0 "D11" V 2846 7470 50  0000 R CNN
F 1 "D" V 2755 7470 50  0000 R CNN
F 2 "Diode_SMD:D_1210_3225Metric" H 2800 7550 50  0001 C CNN
F 3 "~" H 2800 7550 50  0001 C CNN
	1    2800 7550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 60940864
P 3100 7550
AR Path="/5F916EBE/60940864" Ref="R?"  Part="1" 
AR Path="/5F905B56/60940864" Ref="R48"  Part="1" 
F 0 "R48" H 3170 7596 50  0000 L CNN
F 1 "1kOhm" H 3170 7505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3030 7550 50  0001 C CNN
F 3 "~" H 3100 7550 50  0001 C CNN
	1    3100 7550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2650 7550 2500 7550
$Comp
L karta_stm-rescue:GND-power #PWR0102
U 1 1 6094A0DF
P 3250 7550
F 0 "#PWR0102" H 3250 7300 50  0001 C CNN
F 1 "GND" H 3255 7377 50  0000 C CNN
F 2 "" H 3250 7550 50  0001 C CNN
F 3 "" H 3250 7550 50  0001 C CNN
	1    3250 7550
	0    -1   -1   0   
$EndComp
$Comp
L karta_stm-rescue:STM32L4P5RET6-2021-04-28_16-02-19 U1
U 1 1 6089C0EC
P 1350 950
AR Path="/6089C0EC" Ref="U1"  Part="1" 
AR Path="/5F905B56/6089C0EC" Ref="U1"  Part="1" 
F 0 "U1" H 3450 1337 60  0000 C CNN
F 1 "STM32L4P5RET6" H 3450 1231 60  0000 C CNN
F 2 "stm32l4p:STM32L4P5RET6" H 3450 1190 60  0001 C CNN
F 3 "" H 1350 950 60  0000 C CNN
	1    1350 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3700 6200 3700
Wire Wire Line
	5550 3050 5850 3050
Wire Wire Line
	6000 3250 6000 3700
Wire Wire Line
	6600 3700 6950 3700
Wire Wire Line
	6950 2900 6950 3700
Wire Wire Line
	6500 3200 6500 3400
Wire Wire Line
	6500 3400 6200 3400
Wire Wire Line
	6200 3400 6200 3700
Connection ~ 6200 3700
Wire Wire Line
	6150 2900 6150 3050
Wire Wire Line
	6150 2900 6500 2900
Text GLabel 5550 2850 2    39   Input ~ 0
OSC_IN
Text GLabel 2400 6950 0    39   Input ~ 0
OSC_IN
Text GLabel 5550 2950 2    39   Output ~ 0
OSC_OUT
Text GLabel 2500 7150 0    39   Output ~ 0
OSC_OUT
Wire Wire Line
	2400 6950 3100 6950
Connection ~ 3100 6950
Text GLabel 5550 950  2    39   Input ~ 0
ADC_4
Text GLabel 5550 1150 2    39   Input ~ 0
ADC_3
Text GLabel 5550 1050 2    39   Input ~ 0
ADC_2
Text GLabel 5550 1250 2    39   Input ~ 0
ADC_1
Text GLabel 1350 950  0    39   Input ~ 0
ADC_0
Text GLabel 5550 1950 2    50   Output ~ 0
UART_TX
Text GLabel 5550 2050 2    50   Input ~ 0
UART_RX
$Comp
L karta_stm-rescue:GND-power #PWR0103
U 1 1 6090F369
P 1750 6050
F 0 "#PWR0103" H 1750 5800 50  0001 C CNN
F 1 "GND" H 1755 5877 50  0000 C CNN
F 2 "" H 1750 6050 50  0001 C CNN
F 3 "" H 1750 6050 50  0001 C CNN
	1    1750 6050
	1    0    0    -1  
$EndComp
Connection ~ 1750 5750
Wire Wire Line
	1750 5750 1500 5750
Wire Wire Line
	850  5550 850  5850
Connection ~ 850  5550
Wire Wire Line
	1750 5550 850  5550
Wire Wire Line
	850  5350 850  5550
Connection ~ 850  5350
Wire Wire Line
	1750 5350 850  5350
Wire Wire Line
	850  5250 850  5350
Connection ~ 850  5250
Wire Wire Line
	1750 5250 850  5250
Wire Wire Line
	850  5150 850  5250
Connection ~ 850  5150
Wire Wire Line
	1750 5150 850  5150
Wire Wire Line
	850  5050 1750 5050
Wire Wire Line
	850  5050 850  5150
Connection ~ 4700 5250
Wire Wire Line
	4700 5450 4700 5250
Wire Wire Line
	4350 5450 4700 5450
Wire Wire Line
	4700 5150 5500 5150
Wire Wire Line
	5500 5450 5500 5650
Wire Wire Line
	4350 5650 5500 5650
Wire Wire Line
	4700 5250 4700 5150
Wire Wire Line
	4350 5250 4700 5250
Connection ~ 4700 5150
Wire Wire Line
	4700 5150 4350 5150
Wire Wire Line
	4700 5050 4350 5050
Wire Wire Line
	4700 5150 4700 5050
$Comp
L karta_stm-rescue:STM32L4P5RET6-2021-04-28_16-02-19 U1
U 2 1 608A130B
P 1750 5050
AR Path="/608A130B" Ref="U1"  Part="2" 
AR Path="/5F905B56/608A130B" Ref="U1"  Part="2" 
F 0 "U1" H 3050 5437 60  0000 C CNN
F 1 "STM32L4P5RET6" H 3050 5331 60  0000 C CNN
F 2 "stm32l4p:STM32L4P5RET6" H 3850 5290 60  0001 C CNN
F 3 "" H 1750 5050 60  0000 C CNN
	2    1750 5050
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR05
U 1 1 5FA9C918
P 5500 5950
F 0 "#PWR05" H 5500 5700 50  0001 C CNN
F 1 "GND" H 5505 5777 50  0000 C CNN
F 2 "" H 5500 5950 50  0001 C CNN
F 3 "" H 5500 5950 50  0001 C CNN
	1    5500 5950
	1    0    0    -1  
$EndComp
Connection ~ 6950 5450
Connection ~ 6950 5150
$Comp
L karta_stm-rescue:L-Device L1
U 1 1 5F95F00A
P 5500 5300
F 0 "L1" H 5553 5346 50  0000 L CNN
F 1 "L" H 5553 5255 50  0000 L CNN
F 2 "Inductor_SMD:L_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5500 5300 50  0001 C CNN
F 3 "~" H 5500 5300 50  0001 C CNN
	1    5500 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 5450 6950 5450
Connection ~ 7350 5450
Wire Wire Line
	7750 5450 7350 5450
Wire Wire Line
	7350 5150 6950 5150
Connection ~ 7350 5150
Wire Wire Line
	7750 5150 7350 5150
$Comp
L karta_stm-rescue:C_Polarized-Device C11
U 1 1 5F915B02
P 7750 5300
F 0 "C11" H 7868 5346 50  0000 L CNN
F 1 "1uF" H 7868 5255 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_4x5.7" H 7788 5150 50  0001 C CNN
F 3 "~" H 7750 5300 50  0001 C CNN
	1    7750 5300
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:C-Device C10
U 1 1 5F9153C7
P 7350 5300
F 0 "C10" H 7465 5346 50  0000 L CNN
F 1 "10nF" H 7465 5255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 7388 5150 50  0001 C CNN
F 3 "~" H 7350 5300 50  0001 C CNN
	1    7350 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 5450 6950 5450
Connection ~ 6500 5450
Wire Wire Line
	6150 5450 6500 5450
Connection ~ 6150 5450
Wire Wire Line
	5750 5450 6150 5450
Wire Wire Line
	6500 5150 6150 5150
Connection ~ 6500 5150
Wire Wire Line
	6150 5150 5750 5150
Connection ~ 6150 5150
Wire Wire Line
	6950 5150 6750 5150
$Comp
L karta_stm-rescue:C_Polarized-Device C9
U 1 1 5F913ABD
P 6950 5300
F 0 "C9" H 7068 5346 50  0000 L CNN
F 1 "4,7uF" H 7068 5255 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-6032-15_Kemet-U" H 6988 5150 50  0001 C CNN
F 3 "~" H 6950 5300 50  0001 C CNN
	1    6950 5300
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:C-Device C8
U 1 1 5F9131E5
P 6500 5300
F 0 "C8" H 6615 5346 50  0000 L CNN
F 1 "100nF" H 6615 5255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 6538 5150 50  0001 C CNN
F 3 "~" H 6500 5300 50  0001 C CNN
	1    6500 5300
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:C-Device C7
U 1 1 5F912B6D
P 6150 5300
F 0 "C7" H 6265 5346 50  0000 L CNN
F 1 "100nF" H 6265 5255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 6188 5150 50  0001 C CNN
F 3 "~" H 6150 5300 50  0001 C CNN
	1    6150 5300
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:C-Device C6
U 1 1 5F912174
P 5750 5300
F 0 "C6" H 5865 5346 50  0000 L CNN
F 1 "100nF" H 5865 5255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5788 5150 50  0001 C CNN
F 3 "~" H 5750 5300 50  0001 C CNN
	1    5750 5300
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:C-Device C5
U 1 1 5F911B39
P 1750 5900
F 0 "C5" H 1865 5946 50  0000 L CNN
F 1 "100nF" H 1865 5855 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 1788 5750 50  0001 C CNN
F 3 "~" H 1750 5900 50  0001 C CNN
	1    1750 5900
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:C-Device C4
U 1 1 5F91105D
P 5500 5800
F 0 "C4" H 5615 5846 50  0000 L CNN
F 1 "100nF" H 5615 5755 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5538 5650 50  0001 C CNN
F 3 "~" H 5500 5800 50  0001 C CNN
	1    5500 5800
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR09
U 1 1 5F910BD3
P 6500 5450
F 0 "#PWR09" H 6500 5200 50  0001 C CNN
F 1 "GND" H 6505 5277 50  0000 C CNN
F 2 "" H 6500 5450 50  0001 C CNN
F 3 "" H 6500 5450 50  0001 C CNN
	1    6500 5450
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+3.3V-power #PWR06
U 1 1 5F91057C
P 1500 5750
F 0 "#PWR06" H 1500 5600 50  0001 C CNN
F 1 "+3.3V" H 1515 5923 50  0000 C CNN
F 2 "" H 1500 5750 50  0001 C CNN
F 3 "" H 1500 5750 50  0001 C CNN
	1    1500 5750
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR04
U 1 1 5F908191
P 850 5850
F 0 "#PWR04" H 850 5600 50  0001 C CNN
F 1 "GND" H 855 5677 50  0000 C CNN
F 2 "" H 850 5850 50  0001 C CNN
F 3 "" H 850 5850 50  0001 C CNN
	1    850  5850
	1    0    0    -1  
$EndComp
Connection ~ 5500 5650
$Comp
L karta_stm-rescue:+3.3V-power #PWR0104
U 1 1 609CA708
P 6750 5050
F 0 "#PWR0104" H 6750 4900 50  0001 C CNN
F 1 "+3.3V" H 6765 5223 50  0000 C CNN
F 2 "" H 6750 5050 50  0001 C CNN
F 3 "" H 6750 5050 50  0001 C CNN
	1    6750 5050
	1    0    0    -1  
$EndComp
Connection ~ 6750 5150
Wire Wire Line
	6750 5150 6500 5150
Wire Wire Line
	5750 5150 5500 5150
Connection ~ 5750 5150
Connection ~ 5500 5150
Text GLabel 1350 3550 0    50   Input ~ 0
CAN_TX
Text GLabel 1350 3450 0    50   Output ~ 0
CAN_RX
Wire Wire Line
	6750 5050 6750 5150
$Comp
L Device:R R50
U 1 1 60933271
P 10350 5750
F 0 "R50" V 10143 5750 50  0000 C CNN
F 1 "R" V 10234 5750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10280 5750 50  0001 C CNN
F 3 "~" H 10350 5750 50  0001 C CNN
	1    10350 5750
	0    1    1    0   
$EndComp
$Comp
L Device:R R51
U 1 1 6093FB6C
P 10350 5850
F 0 "R51" V 10143 5850 50  0000 C CNN
F 1 "R" V 10234 5850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10280 5850 50  0001 C CNN
F 3 "~" H 10350 5850 50  0001 C CNN
	1    10350 5850
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x03_Male J7
U 1 1 60945847
P 10000 5850
F 0 "J7" H 10108 6131 50  0000 C CNN
F 1 "Conn_01x03_Male" H 10108 6040 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10000 5850 50  0001 C CNN
F 3 "~" H 10000 5850 50  0001 C CNN
	1    10000 5850
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x05_Female J3
U 1 1 609611C1
P 10500 4300
F 0 "J3" H 10528 4326 50  0000 L CNN
F 1 "Conn_01x05_Female" H 10528 4235 50  0000 L CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43650-0500_1x05_P3.00mm_Horizontal" H 10500 4300 50  0001 C CNN
F 3 "~" H 10500 4300 50  0001 C CNN
	1    10500 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J1
U 1 1 60965CA5
P 9700 3050
F 0 "J1" H 9728 3026 50  0000 L CNN
F 1 "Conn_01x02_Female" H 9728 2935 50  0000 L CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43650-0200_1x02_P3.00mm_Horizontal" H 9700 3050 50  0001 C CNN
F 3 "~" H 9700 3050 50  0001 C CNN
	1    9700 3050
	1    0    0    -1  
$EndComp
$EndSCHEMATC
