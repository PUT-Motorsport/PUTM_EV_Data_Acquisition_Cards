EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 1250 1850 0    39   Input ~ 0
ADC_IN0
Text GLabel 1250 1750 0    39   Input ~ 0
ADC_IN1
Text GLabel 1250 1650 0    39   Input ~ 0
ADC_IN2
Text GLabel 1250 1550 0    39   Input ~ 0
ADC_IN3
Text GLabel 1250 1450 0    39   Input ~ 0
ADC_IN4
Text GLabel 3250 1500 0    39   Input ~ 0
ADC_IN0
Text GLabel 5400 1900 2    39   Input ~ 0
ADC_0
$Comp
L karta_stm-rescue:GND-power #PWR042
U 1 1 5F9386D7
P 4000 1950
F 0 "#PWR042" H 4000 1700 50  0001 C CNN
F 1 "GND" H 4005 1777 50  0000 C CNN
F 2 "" H 4000 1950 50  0001 C CNN
F 3 "" H 4000 1950 50  0001 C CNN
	1    4000 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1950 4000 1900
Wire Wire Line
	3800 1700 3800 2250
Wire Wire Line
	3800 2250 3950 2250
Wire Wire Line
	4450 2250 4450 1600
Wire Wire Line
	4450 1600 4400 1600
$Comp
L karta_stm-rescue:R-Device R12
U 1 1 5F93ABB4
P 5000 1750
F 0 "R12" H 5070 1796 50  0000 L CNN
F 1 "5.1k" H 5070 1705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4930 1750 50  0001 C CNN
F 3 "~" H 5000 1750 50  0001 C CNN
	1    5000 1750
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R13
U 1 1 5F93B3E5
P 5000 2100
F 0 "R13" H 5070 2146 50  0000 L CNN
F 1 "10k" H 5070 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4930 2100 50  0001 C CNN
F 3 "~" H 5000 2100 50  0001 C CNN
	1    5000 2100
	1    0    0    -1  
$EndComp
Connection ~ 4450 1600
Wire Wire Line
	5000 1900 5000 1950
$Comp
L karta_stm-rescue:GND-power #PWR048
U 1 1 5F93C40C
P 5000 2250
F 0 "#PWR048" H 5000 2000 50  0001 C CNN
F 1 "GND" H 5005 2077 50  0000 C CNN
F 2 "" H 5000 2250 50  0001 C CNN
F 3 "" H 5000 2250 50  0001 C CNN
	1    5000 2250
	1    0    0    -1  
$EndComp
Connection ~ 5000 1900
$Comp
L karta_stm-rescue:R-Device R9
U 1 1 5F989343
P 4100 2250
F 0 "R9" V 3893 2250 50  0000 C CNN
F 1 "11k" V 3984 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4030 2250 50  0001 C CNN
F 3 "~" H 4100 2250 50  0001 C CNN
	1    4100 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 2250 4450 2250
Wire Wire Line
	5200 2200 5200 2350
$Comp
L karta_stm-rescue:GND-power #PWR049
U 1 1 5F946058
P 5200 2350
F 0 "#PWR049" H 5200 2100 50  0001 C CNN
F 1 "GND" H 5205 2177 50  0000 C CNN
F 2 "" H 5200 2350 50  0001 C CNN
F 3 "" H 5200 2350 50  0001 C CNN
	1    5200 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1900 5000 1900
$Comp
L karta_stm-rescue:C-Device C19
U 1 1 5F937580
P 5200 2050
F 0 "C19" H 5315 2096 50  0000 L CNN
F 1 "0.1uF" H 5315 2005 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5238 1900 50  0001 C CNN
F 3 "~" H 5200 2050 50  0001 C CNN
	1    5200 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1900 5200 1900
Connection ~ 5200 1900
$Comp
L karta_stm-rescue:R-Device R8
U 1 1 5FA2ADEA
P 3650 1950
F 0 "R8" H 3720 1996 50  0000 L CNN
F 1 "10k Ohm" H 3720 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3580 1950 50  0001 C CNN
F 3 "~" H 3650 1950 50  0001 C CNN
	1    3650 1950
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR040
U 1 1 5FA2B9B9
P 3650 2100
F 0 "#PWR040" H 3650 1850 50  0001 C CNN
F 1 "GND" H 3655 1927 50  0000 C CNN
F 2 "" H 3650 2100 50  0001 C CNN
F 3 "" H 3650 2100 50  0001 C CNN
	1    3650 2100
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR039
U 1 1 5FA3839C
P 3350 1800
F 0 "#PWR039" H 3350 1550 50  0001 C CNN
F 1 "GND" H 3355 1627 50  0000 C CNN
F 2 "" H 3350 1800 50  0001 C CNN
F 3 "" H 3350 1800 50  0001 C CNN
	1    3350 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1600 5000 1600
Text GLabel 6050 1550 0    39   Input ~ 0
ADC_IN1
Text GLabel 8200 1950 2    39   Input ~ 0
ADC_1
$Comp
L karta_stm-rescue:GND-power #PWR056
U 1 1 5FA5AC8E
P 6800 2000
F 0 "#PWR056" H 6800 1750 50  0001 C CNN
F 1 "GND" H 6805 1827 50  0000 C CNN
F 2 "" H 6800 2000 50  0001 C CNN
F 3 "" H 6800 2000 50  0001 C CNN
	1    6800 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 2000 6800 1950
Wire Wire Line
	6600 1750 6600 2300
Wire Wire Line
	6600 2300 6750 2300
Wire Wire Line
	7250 2300 7250 1650
Wire Wire Line
	7250 1650 7200 1650
$Comp
L karta_stm-rescue:R-Device R18
U 1 1 5FA5AC99
P 7800 1800
F 0 "R18" H 7870 1846 50  0000 L CNN
F 1 "5.1k" H 7870 1755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7730 1800 50  0001 C CNN
F 3 "~" H 7800 1800 50  0001 C CNN
	1    7800 1800
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R19
U 1 1 5FA5AC9F
P 7800 2150
F 0 "R19" H 7870 2196 50  0000 L CNN
F 1 "10k" H 7870 2105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7730 2150 50  0001 C CNN
F 3 "~" H 7800 2150 50  0001 C CNN
	1    7800 2150
	1    0    0    -1  
$EndComp
Connection ~ 7250 1650
Wire Wire Line
	7800 1950 7800 2000
$Comp
L karta_stm-rescue:GND-power #PWR057
U 1 1 5FA5ACA7
P 7800 2300
F 0 "#PWR057" H 7800 2050 50  0001 C CNN
F 1 "GND" H 7805 2127 50  0000 C CNN
F 2 "" H 7800 2300 50  0001 C CNN
F 3 "" H 7800 2300 50  0001 C CNN
	1    7800 2300
	1    0    0    -1  
$EndComp
Connection ~ 7800 1950
$Comp
L karta_stm-rescue:R-Device R17
U 1 1 5FA5ACB4
P 6900 2300
F 0 "R17" V 6693 2300 50  0000 C CNN
F 1 "11k" V 6784 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6830 2300 50  0001 C CNN
F 3 "~" H 6900 2300 50  0001 C CNN
	1    6900 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 2300 7250 2300
Wire Wire Line
	8000 2250 8000 2400
$Comp
L karta_stm-rescue:GND-power #PWR058
U 1 1 5FA5ACBC
P 8000 2400
F 0 "#PWR058" H 8000 2150 50  0001 C CNN
F 1 "GND" H 8005 2227 50  0000 C CNN
F 2 "" H 8000 2400 50  0001 C CNN
F 3 "" H 8000 2400 50  0001 C CNN
	1    8000 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 1950 7800 1950
$Comp
L karta_stm-rescue:C-Device C21
U 1 1 5FA5ACC3
P 8000 2100
F 0 "C21" H 8115 2146 50  0000 L CNN
F 1 "0.1uF" H 8115 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8038 1950 50  0001 C CNN
F 3 "~" H 8000 2100 50  0001 C CNN
	1    8000 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 1950 8000 1950
Connection ~ 8000 1950
$Comp
L karta_stm-rescue:R-Device R16
U 1 1 5FA5ACCB
P 6450 1700
F 0 "R16" H 6520 1746 50  0000 L CNN
F 1 "10k Ohm" H 6520 1655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6380 1700 50  0001 C CNN
F 3 "~" H 6450 1700 50  0001 C CNN
	1    6450 1700
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR054
U 1 1 5FA5ACD1
P 6450 1850
F 0 "#PWR054" H 6450 1600 50  0001 C CNN
F 1 "GND" H 6455 1677 50  0000 C CNN
F 2 "" H 6450 1850 50  0001 C CNN
F 3 "" H 6450 1850 50  0001 C CNN
	1    6450 1850
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR053
U 1 1 5FA5ACE4
P 6150 1850
F 0 "#PWR053" H 6150 1600 50  0001 C CNN
F 1 "GND" H 6155 1677 50  0000 C CNN
F 2 "" H 6150 1850 50  0001 C CNN
F 3 "" H 6150 1850 50  0001 C CNN
	1    6150 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 1650 7800 1650
Text GLabel 8600 1550 0    39   Input ~ 0
ADC_IN2
Text GLabel 10750 1950 2    39   Input ~ 0
ADC_2
$Comp
L karta_stm-rescue:GND-power #PWR063
U 1 1 5FA5FA56
P 9350 2000
F 0 "#PWR063" H 9350 1750 50  0001 C CNN
F 1 "GND" H 9355 1827 50  0000 C CNN
F 2 "" H 9350 2000 50  0001 C CNN
F 3 "" H 9350 2000 50  0001 C CNN
	1    9350 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 2000 9350 1950
Wire Wire Line
	9150 1750 9150 2300
Wire Wire Line
	9150 2300 9300 2300
Wire Wire Line
	9800 2300 9800 1650
Wire Wire Line
	9800 1650 9750 1650
$Comp
L karta_stm-rescue:R-Device R22
U 1 1 5FA5FA61
P 10350 1800
F 0 "R22" H 10420 1846 50  0000 L CNN
F 1 "5.1k" H 10420 1755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10280 1800 50  0001 C CNN
F 3 "~" H 10350 1800 50  0001 C CNN
	1    10350 1800
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R23
U 1 1 5FA5FA67
P 10350 2150
F 0 "R23" H 10420 2196 50  0000 L CNN
F 1 "10k" H 10420 2105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10280 2150 50  0001 C CNN
F 3 "~" H 10350 2150 50  0001 C CNN
	1    10350 2150
	1    0    0    -1  
$EndComp
Connection ~ 9800 1650
Wire Wire Line
	10350 1950 10350 2000
$Comp
L karta_stm-rescue:GND-power #PWR064
U 1 1 5FA5FA6F
P 10350 2300
F 0 "#PWR064" H 10350 2050 50  0001 C CNN
F 1 "GND" H 10355 2127 50  0000 C CNN
F 2 "" H 10350 2300 50  0001 C CNN
F 3 "" H 10350 2300 50  0001 C CNN
	1    10350 2300
	1    0    0    -1  
$EndComp
Connection ~ 10350 1950
$Comp
L karta_stm-rescue:R-Device R21
U 1 1 5FA5FA7C
P 9450 2300
F 0 "R21" V 9243 2300 50  0000 C CNN
F 1 "11k" V 9334 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9380 2300 50  0001 C CNN
F 3 "~" H 9450 2300 50  0001 C CNN
	1    9450 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	9600 2300 9800 2300
Wire Wire Line
	10550 2250 10550 2400
$Comp
L karta_stm-rescue:GND-power #PWR065
U 1 1 5FA5FA84
P 10550 2400
F 0 "#PWR065" H 10550 2150 50  0001 C CNN
F 1 "GND" H 10555 2227 50  0000 C CNN
F 2 "" H 10550 2400 50  0001 C CNN
F 3 "" H 10550 2400 50  0001 C CNN
	1    10550 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 1950 10350 1950
$Comp
L karta_stm-rescue:C-Device C22
U 1 1 5FA5FA8B
P 10550 2100
F 0 "C22" H 10665 2146 50  0000 L CNN
F 1 "0.1uF" H 10665 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 10588 1950 50  0001 C CNN
F 3 "~" H 10550 2100 50  0001 C CNN
	1    10550 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 1950 10550 1950
Connection ~ 10550 1950
$Comp
L karta_stm-rescue:GND-power #PWR060
U 1 1 5FA5FAAC
P 8700 1850
F 0 "#PWR060" H 8700 1600 50  0001 C CNN
F 1 "GND" H 8705 1677 50  0000 C CNN
F 2 "" H 8700 1850 50  0001 C CNN
F 3 "" H 8700 1850 50  0001 C CNN
	1    8700 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 1650 10350 1650
Text GLabel 1200 3200 0    39   Input ~ 0
ADC_IN3
Text GLabel 3350 3600 2    39   Input ~ 0
ADC_3
$Comp
L karta_stm-rescue:GND-power #PWR035
U 1 1 5FA629D1
P 1950 3650
F 0 "#PWR035" H 1950 3400 50  0001 C CNN
F 1 "GND" H 1955 3477 50  0000 C CNN
F 2 "" H 1950 3650 50  0001 C CNN
F 3 "" H 1950 3650 50  0001 C CNN
	1    1950 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3650 1950 3600
Wire Wire Line
	1750 3400 1750 3950
Wire Wire Line
	1750 3950 1900 3950
Wire Wire Line
	2400 3950 2400 3300
Wire Wire Line
	2400 3300 2350 3300
$Comp
L karta_stm-rescue:R-Device R6
U 1 1 5FA629DC
P 2950 3450
F 0 "R6" H 3020 3496 50  0000 L CNN
F 1 "5.1k" H 3020 3405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2880 3450 50  0001 C CNN
F 3 "~" H 2950 3450 50  0001 C CNN
	1    2950 3450
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R7
U 1 1 5FA629E2
P 2950 3800
F 0 "R7" H 3020 3846 50  0000 L CNN
F 1 "10k" H 3020 3755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2880 3800 50  0001 C CNN
F 3 "~" H 2950 3800 50  0001 C CNN
	1    2950 3800
	1    0    0    -1  
$EndComp
Connection ~ 2400 3300
Wire Wire Line
	2950 3600 2950 3650
$Comp
L karta_stm-rescue:GND-power #PWR036
U 1 1 5FA629EA
P 2950 3950
F 0 "#PWR036" H 2950 3700 50  0001 C CNN
F 1 "GND" H 2955 3777 50  0000 C CNN
F 2 "" H 2950 3950 50  0001 C CNN
F 3 "" H 2950 3950 50  0001 C CNN
	1    2950 3950
	1    0    0    -1  
$EndComp
Connection ~ 2950 3600
$Comp
L karta_stm-rescue:R-Device R5
U 1 1 5FA629F7
P 2050 3950
F 0 "R5" V 1843 3950 50  0000 C CNN
F 1 "11k" V 1934 3950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1980 3950 50  0001 C CNN
F 3 "~" H 2050 3950 50  0001 C CNN
	1    2050 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	2200 3950 2400 3950
Wire Wire Line
	3150 3900 3150 4050
$Comp
L karta_stm-rescue:GND-power #PWR037
U 1 1 5FA629FF
P 3150 4050
F 0 "#PWR037" H 3150 3800 50  0001 C CNN
F 1 "GND" H 3155 3877 50  0000 C CNN
F 2 "" H 3150 4050 50  0001 C CNN
F 3 "" H 3150 4050 50  0001 C CNN
	1    3150 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3600 2950 3600
$Comp
L karta_stm-rescue:C-Device C18
U 1 1 5FA62A06
P 3150 3750
F 0 "C18" H 3265 3796 50  0000 L CNN
F 1 "0.1uF" H 3265 3705 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3188 3600 50  0001 C CNN
F 3 "~" H 3150 3750 50  0001 C CNN
	1    3150 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 3600 3150 3600
Connection ~ 3150 3600
$Comp
L karta_stm-rescue:GND-power #PWR032
U 1 1 5FA62A27
P 1300 3500
F 0 "#PWR032" H 1300 3250 50  0001 C CNN
F 1 "GND" H 1305 3327 50  0000 C CNN
F 2 "" H 1300 3500 50  0001 C CNN
F 3 "" H 1300 3500 50  0001 C CNN
	1    1300 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 3300 2950 3300
Text GLabel 3950 3250 0    39   Input ~ 0
ADC_IN4
Text GLabel 6100 3650 2    39   Input ~ 0
ADC_4
$Comp
L karta_stm-rescue:GND-power #PWR047
U 1 1 5FA66EB9
P 4700 3700
F 0 "#PWR047" H 4700 3450 50  0001 C CNN
F 1 "GND" H 4705 3527 50  0000 C CNN
F 2 "" H 4700 3700 50  0001 C CNN
F 3 "" H 4700 3700 50  0001 C CNN
	1    4700 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3700 4700 3650
Wire Wire Line
	4500 3450 4500 4000
Wire Wire Line
	4500 4000 4650 4000
Wire Wire Line
	5150 4000 5150 3350
Wire Wire Line
	5150 3350 5100 3350
$Comp
L karta_stm-rescue:R-Device R14
U 1 1 5FA66EC4
P 5700 3500
F 0 "R14" H 5770 3546 50  0000 L CNN
F 1 "5.1k" H 5770 3455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5630 3500 50  0001 C CNN
F 3 "~" H 5700 3500 50  0001 C CNN
	1    5700 3500
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R15
U 1 1 5FA66ECA
P 5700 3850
F 0 "R15" H 5770 3896 50  0000 L CNN
F 1 "10k" H 5770 3805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5630 3850 50  0001 C CNN
F 3 "~" H 5700 3850 50  0001 C CNN
	1    5700 3850
	1    0    0    -1  
$EndComp
Connection ~ 5150 3350
Wire Wire Line
	5700 3650 5700 3700
$Comp
L karta_stm-rescue:GND-power #PWR050
U 1 1 5FA66ED2
P 5700 4000
F 0 "#PWR050" H 5700 3750 50  0001 C CNN
F 1 "GND" H 5705 3827 50  0000 C CNN
F 2 "" H 5700 4000 50  0001 C CNN
F 3 "" H 5700 4000 50  0001 C CNN
	1    5700 4000
	1    0    0    -1  
$EndComp
Connection ~ 5700 3650
$Comp
L karta_stm-rescue:R-Device R11
U 1 1 5FA66EDF
P 4800 4000
F 0 "R11" V 4593 4000 50  0000 C CNN
F 1 "11k" V 4684 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4730 4000 50  0001 C CNN
F 3 "~" H 4800 4000 50  0001 C CNN
	1    4800 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 4000 5150 4000
Wire Wire Line
	5900 3950 5900 4100
$Comp
L karta_stm-rescue:GND-power #PWR051
U 1 1 5FA66EE7
P 5900 4100
F 0 "#PWR051" H 5900 3850 50  0001 C CNN
F 1 "GND" H 5905 3927 50  0000 C CNN
F 2 "" H 5900 4100 50  0001 C CNN
F 3 "" H 5900 4100 50  0001 C CNN
	1    5900 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 3650 5700 3650
$Comp
L karta_stm-rescue:C-Device C20
U 1 1 5FA66EEE
P 5900 3800
F 0 "C20" H 6015 3846 50  0000 L CNN
F 1 "0.1uF" H 6015 3755 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5938 3650 50  0001 C CNN
F 3 "~" H 5900 3800 50  0001 C CNN
	1    5900 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 3650 5900 3650
Connection ~ 5900 3650
$Comp
L karta_stm-rescue:GND-power #PWR044
U 1 1 5FA66F0F
P 4050 3550
F 0 "#PWR044" H 4050 3300 50  0001 C CNN
F 1 "GND" H 4055 3377 50  0000 C CNN
F 2 "" H 4050 3550 50  0001 C CNN
F 3 "" H 4050 3550 50  0001 C CNN
	1    4050 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3350 5700 3350
$Comp
L karta_stm-rescue:R-Device R20
U 1 1 5FA9EC41
P 8950 1700
F 0 "R20" H 9020 1746 50  0000 L CNN
F 1 "10k Ohm" H 9020 1655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8880 1700 50  0001 C CNN
F 3 "~" H 8950 1700 50  0001 C CNN
	1    8950 1700
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR061
U 1 1 5FA9EC47
P 8950 1850
F 0 "#PWR061" H 8950 1600 50  0001 C CNN
F 1 "GND" H 8955 1677 50  0000 C CNN
F 2 "" H 8950 1850 50  0001 C CNN
F 3 "" H 8950 1850 50  0001 C CNN
	1    8950 1850
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R4
U 1 1 5FAA229B
P 1600 3350
F 0 "R4" H 1670 3396 50  0000 L CNN
F 1 "10k Ohm" H 1670 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1530 3350 50  0001 C CNN
F 3 "~" H 1600 3350 50  0001 C CNN
	1    1600 3350
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR033
U 1 1 5FAA22A1
P 1600 3500
F 0 "#PWR033" H 1600 3250 50  0001 C CNN
F 1 "GND" H 1605 3327 50  0000 C CNN
F 2 "" H 1600 3500 50  0001 C CNN
F 3 "" H 1600 3500 50  0001 C CNN
	1    1600 3500
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R10
U 1 1 5FAA57BE
P 4300 3400
F 0 "R10" H 4370 3446 50  0000 L CNN
F 1 "10k Ohm" H 4370 3355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4230 3400 50  0001 C CNN
F 3 "~" H 4300 3400 50  0001 C CNN
	1    4300 3400
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR045
U 1 1 5FAA57C4
P 4300 3550
F 0 "#PWR045" H 4300 3300 50  0001 C CNN
F 1 "GND" H 4305 3377 50  0000 C CNN
F 2 "" H 4300 3550 50  0001 C CNN
F 3 "" H 4300 3550 50  0001 C CNN
	1    4300 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1550 6350 1550
$Comp
L karta_stm-rescue:BAT54S-Diode D8
U 1 1 5FC151EF
P 6150 1550
F 0 "D8" V 6196 1638 50  0000 L CNN
F 1 "BAT54S" V 6105 1638 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6225 1675 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11005.pdf" H 6030 1550 50  0001 C CNN
	1    6150 1550
	0    -1   -1   0   
$EndComp
Connection ~ 6350 1550
Wire Wire Line
	8600 1550 8900 1550
$Comp
L karta_stm-rescue:BAT54S-Diode D9
U 1 1 5FC190C8
P 8700 1550
F 0 "D9" V 8746 1638 50  0000 L CNN
F 1 "BAT54S" V 8655 1638 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8775 1675 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11005.pdf" H 8580 1550 50  0001 C CNN
	1    8700 1550
	0    -1   -1   0   
$EndComp
Connection ~ 8900 1550
Wire Wire Line
	1200 3200 1500 3200
$Comp
L karta_stm-rescue:BAT54S-Diode D5
U 1 1 5FC1CF0F
P 1300 3200
F 0 "D5" V 1346 3288 50  0000 L CNN
F 1 "BAT54S" V 1255 3288 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1375 3325 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11005.pdf" H 1180 3200 50  0001 C CNN
	1    1300 3200
	0    -1   -1   0   
$EndComp
Connection ~ 1500 3200
Wire Wire Line
	3950 3250 4250 3250
$Comp
L karta_stm-rescue:BAT54S-Diode D7
U 1 1 5FC211C2
P 4050 3250
F 0 "D7" V 4096 3338 50  0000 L CNN
F 1 "BAT54S" V 4005 3338 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4125 3375 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11005.pdf" H 3930 3250 50  0001 C CNN
	1    4050 3250
	0    -1   -1   0   
$EndComp
Connection ~ 4250 3250
Wire Wire Line
	3650 1800 3650 1500
Wire Wire Line
	3250 1500 3550 1500
Connection ~ 3550 1500
$Comp
L karta_stm-rescue:BAT54S-Diode D6
U 1 1 5FC0D917
P 3350 1500
F 0 "D6" V 3396 1588 50  0000 L CNN
F 1 "BAT54S" V 3305 1588 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3425 1625 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11005.pdf" H 3230 1500 50  0001 C CNN
	1    3350 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 1500 3650 1500
Connection ~ 3650 1500
Wire Wire Line
	3650 1500 3800 1500
Wire Wire Line
	6350 1550 6450 1550
Connection ~ 6450 1550
Wire Wire Line
	6450 1550 6600 1550
Wire Wire Line
	8900 1550 8950 1550
Connection ~ 8950 1550
Wire Wire Line
	8950 1550 9150 1550
Wire Wire Line
	4250 3250 4300 3250
Connection ~ 4300 3250
Wire Wire Line
	4300 3250 4500 3250
Wire Wire Line
	1500 3200 1600 3200
Connection ~ 1600 3200
Wire Wire Line
	1600 3200 1750 3200
$Comp
L karta_stm-rescue:MCP6001x-LT-Amplifier_Operational U6
U 1 1 5FD3A1D7
P 4100 1600
AR Path="/5FD3A1D7" Ref="U6"  Part="1" 
AR Path="/5F944892/5FD3A1D7" Ref="U6"  Part="1" 
F 0 "U6" H 4444 1646 50  0000 L CNN
F 1 "MCP6001x-LT" H 4444 1555 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 4000 1400 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 4100 1600 50  0001 C CNN
	1    4100 1600
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:MCP6001x-LT-Amplifier_Operational U8
U 1 1 5FD3E194
P 6900 1650
AR Path="/5FD3E194" Ref="U8"  Part="1" 
AR Path="/5F944892/5FD3E194" Ref="U8"  Part="1" 
F 0 "U8" H 7244 1696 50  0000 L CNN
F 1 "MCP6001x-LT" H 7244 1605 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 6800 1450 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 6900 1650 50  0001 C CNN
	1    6900 1650
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:MCP6001x-LT-Amplifier_Operational U9
U 1 1 5FD414BF
P 9450 1650
AR Path="/5FD414BF" Ref="U9"  Part="1" 
AR Path="/5F944892/5FD414BF" Ref="U9"  Part="1" 
F 0 "U9" H 9794 1696 50  0000 L CNN
F 1 "MCP6001x-LT" H 9794 1605 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 9350 1450 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 9450 1650 50  0001 C CNN
	1    9450 1650
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:MCP6001x-LT-Amplifier_Operational U5
U 1 1 5FD44A57
P 2050 3300
AR Path="/5FD44A57" Ref="U5"  Part="1" 
AR Path="/5F944892/5FD44A57" Ref="U5"  Part="1" 
F 0 "U5" H 2394 3346 50  0000 L CNN
F 1 "MCP6001x-LT" H 2394 3255 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 1950 3100 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 2050 3300 50  0001 C CNN
	1    2050 3300
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:MCP6001x-LT-Amplifier_Operational U7
U 1 1 5FD47BF5
P 4800 3350
AR Path="/5FD47BF5" Ref="U7"  Part="1" 
AR Path="/5F944892/5FD47BF5" Ref="U7"  Part="1" 
F 0 "U7" H 5144 3396 50  0000 L CNN
F 1 "MCP6001x-LT" H 5144 3305 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 4700 3150 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 4800 3350 50  0001 C CNN
	1    4800 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x05_Female J5
U 1 1 6096DE4A
P 1450 1650
F 0 "J5" H 1478 1676 50  0000 L CNN
F 1 "Conn_01x05_Female" H 1478 1585 50  0000 L CNN
F 2 "Connector_Molex:Molex_Micro-Fit_3.0_43650-0500_1x05_P3.00mm_Horizontal" H 1450 1650 50  0001 C CNN
F 3 "~" H 1450 1650 50  0001 C CNN
	1    1450 1650
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C0628D
P 1300 2900
F 0 "#PWR?" H 1300 2750 50  0001 C CNN
F 1 "+5V" H 1315 3073 50  0000 C CNN
F 2 "" H 1300 2900 50  0001 C CNN
F 3 "" H 1300 2900 50  0001 C CNN
	1    1300 2900
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C09523
P 1950 3000
F 0 "#PWR?" H 1950 2850 50  0001 C CNN
F 1 "+5V" H 1965 3173 50  0000 C CNN
F 2 "" H 1950 3000 50  0001 C CNN
F 3 "" H 1950 3000 50  0001 C CNN
	1    1950 3000
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C0C7DB
P 4050 2950
F 0 "#PWR?" H 4050 2800 50  0001 C CNN
F 1 "+5V" H 4065 3123 50  0000 C CNN
F 2 "" H 4050 2950 50  0001 C CNN
F 3 "" H 4050 2950 50  0001 C CNN
	1    4050 2950
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C0F916
P 4700 3050
F 0 "#PWR?" H 4700 2900 50  0001 C CNN
F 1 "+5V" H 4715 3223 50  0000 C CNN
F 2 "" H 4700 3050 50  0001 C CNN
F 3 "" H 4700 3050 50  0001 C CNN
	1    4700 3050
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C1293D
P 3350 1200
F 0 "#PWR?" H 3350 1050 50  0001 C CNN
F 1 "+5V" H 3365 1373 50  0000 C CNN
F 2 "" H 3350 1200 50  0001 C CNN
F 3 "" H 3350 1200 50  0001 C CNN
	1    3350 1200
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C15C30
P 4000 1300
F 0 "#PWR?" H 4000 1150 50  0001 C CNN
F 1 "+5V" H 4015 1473 50  0000 C CNN
F 2 "" H 4000 1300 50  0001 C CNN
F 3 "" H 4000 1300 50  0001 C CNN
	1    4000 1300
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C18CC9
P 6150 1250
F 0 "#PWR?" H 6150 1100 50  0001 C CNN
F 1 "+5V" H 6165 1423 50  0000 C CNN
F 2 "" H 6150 1250 50  0001 C CNN
F 3 "" H 6150 1250 50  0001 C CNN
	1    6150 1250
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C1BF48
P 6800 1350
F 0 "#PWR?" H 6800 1200 50  0001 C CNN
F 1 "+5V" H 6815 1523 50  0000 C CNN
F 2 "" H 6800 1350 50  0001 C CNN
F 3 "" H 6800 1350 50  0001 C CNN
	1    6800 1350
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C1F0CD
P 8700 1250
F 0 "#PWR?" H 8700 1100 50  0001 C CNN
F 1 "+5V" H 8715 1423 50  0000 C CNN
F 2 "" H 8700 1250 50  0001 C CNN
F 3 "" H 8700 1250 50  0001 C CNN
	1    8700 1250
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+5V-power #PWR?
U 1 1 60C22258
P 9350 1350
F 0 "#PWR?" H 9350 1200 50  0001 C CNN
F 1 "+5V" H 9365 1523 50  0000 C CNN
F 2 "" H 9350 1350 50  0001 C CNN
F 3 "" H 9350 1350 50  0001 C CNN
	1    9350 1350
	1    0    0    -1  
$EndComp
$EndSCHEMATC
