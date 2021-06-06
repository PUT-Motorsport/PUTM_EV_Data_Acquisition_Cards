EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 6
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
L karta_stm-rescue:EL817-Isolator U10
U 1 1 5F94BAC6
P 2500 1600
F 0 "U10" H 2500 1925 50  0000 C CNN
F 1 "EL817" H 2500 1834 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm_SMDSocket_SmallPads" H 2300 1400 50  0001 L CIN
F 3 "http://www.everlight.com/file/ProductFile/EL817.pdf" H 2500 1600 50  0001 L CNN
	1    2500 1600
	1    0    0    -1  
$EndComp
Text GLabel 1650 1500 0    50   Input ~ 0
IN_0
Wire Wire Line
	2050 1500 2200 1500
Wire Wire Line
	1650 1500 1750 1500
$Comp
L karta_stm-rescue:GND-power #PWR066
U 1 1 5F94C9A7
P 2200 1700
F 0 "#PWR066" H 2200 1450 50  0001 C CNN
F 1 "GND" H 2205 1527 50  0000 C CNN
F 2 "" H 2200 1700 50  0001 C CNN
F 3 "" H 2200 1700 50  0001 C CNN
	1    2200 1700
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR067
U 1 1 5F94CDCE
P 2800 2000
F 0 "#PWR067" H 2800 1750 50  0001 C CNN
F 1 "GND" H 2805 1827 50  0000 C CNN
F 2 "" H 2800 2000 50  0001 C CNN
F 3 "" H 2800 2000 50  0001 C CNN
	1    2800 2000
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+3.3V-power #PWR068
U 1 1 5F94D2C7
P 3100 1200
F 0 "#PWR068" H 3100 1050 50  0001 C CNN
F 1 "+3.3V" H 3115 1373 50  0000 C CNN
F 2 "" H 3100 1200 50  0001 C CNN
F 3 "" H 3100 1200 50  0001 C CNN
	1    3100 1200
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R25
U 1 1 5F94D7F1
P 2800 1850
F 0 "R25" H 2870 1896 50  0000 L CNN
F 1 "R" H 2870 1805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2730 1850 50  0001 C CNN
F 3 "~" H 2800 1850 50  0001 C CNN
	1    2800 1850
	1    0    0    -1  
$EndComp
Text GLabel 2950 1700 2    50   Input ~ 0
INPUT_0
$Comp
L karta_stm-rescue:EL817-Isolator U11
U 1 1 5F94F454
P 4950 1600
F 0 "U11" H 4950 1925 50  0000 C CNN
F 1 "EL817" H 4950 1834 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm_SMDSocket_SmallPads" H 4750 1400 50  0001 L CIN
F 3 "http://www.everlight.com/file/ProductFile/EL817.pdf" H 4950 1600 50  0001 L CNN
	1    4950 1600
	1    0    0    -1  
$EndComp
Text GLabel 4100 1500 0    50   Input ~ 0
IN_1
$Comp
L karta_stm-rescue:R-Device R26
U 1 1 5F94F45B
P 4350 1500
F 0 "R26" V 4557 1500 50  0000 C CNN
F 1 "10k" V 4466 1500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4280 1500 50  0001 C CNN
F 3 "~" H 4350 1500 50  0001 C CNN
	1    4350 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4500 1500 4650 1500
Wire Wire Line
	4100 1500 4200 1500
$Comp
L karta_stm-rescue:GND-power #PWR069
U 1 1 5F94F463
P 4650 1700
F 0 "#PWR069" H 4650 1450 50  0001 C CNN
F 1 "GND" H 4655 1527 50  0000 C CNN
F 2 "" H 4650 1700 50  0001 C CNN
F 3 "" H 4650 1700 50  0001 C CNN
	1    4650 1700
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR070
U 1 1 5F94F469
P 5350 2000
F 0 "#PWR070" H 5350 1750 50  0001 C CNN
F 1 "GND" H 5355 1827 50  0000 C CNN
F 2 "" H 5350 2000 50  0001 C CNN
F 3 "" H 5350 2000 50  0001 C CNN
	1    5350 2000
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+3.3V-power #PWR071
U 1 1 5F94F46F
P 5550 1200
F 0 "#PWR071" H 5550 1050 50  0001 C CNN
F 1 "+3.3V" H 5565 1373 50  0000 C CNN
F 2 "" H 5550 1200 50  0001 C CNN
F 3 "" H 5550 1200 50  0001 C CNN
	1    5550 1200
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R27
U 1 1 5F94F475
P 5350 1850
F 0 "R27" H 5420 1896 50  0000 L CNN
F 1 "R" H 5420 1805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5280 1850 50  0001 C CNN
F 3 "~" H 5350 1850 50  0001 C CNN
	1    5350 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 1700 5350 1700
Text GLabel 5750 1700 2    50   Input ~ 0
INPUT_1
$Comp
L karta_stm-rescue:EL817-Isolator U12
U 1 1 5F9512EE
P 7400 1600
F 0 "U12" H 7400 1925 50  0000 C CNN
F 1 "EL817" H 7400 1834 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm_SMDSocket_SmallPads" H 7200 1400 50  0001 L CIN
F 3 "http://www.everlight.com/file/ProductFile/EL817.pdf" H 7400 1600 50  0001 L CNN
	1    7400 1600
	1    0    0    -1  
$EndComp
Text GLabel 6550 1500 0    50   Input ~ 0
IN_2
$Comp
L karta_stm-rescue:R-Device R28
U 1 1 5F9512F5
P 6800 1500
F 0 "R28" V 7007 1500 50  0000 C CNN
F 1 "10k" V 6916 1500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6730 1500 50  0001 C CNN
F 3 "~" H 6800 1500 50  0001 C CNN
	1    6800 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6950 1500 7100 1500
Wire Wire Line
	6550 1500 6650 1500
$Comp
L karta_stm-rescue:GND-power #PWR072
U 1 1 5F9512FD
P 7100 1700
F 0 "#PWR072" H 7100 1450 50  0001 C CNN
F 1 "GND" H 7105 1527 50  0000 C CNN
F 2 "" H 7100 1700 50  0001 C CNN
F 3 "" H 7100 1700 50  0001 C CNN
	1    7100 1700
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR073
U 1 1 5F951303
P 7850 2000
F 0 "#PWR073" H 7850 1750 50  0001 C CNN
F 1 "GND" H 7855 1827 50  0000 C CNN
F 2 "" H 7850 2000 50  0001 C CNN
F 3 "" H 7850 2000 50  0001 C CNN
	1    7850 2000
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+3.3V-power #PWR074
U 1 1 5F951309
P 8000 1400
F 0 "#PWR074" H 8000 1250 50  0001 C CNN
F 1 "+3.3V" H 8015 1573 50  0000 C CNN
F 2 "" H 8000 1400 50  0001 C CNN
F 3 "" H 8000 1400 50  0001 C CNN
	1    8000 1400
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R29
U 1 1 5F95130F
P 7850 1850
F 0 "R29" H 7920 1896 50  0000 L CNN
F 1 "R" H 7920 1805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7780 1850 50  0001 C CNN
F 3 "~" H 7850 1850 50  0001 C CNN
	1    7850 1850
	1    0    0    -1  
$EndComp
Text GLabel 8200 1700 2    50   Input ~ 0
INPUT_2
$Comp
L karta_stm-rescue:EL817-Isolator U13
U 1 1 5F9528C7
P 9900 1600
F 0 "U13" H 9900 1925 50  0000 C CNN
F 1 "EL817" H 9900 1834 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm_SMDSocket_SmallPads" H 9700 1400 50  0001 L CIN
F 3 "http://www.everlight.com/file/ProductFile/EL817.pdf" H 9900 1600 50  0001 L CNN
	1    9900 1600
	1    0    0    -1  
$EndComp
Text GLabel 9050 1500 0    50   Input ~ 0
IN_3
$Comp
L karta_stm-rescue:R-Device R30
U 1 1 5F9528CE
P 9300 1500
F 0 "R30" V 9507 1500 50  0000 C CNN
F 1 "10k" V 9416 1500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9230 1500 50  0001 C CNN
F 3 "~" H 9300 1500 50  0001 C CNN
	1    9300 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9450 1500 9600 1500
Wire Wire Line
	9050 1500 9150 1500
$Comp
L karta_stm-rescue:GND-power #PWR075
U 1 1 5F9528D6
P 9600 1700
F 0 "#PWR075" H 9600 1450 50  0001 C CNN
F 1 "GND" H 9605 1527 50  0000 C CNN
F 2 "" H 9600 1700 50  0001 C CNN
F 3 "" H 9600 1700 50  0001 C CNN
	1    9600 1700
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:GND-power #PWR076
U 1 1 5F9528DC
P 10300 2000
F 0 "#PWR076" H 10300 1750 50  0001 C CNN
F 1 "GND" H 10305 1827 50  0000 C CNN
F 2 "" H 10300 2000 50  0001 C CNN
F 3 "" H 10300 2000 50  0001 C CNN
	1    10300 2000
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:+3.3V-power #PWR077
U 1 1 5F9528E2
P 10450 1350
F 0 "#PWR077" H 10450 1200 50  0001 C CNN
F 1 "+3.3V" H 10465 1523 50  0000 C CNN
F 2 "" H 10450 1350 50  0001 C CNN
F 3 "" H 10450 1350 50  0001 C CNN
	1    10450 1350
	1    0    0    -1  
$EndComp
$Comp
L karta_stm-rescue:R-Device R31
U 1 1 5F9528E8
P 10300 1850
F 0 "R31" H 10370 1896 50  0000 L CNN
F 1 "R" H 10370 1805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10230 1850 50  0001 C CNN
F 3 "~" H 10300 1850 50  0001 C CNN
	1    10300 1850
	1    0    0    -1  
$EndComp
Text GLabel 10700 1700 2    50   Input ~ 0
INPUT_3
Wire Wire Line
	2950 1700 2800 1700
$Comp
L karta_stm-rescue:R-Device R24
U 1 1 5F94C456
P 1900 1500
F 0 "R24" V 2107 1500 50  0000 C CNN
F 1 "10k" V 2016 1500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1830 1500 50  0001 C CNN
F 3 "~" H 1900 1500 50  0001 C CNN
	1    1900 1500
	0    -1   -1   0   
$EndComp
Connection ~ 2800 1700
Wire Wire Line
	2800 1500 3100 1500
Wire Wire Line
	3100 1200 3100 1500
Text GLabel 3650 3800 0    50   Input ~ 0
IN_0
Text GLabel 3650 3900 0    50   Input ~ 0
IN_1
Text GLabel 3650 4000 0    50   Input ~ 0
IN_2
Text GLabel 3650 4100 0    50   Input ~ 0
IN_3
Connection ~ 5350 1700
Wire Wire Line
	5350 1700 5750 1700
Wire Wire Line
	5550 1200 5550 1500
Wire Wire Line
	5550 1500 5250 1500
Wire Wire Line
	7700 1700 7850 1700
Connection ~ 7850 1700
Wire Wire Line
	7850 1700 8200 1700
Wire Wire Line
	8000 1400 8000 1500
Wire Wire Line
	8000 1500 7700 1500
Wire Wire Line
	10200 1700 10300 1700
Connection ~ 10300 1700
Wire Wire Line
	10300 1700 10700 1700
Wire Wire Line
	10450 1350 10450 1500
Wire Wire Line
	10450 1500 10200 1500
$Comp
L Connector:Conn_01x04_Female J6
U 1 1 60969389
P 3850 3900
F 0 "J6" H 3878 3876 50  0000 L CNN
F 1 "Conn_01x04_Female" H 3878 3785 50  0000 L CNN
F 2 "molex:436500412" H 3850 3900 50  0001 C CNN
F 3 "~" H 3850 3900 50  0001 C CNN
	1    3850 3900
	1    0    0    -1  
$EndComp
$EndSCHEMATC