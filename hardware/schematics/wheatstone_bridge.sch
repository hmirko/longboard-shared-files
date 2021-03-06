EESchema Schematic File Version 4
LIBS:wheatstone_bridge-cache
EELAYER 30 0
EELAYER END
$Descr User 4016 4016
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
L Device:R R?
U 1 1 5DB178C7
P 2900 1450
F 0 "R?" H 2970 1496 50  0001 L CNN
F 1 "R2" H 2970 1405 50  0000 L CNN
F 2 "" V 2830 1450 50  0001 C CNN
F 3 "~" H 2900 1450 50  0001 C CNN
	1    2900 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5DB1C383
P 2250 1875
F 0 "R?" H 2320 1921 50  0001 L CNN
F 1 "R3" H 2320 1830 50  0000 L CNN
F 2 "" V 2180 1875 50  0001 C CNN
F 3 "~" H 2250 1875 50  0001 C CNN
	1    2250 1875
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 5DB1C59E
P 2900 1875
F 0 "R?" H 2970 1921 50  0001 L CNN
F 1 "R4" H 2970 1830 50  0000 L CNN
F 2 "" V 2830 1875 50  0001 C CNN
F 3 "~" H 2900 1875 50  0001 C CNN
	1    2900 1875
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5DB1C8DB
P 2250 1450
F 0 "R?" H 2320 1496 50  0001 L CNN
F 1 "R1" H 2320 1405 50  0000 L CNN
F 2 "" V 2180 1450 50  0001 C CNN
F 3 "~" H 2250 1450 50  0001 C CNN
	1    2250 1450
	-1   0    0    1   
$EndComp
$Comp
L Device:Voltmeter_DC MES?
U 1 1 5DB1CBD6
P 2575 1675
F 0 "MES?" V 2285 1675 50  0001 C CNN
F 1 "Voltmeter" V 2376 1675 50  0000 C CNN
F 2 "" V 2575 1775 50  0001 C CNN
F 3 "~" V 2575 1775 50  0001 C CNN
	1    2575 1675
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5DB212F9
P 2100 1300
F 0 "#PWR?" H 2100 1150 50  0001 C CNN
F 1 "VCC" V 2117 1473 50  0000 C CNN
F 2 "" H 2100 1300 50  0001 C CNN
F 3 "" H 2100 1300 50  0001 C CNN
	1    2100 1300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DB2B8D9
P 2100 2025
F 0 "#PWR?" H 2100 1775 50  0001 C CNN
F 1 "GND" V 2105 1897 50  0000 R CNN
F 2 "" H 2100 2025 50  0001 C CNN
F 3 "" H 2100 2025 50  0001 C CNN
	1    2100 2025
	0    1    1    0   
$EndComp
Wire Wire Line
	2100 2025 2250 2025
Wire Wire Line
	2250 1725 2250 1675
Wire Wire Line
	2900 1725 2900 1675
Connection ~ 2250 2025
Wire Wire Line
	2250 2025 2900 2025
Wire Wire Line
	2250 1675 2375 1675
Connection ~ 2250 1675
Wire Wire Line
	2250 1675 2250 1600
Wire Wire Line
	2775 1675 2900 1675
Connection ~ 2900 1675
Wire Wire Line
	2900 1675 2900 1600
Wire Wire Line
	2100 1300 2250 1300
Connection ~ 2250 1300
Wire Wire Line
	2250 1300 2900 1300
$EndSCHEMATC
