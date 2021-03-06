EESchema Schematic File Version 4
EELAYER 30 0
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
L Device:LED D1
U 1 1 60208E23
P 5500 4400
F 0 "D1" H 5493 4617 50  0000 C CNN
F 1 "LED" H 5493 4526 50  0000 C CNN
F 2 "LED:LED" H 5500 4400 50  0001 C CNN
F 3 "~" H 5500 4400 50  0001 C CNN
	1    5500 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 602094A5
P 6250 4450
F 0 "D2" H 6243 4667 50  0000 C CNN
F 1 "LED" H 6243 4576 50  0000 C CNN
F 2 "LED:LED" H 6250 4450 50  0001 C CNN
F 3 "~" H 6250 4450 50  0001 C CNN
	1    6250 4450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J1
U 1 1 60209395
P 5550 4850
F 0 "J1" V 5396 4898 50  0000 L CNN
F 1 "LEDazul" V 5487 4898 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Horizontal" H 5550 4850 50  0001 C CNN
F 3 "~" H 5550 4850 50  0001 C CNN
	1    5550 4850
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x02_Female J2
U 1 1 60209B44
P 6300 4850
F 0 "J2" V 6146 4898 50  0000 L CNN
F 1 "LEDambar" V 6237 4898 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Horizontal" H 6300 4850 50  0001 C CNN
F 3 "~" H 6300 4850 50  0001 C CNN
	1    6300 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 4400 5350 4650
Wire Wire Line
	5350 4650 5450 4650
Wire Wire Line
	5650 4400 5650 4650
Wire Wire Line
	5650 4650 5550 4650
Wire Wire Line
	6100 4450 6100 4650
Wire Wire Line
	6100 4650 6200 4650
Wire Wire Line
	6400 4450 6400 4650
Wire Wire Line
	6400 4650 6300 4650
$Comp
L OPT3002DNPT:OPT3002DNPT IC2
U 1 1 602119A4
P 4300 2800
F 0 "IC2" H 4900 3065 50  0000 C CNN
F 1 "OPT3002DNPT" H 4900 2974 50  0000 C CNN
F 2 "OPT3002DNPT:SON65P200X200X65-7N-D" H 5350 2900 50  0001 L CNN
F 3 "http://www.ti.com/lit/gpn/OPT3002" H 5350 2800 50  0001 L CNN
F 4 "Light-to-digital sensor" H 5350 2700 50  0001 L CNN "Description"
F 5 "0.65" H 5350 2600 50  0001 L CNN "Height"
F 6 "595-OPT3002DNPT" H 5350 2500 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/OPT3002DNPT/?qs=zEmsApcVOkU5JfY94IcyUw%3D%3D" H 5350 2400 50  0001 L CNN "Mouser Price/Stock"
F 8 "Texas Instruments" H 5350 2300 50  0001 L CNN "Manufacturer_Name"
F 9 "OPT3002DNPT" H 5350 2200 50  0001 L CNN "Manufacturer_Part_Number"
	1    4300 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0105
U 1 1 60211ADA
P 4900 3500
F 0 "#PWR0105" H 4900 3250 50  0001 C CNN
F 1 "GNDREF" H 4905 3327 50  0000 C CNN
F 2 "" H 4900 3500 50  0001 C CNN
F 3 "" H 4900 3500 50  0001 C CNN
	1    4900 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0106
U 1 1 60211AE4
P 4300 3000
F 0 "#PWR0106" H 4300 2750 50  0001 C CNN
F 1 "GNDREF" V 4305 2872 50  0000 R CNN
F 2 "" H 4300 3000 50  0001 C CNN
F 3 "" H 4300 3000 50  0001 C CNN
	1    4300 3000
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR0107
U 1 1 60211AEE
P 4300 2900
F 0 "#PWR0107" H 4300 2650 50  0001 C CNN
F 1 "GNDREF" V 4305 2772 50  0000 R CNN
F 2 "" H 4300 2900 50  0001 C CNN
F 3 "" H 4300 2900 50  0001 C CNN
	1    4300 2900
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R4
U 1 1 60211AF8
P 5600 3000
F 0 "R4" V 5395 3000 50  0000 C CNN
F 1 "10k" V 5486 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 5600 3000 50  0001 C CNN
F 3 "~" H 5600 3000 50  0001 C CNN
	1    5600 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	5500 2900 5600 2900
$Comp
L Device:R_Small_US R6
U 1 1 60211B03
P 5850 2800
F 0 "R6" V 5645 2800 50  0000 C CNN
F 1 "10k" V 5736 2800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 5850 2800 50  0001 C CNN
F 3 "~" H 5850 2800 50  0001 C CNN
	1    5850 2800
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R5
U 1 1 60211B0D
P 5700 2900
F 0 "R5" V 5495 2900 50  0000 C CNN
F 1 "10k" V 5586 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 5700 2900 50  0001 C CNN
F 3 "~" H 5700 2900 50  0001 C CNN
	1    5700 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 2800 5500 2800
Wire Wire Line
	5950 2800 5950 2600
Wire Wire Line
	5950 2600 5800 2600
Wire Wire Line
	5800 2900 5800 2600
Connection ~ 5800 2600
Wire Wire Line
	5800 2600 5700 2600
Wire Wire Line
	5700 3000 5700 2600
Connection ~ 5700 2600
Wire Wire Line
	5700 2600 4300 2600
$Comp
L Device:C_Small C2
U 1 1 60211B20
P 4200 2650
F 0 "C2" V 3971 2650 50  0000 C CNN
F 1 "C_Small" V 4062 2650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4200 2650 50  0001 C CNN
F 3 "~" H 4200 2650 50  0001 C CNN
	1    4200 2650
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR0108
U 1 1 60211B2A
P 4100 2650
F 0 "#PWR0108" H 4100 2400 50  0001 C CNN
F 1 "GNDREF" V 4105 2522 50  0000 R CNN
F 2 "" H 4100 2650 50  0001 C CNN
F 3 "" H 4100 2650 50  0001 C CNN
	1    4100 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 2600 4300 2650
Connection ~ 4300 2650
Wire Wire Line
	4300 2650 4300 2800
$Comp
L Connector:Conn_01x01_Female J4
U 1 1 60211B37
P 4100 2800
F 0 "J4" H 3992 2575 50  0000 C CNN
F 1 "Vin" H 3992 2666 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Horizontal" H 4100 2800 50  0001 C CNN
F 3 "~" H 4100 2800 50  0001 C CNN
	1    4100 2800
	-1   0    0    1   
$EndComp
Connection ~ 4300 2800
$Comp
L OPT3002DNPT:OPT3002DNPT IC1
U 1 1 60449288
P 6500 2800
F 0 "IC1" H 7100 3065 50  0000 C CNN
F 1 "OPT3002DNPT" H 7100 2974 50  0000 C CNN
F 2 "OPT3002DNPT:SON65P200X200X65-7N-D" H 7550 2900 50  0001 L CNN
F 3 "http://www.ti.com/lit/gpn/OPT3002" H 7550 2800 50  0001 L CNN
F 4 "Light-to-digital sensor" H 7550 2700 50  0001 L CNN "Description"
F 5 "0.65" H 7550 2600 50  0001 L CNN "Height"
F 6 "595-OPT3002DNPT" H 7550 2500 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/OPT3002DNPT/?qs=zEmsApcVOkU5JfY94IcyUw%3D%3D" H 7550 2400 50  0001 L CNN "Mouser Price/Stock"
F 8 "Texas Instruments" H 7550 2300 50  0001 L CNN "Manufacturer_Name"
F 9 "OPT3002DNPT" H 7550 2200 50  0001 L CNN "Manufacturer_Part_Number"
	1    6500 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR04
U 1 1 604493BE
P 7100 3500
F 0 "#PWR04" H 7100 3250 50  0001 C CNN
F 1 "GNDREF" H 7105 3327 50  0000 C CNN
F 2 "" H 7100 3500 50  0001 C CNN
F 3 "" H 7100 3500 50  0001 C CNN
	1    7100 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR03
U 1 1 604493C8
P 6500 3000
F 0 "#PWR03" H 6500 2750 50  0001 C CNN
F 1 "GNDREF" V 6505 2872 50  0000 R CNN
F 2 "" H 6500 3000 50  0001 C CNN
F 3 "" H 6500 3000 50  0001 C CNN
	1    6500 3000
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR02
U 1 1 604493D2
P 6500 2900
F 0 "#PWR02" H 6500 2650 50  0001 C CNN
F 1 "GNDREF" V 6505 2772 50  0000 R CNN
F 2 "" H 6500 2900 50  0001 C CNN
F 3 "" H 6500 2900 50  0001 C CNN
	1    6500 2900
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 604493DC
P 7800 3000
F 0 "R1" V 7595 3000 50  0000 C CNN
F 1 "10k" V 7686 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 7800 3000 50  0001 C CNN
F 3 "~" H 7800 3000 50  0001 C CNN
	1    7800 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 2900 7800 2900
$Comp
L Device:R_Small_US R3
U 1 1 604493E7
P 8050 2800
F 0 "R3" V 7845 2800 50  0000 C CNN
F 1 "10k" V 7936 2800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8050 2800 50  0001 C CNN
F 3 "~" H 8050 2800 50  0001 C CNN
	1    8050 2800
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R2
U 1 1 604493F1
P 7900 2900
F 0 "R2" V 7695 2900 50  0000 C CNN
F 1 "10k" V 7786 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 7900 2900 50  0001 C CNN
F 3 "~" H 7900 2900 50  0001 C CNN
	1    7900 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	7950 2800 7700 2800
Wire Wire Line
	8150 2600 8000 2600
Wire Wire Line
	8000 2900 8000 2600
Connection ~ 8000 2600
Wire Wire Line
	8000 2600 7900 2600
Wire Wire Line
	7900 3000 7900 2600
Connection ~ 7900 2600
Wire Wire Line
	7900 2600 6500 2600
$Comp
L Device:C_Small C1
U 1 1 60449403
P 6400 2650
F 0 "C1" V 6171 2650 50  0000 C CNN
F 1 "C_Small" V 6262 2650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6400 2650 50  0001 C CNN
F 3 "~" H 6400 2650 50  0001 C CNN
	1    6400 2650
	0    1    1    0   
$EndComp
$Comp
L power:GNDREF #PWR01
U 1 1 6044940D
P 6300 2650
F 0 "#PWR01" H 6300 2400 50  0001 C CNN
F 1 "GNDREF" V 6305 2522 50  0000 R CNN
F 2 "" H 6300 2650 50  0001 C CNN
F 3 "" H 6300 2650 50  0001 C CNN
	1    6300 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	6500 2600 6500 2650
Connection ~ 6500 2650
Wire Wire Line
	6500 2650 6500 2800
$Comp
L Connector:Conn_01x01_Female J3
U 1 1 6044941A
P 6300 2800
F 0 "J3" H 6192 2575 50  0000 C CNN
F 1 "Vin" H 6192 2666 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Horizontal" H 6300 2800 50  0001 C CNN
F 3 "~" H 6300 2800 50  0001 C CNN
	1    6300 2800
	-1   0    0    1   
$EndComp
Connection ~ 6500 2800
$EndSCHEMATC
