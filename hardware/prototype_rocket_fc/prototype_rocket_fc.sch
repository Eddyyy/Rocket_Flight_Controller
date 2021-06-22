EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
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
L teensy:Teensy4.0 U2
U 1 1 609E097F
P 5600 3750
F 0 "U2" H 5600 5365 50  0000 C CNN
F 1 "Teensy4.0" H 5600 5274 50  0000 C CNN
F 2 "teensy:Teensy40" H 5200 3950 50  0001 C CNN
F 3 "" H 5200 3950 50  0001 C CNN
	1    5600 3750
	1    0    0    -1  
$EndComp
$Sheet
S 1900 900  1200 1550
U 609E7FAE
F0 "battery_power_supply" 50
F1 "battery_power_supply.sch" 50
F2 "VIN" I L 1900 1250 50 
F3 "GND" I L 1900 1950 50 
F4 "5V" I R 3100 1250 50 
F5 "3.3V" I R 3100 1950 50 
$EndSheet
$Comp
L power:VCC #PWR01
U 1 1 60A13FBE
P 1750 1250
F 0 "#PWR01" H 1750 1100 50  0001 C CNN
F 1 "VCC" H 1765 1423 50  0000 C CNN
F 2 "" H 1750 1250 50  0001 C CNN
F 3 "" H 1750 1250 50  0001 C CNN
	1    1750 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 60A147F1
P 1750 1950
F 0 "#PWR02" H 1750 1700 50  0001 C CNN
F 1 "GND" H 1755 1777 50  0000 C CNN
F 2 "" H 1750 1950 50  0001 C CNN
F 3 "" H 1750 1950 50  0001 C CNN
	1    1750 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR05
U 1 1 60A14BD7
P 3250 1250
F 0 "#PWR05" H 3250 1100 50  0001 C CNN
F 1 "+5V" H 3265 1423 50  0000 C CNN
F 2 "" H 3250 1250 50  0001 C CNN
F 3 "" H 3250 1250 50  0001 C CNN
	1    3250 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 1250 1750 1250
Wire Wire Line
	1900 1950 1750 1950
Wire Wire Line
	3100 1250 3250 1250
$Comp
L power:+3.3V #PWR06
U 1 1 60A2E4FF
P 3250 1950
F 0 "#PWR06" H 3250 1800 50  0001 C CNN
F 1 "+3.3V" H 3265 2123 50  0000 C CNN
F 2 "" H 3250 1950 50  0001 C CNN
F 3 "" H 3250 1950 50  0001 C CNN
	1    3250 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1950 3250 1950
$Comp
L power:+5V #PWR07
U 1 1 60A3012D
P 6900 4700
F 0 "#PWR07" H 6900 4550 50  0001 C CNN
F 1 "+5V" H 6915 4873 50  0000 C CNN
F 2 "" H 6900 4700 50  0001 C CNN
F 3 "" H 6900 4700 50  0001 C CNN
	1    6900 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 4700 6900 4700
$Comp
L power:GND #PWR08
U 1 1 60A30AD9
P 6900 4800
F 0 "#PWR08" H 6900 4550 50  0001 C CNN
F 1 "GND" H 6905 4627 50  0000 C CNN
F 2 "" H 6900 4800 50  0001 C CNN
F 3 "" H 6900 4800 50  0001 C CNN
	1    6900 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 4800 6900 4800
$Comp
L GY-BMP280_custom:GY-BMP280 U3
U 1 1 60A23310
P 8200 1400
F 0 "U3" H 8400 1850 50  0000 C CNN
F 1 "GY-BMP280" H 8550 900 50  0000 C CNN
F 2 "myFootprints:GY-BMP280" H 8805 830 50  0001 C CNN
F 3 "" H 8850 1250 50  0001 C CNN
	1    8200 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 60A261DE
P 8200 950
F 0 "#PWR09" H 8200 800 50  0001 C CNN
F 1 "+3.3V" H 8215 1123 50  0000 C CNN
F 2 "" H 8200 950 50  0001 C CNN
F 3 "" H 8200 950 50  0001 C CNN
	1    8200 950 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 60A26749
P 8200 1900
F 0 "#PWR010" H 8200 1650 50  0001 C CNN
F 1 "GND" H 8205 1727 50  0000 C CNN
F 2 "" H 8200 1900 50  0001 C CNN
F 3 "" H 8200 1900 50  0001 C CNN
	1    8200 1900
	1    0    0    -1  
$EndComp
Text Label 8650 1350 0    50   ~ 0
SCL0
Text Label 8650 1500 0    50   ~ 0
SDA0
Wire Wire Line
	8550 1350 8650 1350
Wire Wire Line
	8550 1500 8650 1500
$Comp
L MPU9250_Invensense_custom:MPU9250_custom U5
U 1 1 60A32DF1
P 10000 3400
F 0 "U5" H 10500 4200 50  0000 C CNN
F 1 "MPU9250_custom" H 10600 2550 50  0000 C CNN
F 2 "myFootprints:MPU9250" H 10600 2400 50  0001 C CNN
F 3 "" H 10000 3400 50  0001 C CNN
	1    10000 3400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 60A35DA1
P 10100 2600
F 0 "#PWR014" H 10100 2450 50  0001 C CNN
F 1 "+3.3V" H 10115 2773 50  0000 C CNN
F 2 "" H 10100 2600 50  0001 C CNN
F 3 "" H 10100 2600 50  0001 C CNN
	1    10100 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 60A3651E
P 10000 4300
F 0 "#PWR013" H 10000 4050 50  0001 C CNN
F 1 "GND" H 10005 4127 50  0000 C CNN
F 2 "" H 10000 4300 50  0001 C CNN
F 3 "" H 10000 4300 50  0001 C CNN
	1    10000 4300
	1    0    0    -1  
$EndComp
Text Label 9100 3600 2    50   ~ 0
SDA0
Text Label 9100 3400 2    50   ~ 0
SCL0
Wire Wire Line
	9200 3400 9100 3400
Wire Wire Line
	9200 3600 9100 3600
$Comp
L TinyGPS++_custom:TinyGPS++ U4
U 1 1 60A4F9A3
P 8200 5250
F 0 "U4" H 8450 5700 50  0000 C CNN
F 1 "TinyGPS++" H 8500 4800 50  0000 C CNN
F 2 "myFootprints:TinyGPS++" H 8800 4700 50  0001 C CNN
F 3 "" H 8800 4700 50  0001 C CNN
	1    8200 5250
	1    0    0    -1  
$EndComp
Text Label 4350 4800 2    50   ~ 0
SDA0
Text Label 4350 4900 2    50   ~ 0
SCL0
Wire Wire Line
	4500 4800 4350 4800
Wire Wire Line
	4500 4900 4350 4900
$Comp
L power:+5V #PWR011
U 1 1 60A5670D
P 8200 4800
F 0 "#PWR011" H 8200 4650 50  0001 C CNN
F 1 "+5V" H 8215 4973 50  0000 C CNN
F 2 "" H 8200 4800 50  0001 C CNN
F 3 "" H 8200 4800 50  0001 C CNN
	1    8200 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 60A56B29
P 8200 5700
F 0 "#PWR012" H 8200 5450 50  0001 C CNN
F 1 "GND" H 8205 5527 50  0000 C CNN
F 2 "" H 8200 5700 50  0001 C CNN
F 3 "" H 8200 5700 50  0001 C CNN
	1    8200 5700
	1    0    0    -1  
$EndComp
Text Label 4350 2500 2    50   ~ 0
RX1
Text Label 4350 2600 2    50   ~ 0
TX1
Wire Wire Line
	4500 2500 4350 2500
Wire Wire Line
	4500 2600 4350 2600
Text Label 8800 5200 0    50   ~ 0
RX2
Text Label 8800 5300 0    50   ~ 0
TX2
Wire Wire Line
	8800 5200 8700 5200
Wire Wire Line
	8800 5300 8700 5300
$Comp
L RobotDyn_SD_Card_Reader:SD_Card_Reader U1
U 1 1 60A335D3
P 1850 4550
F 0 "U1" H 2344 4596 50  0000 L CNN
F 1 "SD_Card_Reader" H 2344 4505 50  0000 L CNN
F 2 "myFootprints:RobotDyn_SD_Card_Reader" H 2700 4000 50  0001 C CNN
F 3 "" H 1850 4550 50  0001 C CNN
	1    1850 4550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR04
U 1 1 60A37841
P 1900 4050
F 0 "#PWR04" H 1900 3900 50  0001 C CNN
F 1 "+5V" H 1915 4223 50  0000 C CNN
F 2 "" H 1900 4050 50  0001 C CNN
F 3 "" H 1900 4050 50  0001 C CNN
	1    1900 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 60A384C5
P 1850 5050
F 0 "#PWR03" H 1850 4800 50  0001 C CNN
F 1 "GND" H 1855 4877 50  0000 C CNN
F 2 "" H 1850 5050 50  0001 C CNN
F 3 "" H 1850 5050 50  0001 C CNN
	1    1850 5050
	1    0    0    -1  
$EndComp
Text Label 1300 4400 2    50   ~ 0
SCK
Text Label 4350 4300 2    50   ~ 0
SCK
Text Label 4400 3600 2    50   ~ 0
MOSI
Text Label 4400 3700 2    50   ~ 0
MISO
Wire Wire Line
	4400 3600 4500 3600
Wire Wire Line
	4500 3700 4400 3700
Wire Wire Line
	4500 4300 4350 4300
Text Label 1300 4500 2    50   ~ 0
MISO
Text Label 1300 4600 2    50   ~ 0
MOSI
Wire Wire Line
	1400 4400 1300 4400
Wire Wire Line
	1400 4500 1300 4500
Wire Wire Line
	1400 4600 1300 4600
Text Label 4400 3500 2    50   ~ 0
CS
Wire Wire Line
	4500 3500 4400 3500
Text Label 1300 4700 2    50   ~ 0
CS
Wire Wire Line
	1400 4700 1300 4700
Text Label 4350 2700 2    50   ~ 0
CD
Text Label 2400 4400 0    50   ~ 0
CD
Wire Wire Line
	2300 4400 2400 4400
Wire Wire Line
	4500 2700 4350 2700
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 60A6CC31
P 1100 1550
F 0 "J1" H 1180 1542 50  0000 L CNN
F 1 "BATTERY_IN" H 1180 1451 50  0000 L CNN
F 2 "Connector_AMASS:AMASS_XT60-M_1x02_P7.20mm_Vertical" H 1100 1550 50  0001 C CNN
F 3 "~" H 1100 1550 50  0001 C CNN
	1    1100 1550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR025
U 1 1 60A7A148
P 900 1550
F 0 "#PWR025" H 900 1400 50  0001 C CNN
F 1 "VCC" H 915 1723 50  0000 C CNN
F 2 "" H 900 1550 50  0001 C CNN
F 3 "" H 900 1550 50  0001 C CNN
	1    900  1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 60A7A5F4
P 900 1650
F 0 "#PWR026" H 900 1400 50  0001 C CNN
F 1 "GND" H 905 1477 50  0000 C CNN
F 2 "" H 900 1650 50  0001 C CNN
F 3 "" H 900 1650 50  0001 C CNN
	1    900  1650
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 60AA3367
P 3750 1500
F 0 "D1" H 3743 1245 50  0000 C CNN
F 1 "LED" H 3743 1336 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3750 1500 50  0001 C CNN
F 3 "~" H 3750 1500 50  0001 C CNN
	1    3750 1500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 60AA8046
P 3750 1250
F 0 "R5" H 3809 1296 50  0000 L CNN
F 1 "200" H 3809 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3750 1250 50  0001 C CNN
F 3 "~" H 3750 1250 50  0001 C CNN
	1    3750 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR027
U 1 1 60AA87AF
P 3750 1150
F 0 "#PWR027" H 3750 1000 50  0001 C CNN
F 1 "+5V" H 3765 1323 50  0000 C CNN
F 2 "" H 3750 1150 50  0001 C CNN
F 3 "" H 3750 1150 50  0001 C CNN
	1    3750 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 60AA90D6
P 3750 1650
F 0 "#PWR028" H 3750 1400 50  0001 C CNN
F 1 "GND" H 3755 1477 50  0000 C CNN
F 2 "" H 3750 1650 50  0001 C CNN
F 3 "" H 3750 1650 50  0001 C CNN
	1    3750 1650
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 60AB5496
P 5350 6450
F 0 "H2" H 5450 6499 50  0000 L CNN
F 1 "MountingHole_Pad" H 5450 6408 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 5350 6450 50  0001 C CNN
F 3 "~" H 5350 6450 50  0001 C CNN
	1    5350 6450
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 60AB5D37
P 5350 5950
F 0 "H1" H 5450 5999 50  0000 L CNN
F 1 "MountingHole_Pad" H 5450 5908 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 5350 5950 50  0001 C CNN
F 3 "~" H 5350 5950 50  0001 C CNN
	1    5350 5950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 60AB680F
P 5350 6550
F 0 "#PWR030" H 5350 6300 50  0001 C CNN
F 1 "GND" H 5355 6377 50  0000 C CNN
F 2 "" H 5350 6550 50  0001 C CNN
F 3 "" H 5350 6550 50  0001 C CNN
	1    5350 6550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 60AB6BBB
P 5350 6050
F 0 "#PWR029" H 5350 5800 50  0001 C CNN
F 1 "GND" H 5355 5877 50  0000 C CNN
F 2 "" H 5350 6050 50  0001 C CNN
F 3 "" H 5350 6050 50  0001 C CNN
	1    5350 6050
	1    0    0    -1  
$EndComp
Text Label 4350 3200 2    50   ~ 0
RX2
Text Label 4350 3300 2    50   ~ 0
TX2
Wire Wire Line
	4500 3200 4350 3200
Wire Wire Line
	4500 3300 4350 3300
$Comp
L RFD900x:RFD900x_Pins U9
U 1 1 60AE6DD5
P 2400 6500
F 0 "U9" H 3178 6621 50  0000 L CNN
F 1 "RFD900x_Pins" H 3178 6530 50  0000 L CNN
F 2 "myFootprints:RFD900x_Pins" H 3600 6150 50  0001 C CNN
F 3 "" H 2600 6800 50  0001 C CNN
	1    2400 6500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 60AE8180
P 2800 6800
F 0 "#PWR0101" H 2800 6650 50  0001 C CNN
F 1 "+5V" H 2815 6973 50  0000 C CNN
F 2 "" H 2800 6800 50  0001 C CNN
F 3 "" H 2800 6800 50  0001 C CNN
	1    2800 6800
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 60AE86B2
P 2100 6800
F 0 "#PWR0102" H 2100 6550 50  0001 C CNN
F 1 "GND" H 2105 6627 50  0000 C CNN
F 2 "" H 2100 6800 50  0001 C CNN
F 3 "" H 2100 6800 50  0001 C CNN
	1    2100 6800
	1    0    0    -1  
$EndComp
Text Label 2300 5950 1    50   ~ 0
TX1
Text Label 2400 5950 1    50   ~ 0
RX1
Wire Wire Line
	2300 5950 2300 6050
Wire Wire Line
	2400 5950 2400 6050
$Comp
L RFD900x:RFD900x_Mech U8
U 1 1 60AEBDEB
P 1200 6500
F 0 "U8" H 1328 6513 50  0000 L CNN
F 1 "RFD900x_Mech" H 1328 6422 50  0000 L CNN
F 2 "myFootprints:RFD900x_Mech" H 1800 6250 50  0001 C CNN
F 3 "" H 1300 6400 50  0001 C CNN
	1    1200 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 60AEC788
P 1200 6700
F 0 "#PWR0103" H 1200 6450 50  0001 C CNN
F 1 "GND" H 1205 6527 50  0000 C CNN
F 2 "" H 1200 6700 50  0001 C CNN
F 3 "" H 1200 6700 50  0001 C CNN
	1    1200 6700
	1    0    0    -1  
$EndComp
$EndSCHEMATC