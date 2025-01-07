# Management-and-control-of-a-greenhouse-with-ESP32

## Description

The system will be able to monitor the conditions in a greenhouse in real time, warning of different dangerous situations and sending the data via Bluetooth to the users. Analogue-to-digital converters and i2c channels will be used for communication between elements. Hardware interruptions, buffer systems, semaphores and mutexes will be used for synchronous data collection and sending. In addition, by using a button on the board, a low-energy mode can be switched on, which stops data collection for a certain period of time, to avoid unnecessary energy consumption. 



## Resources
To do this, the BH1750 (luminosity), SEN0193 (humidity) and SCD30 (CO2 and temperature) sensors are connected to an ESP32-S3 board. We will also need 3 LEDS to indicate different situations that may occur such as temperature drop or humidity below certain limits. 

## Setup

![montaje](https://github.com/user-attachments/assets/2dbeb967-1e8f-4349-b930-dd7b25cde7a0)

## Code

The main.c code contains all the described functionalities. 

## For a whole description of the project: 

[Project I.pdf](https://github.com/user-attachments/files/18330691/Project.I.pdf)

[Project II.pdf](https://github.com/user-attachments/files/18330692/Project.II.pdf)




