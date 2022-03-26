Electronic Project

Bicycle Light with Acceleration perception


By: Stefan Kaufmann	

Supervisor: Dipl.-Ing. Dr. Martin Horauer

Vienna, March 01.2022


 
Table of Contents

Introduction	3
Motivation	3
Video of functionality	3
Methodology	3
STM32	3
Bluetooth Low Energy	3
Acceleration Sensor	4
LED Control (WS2811)	4
Code Functionality	5
STM32 Pinout	5
Clock Configuration	6
TIMER1 Configuration	6
Defines	7
Privat Variables	7
Acceleration Sensor functions	7
LED Functions	8
Main function	9
Specifications for Final Hardware Design	9
STM32	9
RF-Antenna	10
Acceleration Sensor	10
DC – DC Converter	10
Wiring Diagram	11
PCB Layout	11
Results	13
Discussion	13

 
Introduction
Lights are an important factor when it comes to safety on the streets. Even on a Bicycle it is required by law to have a Front- and Backlight. The goal of this project is to create a power efficient backlight for your Bicycle. The brightness of the Light emitting diodes reacts to the acceleration Sensor which is implemented on the Device. 
With the use of an Stm32 Microcontroller, data between the acceleration sensor and the Microcontroller exchanges, so the Device know your movement and speed to regulate the brightness of the lights with the direct memory access (DMA). It is also possible to connect your Smartphone (BLE) and control the lights manually, for example to show the turning direction. The power supply is ensured by batteries. The idea comes from the FH Technikum. 

Motivation
Being on the road in a big city, by bike, can be dangerous. I am also a cyclist, but you must be careful when you are cycling in the city, because most of the time there is a lot of traffic and to be overlooked can be fatal. That's why safety is an important factor. It is still the best vehicle to get to quickly from one place to another if the distance is not too far. To increase visibility of the bike and the biker, an interacting light can be helpful. A light that lasts very long and due to the current rather minimalist designs of bicycle lights on the market, I decided to start this project. Also, the demand of bicycles has increased sharply during the last two years, due to Covid.
Video of functionality 


Methodology
STM32 
With the STM32L432kc Nucleon board the testing phase of all components and of the software has been realized. It has an onboard ST-Link debugger/programmer port and can be programmed with STM32CubeIDE in Programming language C. [3]
In the testing case the MCU was running with a frequency of 72Mhz. With the I²C Bus the Acceleration Sensor gets its connection to the STM32 Controller. The Bluetooth Modul is connected via UART to the MCU and the WS2811 LEDs get controlled with a Timer output channel in PWM Mode. The LED data are provided with DMA. 

Bluetooth Low Energy
The Bluetooth Low Energy (LE) radio is designed for very low power operation. Transmitting data over 40 channels in the 2.4GHz unlicensed ISM frequency band, the Bluetooth LE radio provides developers a tremendous amount of flexibility to build products that meet the unique connectivity requirements of their market. [2]
 
Acceleration Sensor 
With the Adafruit-LIS3DH the STM32L432kc gets its Bluetooth functionality. The Lish3dh is an ultra low power three-axis linear accelerometer with digital I²C/SPI serial interface standard output. The output data rates from 1 Hz to 5.3 kHz. The self-test capability allows the user to check the functioning of the sensor in the final application. It has also an tempratur sensor but it is not necessary for the project. 
The Microcontroller gets data from the acceleration sensor over the I²C Bus. Depending on this Data Stream, the LEDs get a timed signal over DMA to change the Brightness. 

LED Control (WS2811)
WS2811 LEDs are running with a Frequency of 800KHz. To get get the right timing for the LEDs, the internal Timer of the STM32 must be adjusted which will be explained later.  
The WS2812 is controlled via a single data line with an asynchronous serial protocol. A "0" is defined via a short and a "1" via a long high pulse. Each LED requires 24 data bits. The data of all LEDs are transmitted serially directly one after the other. If the data line is held low for more than 50µs (reset code), the data is transferred to the PWM registers of the LEDs.
The data for all the LEDs should be sent together. With 12 LEDs, 12×24 = 288 bits get transfered one after other.

 

 
Code Functionality

STM32 Pinout
 
Abbildung 3 STM32 Pinout
SWCLK and SWDIO are the interfaces for the Programming connection. RCC_OSC32_IN and _OUT is the connection to the external Oscillator of the development Board. MCO is the connection of the high-speed Oscillator. 
USART2_TX and _RX are used to communicate with the Adafruit Bluetooth Modul. The I2C1_SDA and I2C_SCL Pins are connected to the Acceleration Sensor LIS3DH. With the TIM1_CH1 Output, the LEDs are controlled. 
Clock Configuration
 
Abbildung 4 Clock Configuration
As you can see in the clock configuration, the PWM output of the Timer 1 is set to 72MHz. This value is used because it can be used well with the LED frequency. 
TIMER1 Configuration
 
Abbildung 5 Timer Configuration
With the Timer1, the LEDs get controlled. The Timer uses the internal Clock Frequency. Setting the Channel1 to PWM output is required to control the LEDs with the correct timing. 
The Clock frequency is 72MHz, by dividing it with 90, a frequency of 800Khz is the result, which matches the timing of the LEDs perfectly. The prescaler is 0 because we want to use the full range of 90 for better timing control. 
Defines
 
MAX_LED is the value for the number of LEDs which are used. PWM_HI and PWM_LO indicates the PWM duration. With the counter Period 89 of the Timer 1, To send a 1, we we keep the Pulse High for 2/3rd of the time for one period which equals the 60 in the define, because 60 is 2/3 of 89. The same case with sending a 0. The puls is set high for 1/3 of the PWM period, which is equal to 30. 

Privat Variables
 
With the Variable whoamI, the STM32 controller checks if the Accelaration Sensor was recognized. 
With int16_t data_raw_acceleration and acceleration_mg[3] we get the 3 differnet dircetions or Axis of the sensor. 
The acceleration_buff and _buffmg are fort he soft controlling of the light intensity of the LEDs. 
Uint8_t adafrui_buffer is the buffer who safes the data oft he UART connection oft he Bluetooth Modul. 
LED_Data safes the color information of an invidual LED. With DMAflag, the Transfer status of DMA gets checked. Uint16_t pwmData ist he buffer which will be sent tot he LEDs via DMA.

Acceleration Sensor functions 
 
 
With the included driver files(lis3dh_reg.h and .c) from the STM32 website, the acceleration sensor communicates with the STM32 correctly. With the write and read functions, the sensor gets the right communication interfaces and variables to handle a correct connection with the Microcontroller.


LED Functions
 
Every LED gets a number to identication. Also the colors Red, Green and Blue gets transmitted as integer (Range from 0 – 255). 

 
In this function, every value of the defined LEDs are send with Direct memory access (DMA). 
The first loop repeats for each Led. In this loop, a loop which repeats 24 times is implemented, it indicates a full Transfer of one LED data. This is done for every LED. After it is done, a Reset code is sent to indicate the LEDs that the transfer is finished. 
The full data stream is saved in the pwmData variable which is send to the DMA. 
With the DMAflag = 0 in the while loop, the status of the DMA Transfer gets monitored. If DMAflag = 1, there is a DMA Tranfere ongoing.  

 
When the DMA Transfer is finished, an interrupt occurs. In the function the PWM Output of the Timer gets stopped. And the DMAflag is set to 1. 

Main function
In the main function, first the acceleration sensor driver interface gets initialized, continue with checking if the sensor gets recognized. Then the initialization of the sensor starts. In the main while loop, the STM32 checks continuously new data from the Sensor in polling mode. In this while loop we also recieve data from the Bluetooth modul with Uart. The Brightness of every LED get set after revieving new data from the Sensor.  For more details, look in the main.c file. 

Specifications for Final Hardware Design
STM32
The STM32WB55xx and STM32WB35xx multiprotocol wireless and ultra-low-power devices embed a powerful and ultra-low-power radio compliant with the Bluetooth® Low Energy SIG specification 5.2 and with IEEE 802.15.4-2011. They contain a dedicated Arm® Cortex®-M0+ for performing all the real-time low layer operation. The devices are designed to be extremely low-power and are based on the high-performance Arm® Cortex®-M4 32-bit RISC core operating at a frequency of up to 64 MHz. The microcontroller can operate in ambient temperatures from -40°C to 125°C and it is operated with a supply voltage of 1.7 V to 3.6 V.  [1]
The STM32 runs with 3 V which is given from two 1.5 V Akali batteries.  It has some decoupling capacitor to decouple one part of the circuit from another. Noise caused by other circuit elements is shunted through the capacitors.
With a power switch the microcontroller gets disconnected from the batteries. The STM32 stays in sleep mode when no movement is detected and needs only 46 µA/MHz.
There are two external Oscillator connected, 32 KHz and 32MHz, for better accuracy than the internal oscillators and it is also needed for the RF function. 

RF-Antenna
The Bluetooth low energy functionality is given by the RF Antenna.  A small ceramic Antenna and a matching Circuit for the connection between the Microcontroller and the Ceramic Antenna is needed. The Line wide should match 50 Ohm. The Antenna is working in a Frequency of 2400 – 2480 MHz. As a result it is possible to use Bluetooth low energy 5.2 with the Stm32 Chip and so on the Microcontroller is on optimal power saving functionality. 

Acceleration Sensor 
For the Hardware Implementation an SPI communication is used for communication between the Microcontroller and the acceleration Sensor. I²C bus communication could also be possible but the SPI connection is more power saving. SPI is also a lot more power saving and simpler than Bluetooth or Wi-Fi connection, because less system resources are needed. 

 
Abbildung 6 SPI

DC – DC Converter
The two converters LM317 and LM2622 are for the LED power supply and for testing reasons. The stm32 gets its voltage directly from the source.  The LM317 is not needed, because it was just a additional possibility for testing cases. 
The LEDs. get 5V as power supply. 


Wiring Diagram 
Abbildung 7 PCB Wire Diagram
PCB Layout
 
Abbildung 8 PCB Layout
The Software gets flashed to the Microcontroller with the SWD Interface via a ST-Link programmer Device. The NRST Pin is required for this communication to reset the connection during a debug session.  

 
Abbildung 9 STM32WB35 Pinout

The Pinout configuration shows which output is used of the STM32WB25CCU6A. PA1 and PA4 are interrupt connections of the sensor to the STM32. PA6 and PA7 are necessary for the Acceleration connectivity. PA2 is just a simple LED control output. With PB1 a button is queried, for turning the lights on or off. PB3, PA13 and PA14 are used for programming the Microcontroller with the SWD interface. The Bluetooth Antenna is controlled with the RF1 outline from the Microcontroller. 
A Single data conversion is enough functionality use of the sensor. With a clock Signal, controlled by the STM32 chip, the sensor gives an output, for example every 5 seconds. When the output data is saved to an output register, the sensor goes to power-down mode, to save as much energy as possible.


Results
After programming the STM32 Development Board over the SWD interface, the software is running autonomous. The acceleration Sensor reacts on every little motion and changes the brightness of the LEDs. With the Bluetooth app from Adafruit, you can indicate the turning directing of your bicycle. Controlling with a smartphone gives an additional safety factor with indication the left and right turns of the Bicycle.

This project is only tested with the STM32L432KC development Board. The final Hardware Design did not work in the end. Since I soldered the board myself, there was an undetectable short circuit. 


Discussion
The order of starting this project were choosen wrong. I started with designing a PCB and then started with coding. In the end, the PCB didnt work and i couldnt test the software. Because of less time, i couldnt design and order a new one with the fear of having the same issue again. So as an emergency solution, the FH technikum borrowed me a STM32 Development board. It would have minimized the work if i had started with a development board first.
So there is still a problem with the Hardware. A short circuit is appeared on the board and could not be fixed. So the Code for the final Device could not be tested. 
A reasen of not finding the short circuit is the bad and too small desgined PCB Layout. Due to order difficulties, some electronic parts have a too small footprint for soldering it safly by hand.
The DC-DC Converter LM317 is not need and shouldn’t have been build in to reduce complexity of the circuit. It would have saved time to find errors that have arisen. 
Normally I would discuss the stability of the RF Network, because it is difficult to determine the correct dimensions of the electronic components to have the best possible signal strength for the Bluetooth connection. 
It is also not clearly to say if the external oscillators are really needed or if the internal ones are accurate enough for the RF Network. 



 
Abbildungsverzeichnis

Abbildung 1 WS2811 Data Transmission	4
Abbildung 2 WS2811 Timing	4
Abbildung 3 STM32 Pinout	5
Abbildung 4 Clock Configuration	6
Abbildung 5 Timer Configuration	6
Abbildung 6 SPI	10
Abbildung 7 PCB Wire Diagram	11
Abbildung 8 PCB Layout	11
Abbildung 9 STM32WB35 Pinout	12



[1] STM32WB35CC
https://www.st.com/en/microcontrollers-microprocessors/stm32wb35cc.html 

[2] Bluetooth SIG, Inc. Headquarters
https://www.bluetooth.com/learn-about-bluetooth/tech-overview/ 

[3] STM32 Nucleo-32 development board
https://www.st.com/en/evaluation-tools/nucleo-l432kc.html 


