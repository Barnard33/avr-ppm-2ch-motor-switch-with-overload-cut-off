# avr-ppm-2ch-motor-switch-with-overload-cut-off

This software drives two gear motors with two PPM channels of your RC 
transmitter without the need for mechanical end-position switches.

It recognized the end-position of the gear motors by measuring the 
current with a shunt. 
If a specific cut-off current is reached, both gear motors are turned 
off and the status LED is activated.

To activate the gear motors again, the gimbals or three-way switches of 
your RC transmitter that control the motors must be moved back to the 
middle position. A deactivated status LED indicates, that the motors 
can be started again.

Although this setup uses an Arduino Pro Mini board, the software does
not make use of any Arduino libraries. It uses plain C with AVR GCC and
is directly flashed to the microcontroller via ISP without using the
Arduino bootloader.

The hardware setup is as follows:
* 1x Arduino Pro Mini, 5V variant at 16 MHz with an Microchip Atmega 328P
microcontroller (Atmega 328PB may also work)
* 1x DRV8833 based motor controller breakout board
* 1x 1 Ohm 0.5W metal film resistor with 1% accuracy as a shunt
* 2x 5V gear motor
* 1x RC receiver with at least two channels as PPM outputs used to 
control the motors

Wiring on a breadboard:
* Pro Mini pin "GND" --> GND
* Pro Mini pin "RAW" --> VCC
* Pro Mini pin "D2" --> Receiver channel for motor 1 PPM signal out
* Pro Mini pin "D3" --> Receiver channel for motor 2 PPM signal out
* Pro Mini pin "A0" --> Shunt pin 1
* Pro Mini pin "D4" --> DRV8833 "IN1"
* Pro Mini pin "D5" --> DRV8833 "IN2"
* Pro Mini pin "D6" --> DRV8833 "IN3"
* Pro Mini pin "D7" --> DRV8833 "IN4"
* DRV8833 pin "GND" --> Shunt pin 1
* Shunt pin 0 --> GND
* DRV8833 pin "VCC" --> VCC
* DRV8833 pin "OUT1" --> Motor 1 red side
* DRV8833 pin "OUT2" --> Motor 1 blue side
* DRV8833 pin "OUT3" --> Motor 2 red side
* DRV8833 pin "OUT4" --> Motor 2 blue side
* Receiver channel for motor 1 GND --> GND
* Receiver channel for motor 2 GND --> GND

Do not forget to power your receiver with a suitable power supply. 

Please, consult the datasheets of your Arduino board and your DRV8833 
breakout board to choose a suitable VCC supply voltage.

Also keep in mind that the voltage of the PPM signal of your receiver
shall never exceed the maximum ratings of the microcontroller. Consult 
the datasheet of the micrcontroller to learn about the allowed maximum 
ratings.

__WARNING__

__This source code (software) and the given wiring layout is only 
intended to be used for educational purposes. It is not production 
stable and must under no circumstance be used in any kind of radio 
controlled machinery, e.g., planes, cars, boats, etc.__
