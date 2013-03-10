
This is code which drives a microcontroller assembly for collecting accelerometer readings during a
collision event.  The hardware is composed of:

1.  A LeafLabs [Maple Mini](https://www.sparkfun.com/products/11280) board, featuring an STM32
    microcontroller

2.  An Analog Devices [ADXL377](http://www.analog.com/en/mems-sensors/mems-accelerometers/adxl377/products/product.html)
    high-g three-axis accelerometer, wired to STM32 analog inputs

3.  A Roving Networks [RN42](http://www.rovingnetworks.com/products/RN42) bluetooth module,
    wired to USART3 on the STM32 and used for transmitting accelerometer data for display on a
    remote workstation.

Directory *maple-mini* contains firmware for the STM32.
The [libmaple repository](https://github.com/leaflabs/maplemini) must be checked out as a
dependency.

Directory *view* contains Python code for downloading accelerometer data to a Linux PC over the
bluetooth link.


POOR MAN'S SCHEMATIC
====================
I'm too lazy to draw up a schematic for this, but here are the connections.  Maple Mini pin numbers
are the numbers written on the PCB, not the standard pin numbers associated with the for the 40-pin
package.

Maple Mini Power:
-----------------
* Batteries hooked up between VIN and GND pins, in range 3.3V to 12V.

Maple Mini to RN42XV:
---------------------
* MM pin VCC to RN pin 1 (VDD\_3V3)
* MM pin GND to RN pin 10 (GND)
* MM pin 0 (USART3\_RX) to RN pin 2 (TXD)
* MM pin 1 (USART3\_TX) to RN pin 3 (RXD)
* MM pin 30 (SPI2\_SCK/USART3\_CTS) to RN pin 12 (RTS)
* MM pin 29 (SPI2\_MISO/USART3\_RTS) to RN pin 16 (CTS)
* MM pin 28 (SPI2\_MOSI) to RN pin 5 (RESET\_N)
* MM pin 27 (PWM/USART1\_CK) to RN pin 13 (GPIO2, connection status)

Maple Mini to ADXL377 evaluation board:
---------------------------------------
* MM pin AV+ to ADXL pin 2 (VS)
* MM pin AV- to ADXL pin 6 (GND)
* MM pin 11 (ADC\_IN0) to ADXL pin 3 (X\_OUT)
* MM pin 10 (ADC\_IN1) to ADXL pin 4 (Y\_OUT)
* MM pin 9  (ADC\_IN2) to ADXL pin 5 (Z\_OUT)



LICENSE
=======
Copyright (c) 2013 Paul Pelzl  
All rights reserved. 

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met: 

1. Redistributions of source code must retain the above copyright notice, this list of conditions
   and the following disclaimer. 

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
   and the following disclaimer in the documentation and/or other materials provided with the
   distribution. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


