
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


