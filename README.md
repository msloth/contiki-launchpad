# Contiki for Launchpad

This repo contains a port of Contiki for the TI MSP430 Launchpad.
It is aimed at the msp430g2553 (out of the mcus delivered with LP)
as the 'g2452 has some limitations (RAM, no hw UART etc) that requires
extra care that is at best rudimentary now.

## Why Contiki @ LP?
Contiki serves an excellent middleground compared with the extremes of Arduino
(Energia on LP) and doing everything yourself from the ground up. In my opinion,
Arduino is too limited, especially when it comes to networking and complex
applications.

-The most common way Arduino-users seem to get at networking is
with an XBee, which in itself is very limited compared to Contiki/Rime. It
limits you in terms of routing, topology, saving power etc.

-Complex applications are often implemented as huge state-machines which
rapidly gets out of control. Contiki processes are lean, simple and powerful.

Doing it yourself from the ground up is a lesson in self-inflicted pain. Of
course, you learn a lot by doing it, and it is the way I started out myself, but
I think there is a lot of people that stopped playing with and enjoying stuff
like the LP just because it is too hard. Still, after several years, I find
doing the low-level stuff hard. Not to comprehend anymore, but to implement.
Accidently used ''__bic_SR(...)'' instead of ''__bic_SR_onexit(...)''? Or ''DIVS_2'' instead
of ''DIVS__2''? This takes away a lot of that pain, giving you a simple way of
doing simple things like a periodic ADC sample with LED blink and actuator control.


## Usage

To use this port, I recommend using Instant Contiki, which is an Ubuntu
virtual machine with mspgcc and other tools installed from start.
* download and install eg Virtualbox, VMWare Player or similar
  software for virtual machines. They are free for personal use.
* download and run the Instant Contiki VM.
  see www.contiki-os.org for download links to the latest version.
* clone this repository
* Check the examples in contiki/projects/
  start with hello-world and get a feel for how to use processes,
  timers etc, working your way through and experimenting with
  processes 

## Limitations

The 2553 is very capable for being such a small and inexpensive device, but it
comes with some limitations. For one, it haven't got much RAM and thus I had to
cut down on a lot of features and simplify others to make it fit. Other limitations
are Contiki features that I haven't ported yet.

### Not working/ported
*  energest - energy estimation (save RAM)
*  packetbuf, Rime. Coming later but I have no radio to port for yet.
*  uIPv6 - maybe later, depends on how small packetbuf+Rime gets (save RAM).
*  SPI, I2C - highly application dependent so I haven't done any such yet.
*  sensors - was way too complex and hard to add new sensors to (see below)

### Re-made to simplify/fit
*  sensors - instead of the sensors API, there is now a generic ADC API
*  button - simple yet powerful button API
  

## Coming features

Subject to change.

*  packetbuffer, Rime
*  radio drivers for CC2500
*  node-id, burning and reading
*  PWM API
*  UART with interrupts
*  Contiki serial shell
*  SPI, I2C
