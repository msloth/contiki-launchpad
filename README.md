# Contiki for Launchpad

This repo contains a port of Contiki for the TI MSP430 Launchpad.
It is aimed at the msp430g2553 as the 'g2452 has some limitations (RAM, no hw 
UART etc) that requires extra care that is at best rudimentary now and likely
won't happen soon.

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
Have you accidently used ''__bic_SR(...)'' instead of ''__bic_SR_onexit(...)''?
Or ''DIVS_2'' instead of ''DIVS__2''? The former will end up stuck in an ISR and
the latter will set the wrong clock speed. This Contiki port takes away a lot of
that pain, giving you an easy way to build stuff while being very open for you
to get down and dirty with the nitty-gritty details should you like to.

## Getting started

I recommend using Instant Contiki, which is an Ubuntu
virtual machine with mspgcc and other tools installed from start.
* download and install eg Virtualbox, VMWare Player or similar
  software for virtual machines. They are free for personal use.
* download and run the Instant Contiki VM.
  see www.contiki-os.org for download links to the latest version.
* in the virtual machine, open a terminal and clone this repository
    ''git clone git://github.com/MarcusLunden/contiki-launchpad.git contiki-launchpad''
* check that your compiler path is ok by going into eg 
    contiki-launchpad/examples/launchpad/blink
  and running
    ''msp430-gcc --version''
  Then try compiling it with
    ''make all''
  and uploading it to the Launchpad with
    ''make ul''
  (I don't remember if mspdebug is included in the VM, otherwise you may need to
  download it first and make sure it is in the path)
* Check the examples in contiki/examples/launchpad
  Follow the README in that folder and experiment. Learn and get a feel for 
  how to use processes, timers etc, working your way through and experimenting
  with processes.

## Limitations

The msp430g2553 is very capable for being such a small and inexpensive device but
is first and foremost limited in peripherals and RAM. For example, it has only
two timers and not much RAM and thus I had to cut down on a lot of Contiki 
features and simplify others to make it fit. Other limitations are some Contiki
features that I haven't ported yet.

### Not working on this port, not ported yet or very limited support
*  energest - energy estimation (save RAM)
*  uIPv6 - some parts may come later, depends on how small packetbuf+Rime gets (save RAM).
*  the sensors module was way too complex and hard to add new sensors so it is
   now replaced by a generic ADC module
*  the rimestats module - it needs 72 B RAM = too much. see ''contiki/core/net/rime/rimestats.h''.
*  packetbuf/Rime. I will remove quite many packetbuf attributes as they take a
   lot of RAM and are allocated even if they are used or not. This means that
   some Rime communication primitives do not work out of the box as they depend
   on eg the attribute for 'reliable' (ACK and retransmissions) so if you need
   those they will have to be re-implemented on top of this leaner Rime.
   (planned but not done yet...)

### New stuff, and some re-written to simplify/fit
*  adc - instead of the sensors API, there is now a generic ADC API with functions
   for synchronous and asynch conversions etc. Pick your flavor.
*  button - simple yet powerful button API
*  PWM API for doing simple pulse width modulation, for eg dimming LEDs, with a
   straight-forward API and flexible regarding pins, period and duty cycle. There
   are two APIs due to limited timers: the 'simple-pwm' uses the same hw-timer as
   the clock so even the 2452 has one, but the 'pwm' needs a separate hw-timer
   but gives two pwm-channels
*  Servo motor API
*  node-id is burnt into Infomem with the script in tools/launchpad/burnid
*  lots and lots of shrinking of buffers etc.

## Coming features

Of course, this is subject to change and this file might not be up-to-date.

*  radio drivers for CC2500 (they are there and almost fully working)
*  simple power-saving MAC protocol, like a re-implementation of X-MAC or ContikiMAC
   to reduce complexity and RAM/ROM-requirements --it's there, running at 7% idle DC with avg 64ms latency.
*  a driver for HD44780-based LCD displays
*  a driver for alphanumeric displays (a la Sparkfun) --done!
*  really simple (eg likely not that reliable or high performance) network stuff
   like leaf nodes to sink data routing, data dissemination through polite gossip
   and similar things.
*  UART with interrupts; now it is just blocking which works but as the speed is
   limited in hardware to 9600 baud any printf's can block the mcu quite long time
*  Contiki serial shell; now serial input works (wait for serial_input_events)
   but you have to parse the input yourself. I'll see how much space this takes,
   might have to skip it.
*  simple uni-directed UDP-messages
*  would be cool with some simple service discovery stuff, or growl notifications!
   (growl could work nice, it's mostly static strings anyway that easily fit in ROM)

## Most likely won't come

*  uIP; takes too much space
*  nothing but a bare minimum of routing/neighbor tables; takes way too much space
*  very generic SPI/I2C APIs as they are too application specific to be worthwhile


## What did I do to make it fit?

The biggest problem wasn't flash space (program and const variables), it's RAM.
By the way Contiki works, you use static variables a lot. This has the benefit
that an application becomes more easy to analyze as you know more about it at
compile time but the RAM requirements become harder. There is little hope of
doing any serious routing on these for example.

The protothread abstraction basically folds out (preprocessor macro expansion) into 
function calls with a big switch-case for each process; any YIELD or WAIT_EVENT
exits the function, hence any automatic (ie non-static or larger scope) variables
are not saved across the block. So you use static a lot, and so they are stored
in RAM. So I did simplify some things, removed others and managed to quite easily
squeeze it into the 16kB/512B of the 2553 without any radio or Rime. With Rime,
it's harder. The packet buffer, and queue buffer, needs a lot of space. I shrunk
the packetbuf and removed any queuebufs.

## Why no love for the 2452? And what about the 2231 and 2331?

The 2452, compared with the 2553, has one timer less, no hardware UART and only
256 byte RAM. This makes it much messier to use the 2452 with a Contiki that can
printf as the timer would be needed for software UART but is now used for clock
and rtimer. And just 256B RAM is quite narrow... But if you disable serial in
the platform/launchpad/contiki-conf.h you can still fit Contiki on it to do
simple things like buttons and LEDs and the like.

The 2231 and 2331 (?) where shipped with the early versions of Launchpad but are
even more restrained than the 2452 so I won't spread the love to them.

