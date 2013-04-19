
This folder contains some examples that demonstrates different things with
Contiki and stuff I wrote for my Launchpad Contiki port, such as the pwm- and
adc-modules.

I suggest you start by checking out the examples in roughly this order:
  blink
    etimers, processes
  button
    button API
  readadc
    ADC API
  pwm
    PWM API
  servomotor
    PWM API with absolute period setting, using project-conf.h, changing PWM freq
  readserial-printf
    getting serial input
  blink-rtimer
    rtimers
  
and if you are interested and have the hardware for it,
  periodic-broadcast
    radio communication, Rime networking stack
    use contiki/tools/launchpad/burnid/burnid to burn a node id to the nodes

  alphanum-test
    testing the alphanumeric displays sold on Sparkfun Electronics (sparkfun.com)

  hpdl-display
    using a HPDL-1414 display to display text and numbers

  servomotor
    using the PWM to control a servo motor

other examples/projects  
  break-timer
  clock-hpdl1414
  pwm-led - pwm'ing a LED
  simplepwm-led - using the simplePWM (compatible even on '2452)
  simplepwm-callbacks - using the simplePWM with callbacks instead
  tempdemo - the TI stock example but re-written for Contiki
