# AGV

##Current Specs
- BLDC motor driver
- STM32F103C8 microcontroller
- LEDs on each phase's PWM output, so they will breath in sync with the speed of the motor
- Sense lines on all three phases, allowing sensorless operation
- UART connector
- SWD connector
- Vin up to 24V (need to double check voltage range on all parts)
- 3.3v linear reg (want to replace with switched)
- 3.3v rail indicator LED
- four quadrant operation (sw not yet implemented)


## TO DO
- add connection for rotary encoder
- add silkscreen indication for power input
- add bulk input capacitance
- get overall size down a little
- determine if parallel 'FETs are necessary for desired use cases.
- determine power handling capability (similar to above)

- usb port? depends on uC. Would need isolation probably
- isolated uart/i2c/spi port, for receiving commands from master/host
- add adc?

## Fixes:
- add power led on VBUS
- add led to EN_GATE pin to tell state
- increase drill size on stitching vias (currently unnecessarily small)
- fix/remove thermal relief on source pads for left most FETs
- add method to sense line voltages on each output phase, for back EMF sensing
- add fourth mounting hole
- increase package size where applicable (0402s are unnecessarily small in some places)
- do the capacitors on the back need to be there? maybe align them better at least
- improve fill polygon around C5 and C6
- align power leds more appropriately
