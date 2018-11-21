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
