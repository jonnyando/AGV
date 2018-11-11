# AGV

## TODO:
- Add microcontroller (STM32, probably F103)
- add smps to power drv8303 and micro (3v3)
- add LEDs (Bus power, drv power, micro power, drv enable)
- bulk capacitance
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
