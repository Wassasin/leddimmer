# LED PWM dimmer
Quick low-side PWM device to crudely dim two 24V 4A LED strips.

Uses ESP32-C3 LEDC peripheral to provide the PWM signal, and uses two low side gate drivers to pull a N-channel Mosfet. Uses high side current limiters and fuses as protection.

## TODO
Pull-up esp32 enable
Pull-downs for drv-enable?
47nF filter voor isense
Swap drv2 isense en drvsense
WS2812B data line derp
Status line high side driver useless without output voltages
Caps C5750X7R1V476M230KC / 47uF 35V X7R 2220