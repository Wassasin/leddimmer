# LED PWM dimmer
Quick low-side PWM device to crudely dim two 24V 4A LED strips.

Uses ESP32-C3 LEDC peripheral to provide the PWM signal, and uses two low side gate drivers to pull a N-channel Mosfet. Uses high side current limiters and fuses as protection.