# MAVLinkEFIS
Arduino based MAVLink miniature glass cockpit display

The display part is based on bodmer's work, see https://forum.arduino.cc/index.php?topic=417918.msg2883135#msg2883135 and https://github.com/Bodmer for reference.

Arduino MAVLink code by Juan Pedro López, see https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566 for a great tutorial.

Indiviual hardware used is an arduino pro mini and a 0.96" 80 x 160 st7735-based RGB oled display to fit a ~ 1:5 scale glider cockpit.

### Wiring ###

![MAVLinkEFIS](https://github.com/vierfuffzig/MAVLinkEFIS/blob/main/MAVLinkEFIS.jpeg)


### Config ###

Default input 57.6 kbaud MAVLink on hardware serial. Set stream rates for attitude (EXTRA1) & vfr_hud (EXTRA2) if required.



