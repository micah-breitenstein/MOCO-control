# MOCO-Control

Arduino Mega control firmware for the MOCO camera rig, coordinating multi-axis motion, PS2 operator input, and system-level control signals.

## Repository Layout

- `MEGA__master/MEGA__master.ino` - main Arduino Mega firmware
- `libraries/PS2X_lib/` - vendored PS2 controller library dependency

## Build

```sh
arduino-cli compile --fqbn arduino:avr:mega --libraries ./libraries ./MEGA__master
```

## Upload

```sh
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:mega --input-dir ./MEGA__master/build/arduino.avr.mega ./MEGA__master
```

If you use Arduino IDE, open `MEGA__master/MEGA__master.ino`, select Arduino Mega 2560 and the correct port, then upload.
