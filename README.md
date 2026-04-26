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
arduino-cli upload -v -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:mega ./MEGA__master
```

Notes:
- `arduino-cli upload` does not take a `--libraries` flag. Libraries are only needed at compile time.
- Compile first, then upload.

Verified example:

```sh
arduino-cli compile --fqbn arduino:avr:mega --libraries ./libraries ./MEGA__master
arduino-cli upload -v -p /dev/cu.usbmodem1401 --fqbn arduino:avr:mega ./MEGA__master
```

If you use Arduino IDE, open `MEGA__master/MEGA__master.ino`, select Arduino Mega 2560 and the correct port, then upload.
