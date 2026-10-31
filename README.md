# WiFi RC Rover with Robotic Arm

A two-ESP32 remote-controlled rover with a 4-axis arm and gripper. A handheld controller (ESP32 + two joysticks + two potentiometers) talks to the rover (ESP32 + servos) over WiFi using UDP. The rover hosts its own WiFi access point, so no router or internet connection is needed.

> **Status:** [work in progress / working prototype - update this]

## Features

- **Drive and steering** from the left (rover) joystick
- **4-part arm:** base rotation, shoulder, elbow, and gripper (no wrist)
- **Synchronized arm motion:** one joystick axis moves the shoulder and elbow together between an upright pose and a low forward-reaching pose
- **Fine adjustment:** two potentiometers add an offset to the shoulder and elbow on top of the synchronized motion
- **Gripper:** hold the arm joystick button to close, release to reopen
- **Failsafe:** the rover stops driving if it hears nothing from the controller for 500 ms
- Controller and rover communicate over plain WiFi UDP (no ESP-NOW or Bluetooth)

## Repository layout

```
.
├── electrical/
│   ├── kicad/            KiCad project: controller and rover schematics
│   └── exports/          Exported schematic files
├── firmware/
│   ├── rc_controller/    Controller firmware (rc_controller.ino)
│   └── rc_rover/         Rover firmware (rc_rover.ino)
├── mechanical/
│   ├── rover/            Rover chassis and steering parts
│   ├── arm/              Arm and gripper parts
│   └── final/            Final rover and arm models
└── README.md
```

## How it works

1. The rover's ESP32 starts a WiFi access point named `RCCar` (default address `192.168.4.1`).
2. The controller's ESP32 joins that network.
3. 50 times a second, the controller reads its inputs, normalizes them (centering, deadzones), and sends one small UDP packet to the rover on port `4210`.
4. The rover turns each packet into servo commands, with speed limiting for smooth motion.

## Controls

| Input | Action |
|---|---|
| Rover joystick, Y axis | Drive forward / reverse |
| Rover joystick, X axis | Steer left / right |
| Arm joystick, X axis | Rotate arm base left / right |
| Arm joystick, Y axis | Move shoulder and elbow together (forward = reach out, back = return upright) |
| Arm joystick, button | Hold to close gripper, release to reopen |
| Shoulder pot (RV2) | Fine offset for the shoulder joint |
| Elbow pot (RV1) | Fine offset for the elbow joint |

With a pot centered, no offset is applied.

## Hardware

### Bill of materials

| Qty | Part | Notes |
|---|---|---|
| 2 | ESP32-DevKitC (original ESP32) | One for the controller, one for the rover |
| 2 | Analog thumb joysticks | Rover joystick (X/Y) and arm joystick (X/Y plus push switch) |
| 2 | Potentiometers | Shoulder and elbow trim |
| 2 | 100 nF capacitors | Noise filtering on the two pot wipers (controller) |
| 1 | Continuous rotation servo | Drive |
| 1 | 180° servo | Steering (rack and pinion) |
| 3 | 180° servos | Arm base, shoulder, elbow |
| 1 | Continuous rotation servo | Gripper |
| 1 | Buck converter | Steps the rover battery down to 5 V for the ESP32 |
| 2 | Battery packs | One per board |
| 2 | Power switches | One per board |

Check that your rover battery voltage is within your servos' rated range (many are limited to 6 V).

### Controller pinout

| Signal | GPIO | Notes |
|---|---|---|
| ROVER_X | 34 | ADC1, steering |
| ROVER_Y | 35 | ADC1, drive |
| ARM_BASE | 32 | ADC1, arm joystick X |
| ARM_FWD | 33 | ADC1, arm joystick Y |
| ARM_SHOULDER | 36 (VP) | ADC1, pot |
| ARM_ELBOW | 39 (VN) | ADC1, pot |
| GRIP_BTN | 14 | Digital, internal pull-up, active low |

All analog inputs are on ADC1 because ADC2 does not work while WiFi is on. The two pot wipers each have a 100 nF capacitor to ground for noise filtering. Joysticks and pots run from 3.3 V.

### Rover pinout

| Signal | GPIO | Servo type |
|---|---|---|
| SIG_DRIVE | 13 | Continuous |
| SIG_STEER | 14 | 180° |
| SIG_GRIP | 32 | Continuous |
| SIG_BASE | 25 | 180° |
| SIG_SHOULDER | 26 | 180° |
| SIG_ELBOW | 27 | 180° |

### Power notes

- Power servos from their own supply, not from the ESP32's regulator. Tie all grounds together.
- Servo current spikes can reset the ESP32. Power the ESP32 through the buck converter rather than directly from the servo supply.
- Do not connect a battery and USB to the controller at the same time unless a diode protects the supply.

## Software setup

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) and add ESP32 board support (**Espressif Systems esp32** in the Boards Manager).
2. Install the **ESP32Servo** library from the Library Manager (rover only).
3. Open `firmware/rc_rover/rc_rover.ino`, select **ESP32 Dev Module**, and upload it to the rover's ESP32.
4. Open `firmware/rc_controller/rc_controller.ino` and upload it to the controller's ESP32.
5. **Change the WiFi password** (`WIFI_PASS`) in both files. It must be identical in both and at least 8 characters.

## First run and calibration

1. Fold the arm upright and center the steering before powering the rover. The servos snap to their start positions.
2. Lift the drive wheels off the ground for the first test.
3. Leave both joysticks untouched while the controller boots. It measures their center position at startup.
4. Open the controller's serial monitor (115200 baud). Centered sticks should read about 0 and full deflection about ±100. If an axis never reaches ±100, adjust `ADC_MIN` / `ADC_MAX`.
5. Fix any reversed controls with the `INV_` flags in the controller sketch or the `_DIR` constants in the rover sketch.
6. Tune the values in the rover sketch:
   - `DRIVE_STOP_US`: where the drive servo sits perfectly still (also adjustable with the trim screw on the servo)
   - `STEER_CENTER_DEG` and `STEER_MAX_DEG`: straight-ahead position and steering travel
   - `GRIP_TRAVEL_MS`: time for one full 360° gripper turn
   - `SH_UPRIGHT_DEG` / `EL_UPRIGHT_DEG`: servo angle that makes each joint straight up
   - `SH_MAX_TILT` / `EL_MAX_BEND`: how far each joint goes at full reach

## Configuration summary

Most tuning lives in constants at the top of each sketch.

| Sketch | What to adjust |
|---|---|
| `rc_controller.ino` | WiFi credentials, pins, `INV_` direction flags, deadzones, ADC range, debug output |
| `rc_rover.ino` | WiFi credentials, pins, servo centers and limits, arm poses, speeds, failsafe timeout |

## Known limitations

- The gripper is timed, not sensed. It assumes it starts open, and a blocked gripper can slip relative to its estimated position.
- The arm has no position feedback, so it jumps to its start pose at power-on.
- Range is that of the ESP32's onboard antenna: roughly a house or large yard. An ESP32 module with an external antenna would extend it.

## License

[Choose a license, e.g. MIT, and add a LICENSE file.]
