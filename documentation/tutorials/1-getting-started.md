<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Getting Started

This guide walks you through setting up a PCA9685 PWM controller to drive RC
servos with BB.Servo.PCA9685.

## Hardware Requirements

- Raspberry Pi (or any board with I2C support)
- PCA9685 16-channel PWM controller board
- RC servos (standard hobby servos with 3-wire connectors)
- 5V power supply for the servos
- Jumper wires

## Understanding the PCA9685

The PCA9685 is a 16-channel, 12-bit PWM controller that communicates via I2C.
Key features:

- **16 independent PWM channels** - Control up to 16 servos per board
- **I2C interface** - Only uses 2 GPIO pins regardless of servo count
- **Chainable** - Connect multiple boards for more channels (up to 62 boards)
- **Hardware PWM** - Precise timing without CPU load

### Board Pinout

```
PCA9685 Board
┌─────────────────────────────────────────┐
│  V+  VCC  SDA  SCL  GND  OE            │  ← Control side
├─────────────────────────────────────────┤
│  PWM0  PWM1  PWM2  ... PWM15           │  ← Servo outputs
│  (each has 3 pins: V+, GND, PWM)       │
└─────────────────────────────────────────┘
```

## Wiring

### I2C Connection

| PCA9685 | Raspberry Pi |
|---------|--------------|
| VCC | 3.3V (Pin 1) |
| GND | GND (Pin 6) |
| SDA | SDA (Pin 3, GPIO 2) |
| SCL | SCL (Pin 5, GPIO 3) |

### Servo Power

> **Important:** Servos draw significant current. Do NOT power servos from the
> Pi's 5V pin. Use an external 5V supply.

| Connection | Description |
|------------|-------------|
| V+ terminal | External 5V power supply (+) |
| GND | External 5V power supply (-) AND Pi GND |

### Servo Connection

Connect each servo to its channel (0-15):

| Servo Wire | PCA9685 Channel Pin |
|------------|---------------------|
| Brown/Black (GND) | GND row |
| Red (Power) | V+ row |
| Orange/Yellow (Signal) | PWM row |

### Wiring Diagram

```
                    External 5V Supply
                         │
                    ┌────┴────┐
                    │  + -    │
                    └────┬────┘
                         │
Raspberry Pi         PCA9685              Servos
───────────         ────────              ──────
3.3V ──────────────► VCC
GND ───────────────► GND ◄──── GND ◄───── All servo GND
SDA (GPIO 2) ──────► SDA                  (brown wire)
SCL (GPIO 3) ──────► SCL
                     V+ ◄───── +5V ◄───── All servo power
                                          (red wire)
                     PWM0 ────────────────► Servo 0 signal
                     PWM1 ────────────────► Servo 1 signal
                     ...
                     PWM15 ───────────────► Servo 15 signal
```

## Software Setup

### 1. Enable I2C on Raspberry Pi

```bash
# Using raspi-config
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable

# Or edit config directly
echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
sudo reboot
```

Verify I2C is enabled:

```bash
ls /dev/i2c*
# Should show: /dev/i2c-1
```

### 2. Install I2C Tools (Optional but Recommended)

```bash
sudo apt-get update
sudo apt-get install i2c-tools
```

Scan for the PCA9685:

```bash
i2cdetect -y 1
```

You should see the device at address `0x40` (default):

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: 70 -- -- -- -- -- -- --
```

### 3. Add Dependencies

Add `bb_servo_pca9685` to your `mix.exs`:

```elixir
def deps do
  [
    {:bb, "~> 0.2"},
    {:bb_servo_pca9685, "~> 0.1"}
  ]
end
```

Then fetch dependencies:

```bash
mix deps.get
```

## Verify Your Setup

Create a simple test to verify everything works:

```elixir
# In IEx on your Raspberry Pi
iex> {:ok, device} = PCA9685.acquire(bus: "i2c-1", address: 0x40)
{:ok, #PID<0.123.0>}

iex> PCA9685.Device.pulse_width(device, 0, 1500)
:ok
```

This should move the servo on channel 0 to its centre position. If you see
`:ok`, your setup is working correctly.

Release the device when done:

```elixir
iex> PCA9685.release(device)
:ok
```

## Multiple PCA9685 Boards

To use more than 16 servos, connect multiple PCA9685 boards to the same I2C bus
with different addresses. Change the address using the solder jumpers on the
board:

| Jumpers | Address |
|---------|---------|
| None | 0x40 |
| A0 | 0x41 |
| A1 | 0x42 |
| A0 + A1 | 0x43 |
| ... | ... |
| All (A0-A5) | 0x7F |

## Troubleshooting

### "No such file or directory" for /dev/i2c-1

I2C is not enabled:

```bash
sudo raspi-config
# Interface Options → I2C → Enable
sudo reboot
```

### Device not found at expected address

1. Check wiring connections
2. Verify VCC is connected to 3.3V (not 5V)
3. Run `i2cdetect -y 1` to scan for devices
4. Check address jumpers on the board

### Servo doesn't move

1. Check servo power supply is connected
2. Verify the channel number (0-15)
3. Try a different channel
4. Check servo wiring (signal to PWM row)

### Servo jitters or moves erratically

1. Use a separate power supply for servos (not the Pi's 5V)
2. Add capacitors across the servo power rails
3. Ensure ground is shared between Pi, PCA9685, and power supply

### I2C communication errors

1. Keep I2C wires short (under 50cm)
2. Check for loose connections
3. Try reducing I2C speed if using long wires

## Next Steps

Now that your hardware is set up, proceed to [Basic Usage](2-basic-usage.md) to
learn how to integrate servos into your BB robot.
