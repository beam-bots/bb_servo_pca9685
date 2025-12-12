<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots PCA9685 servo control

[![CI](https://github.com/beam-bots/bb_servo_pca9685/actions/workflows/ci.yml/badge.svg)](https://github.com/beam-bots/bb_servo_pca9685/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb_servo_pca9685.svg)](https://hex.pm/packages/bb_servo_pca9685)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb_servo_pca9685)](https://api.reuse.software/info/github.com/beam-bots/bb_servo_pca9685)

# BB.Servo.PCA9685

BB integration for driving RC servos connected via a PCA9685 PWM driver connected via I2C.
This library provides actuator and sensor modules for controlling RC servos.

## Installation

Add `bb_servo_pca9685` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_servo_pca9685, "~> 0.1.0"}
  ]
end
```

## Requirements

- I2C connected PCA9685 device.
- BB framework (`~> 0.2`)

## Usage

Define a joint with a servo actuator in your robot DSL:

```elixir
defmodule MyRobot do
  use BB.Robot

  robot do
    link :base do
      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, pin: 17}
        sensor :feedback, {BB.Servo.PCA9685.Sensor, actuator: :servo}

        link :arm do
          # ...
        end
      end
    end
  end
end
```

The actuator automatically derives its configuration from the joint limits - no
need to specify servo rotation range or speed separately.

## Components

### Actuator

`BB.Servo.PCA9685.Actuator` controls servo position via PWM.

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `pin` | integer | required | GPIO pin number |
| `min_pulse` | integer | 500 | Minimum PWM pulse width (µs) |
| `max_pulse` | integer | 2500 | Maximum PWM pulse width (µs) |
| `reverse?` | boolean | false | Reverse rotation direction |
| `update_speed` | unit | 50 Hz | PWM update frequency |

**Behaviour:**

- Maps joint position limits directly to PWM range
- Clamps commanded positions to joint limits
- Publishes `{:position_commanded, angle, expected_arrival}` after each command
- Calculates expected arrival time based on joint velocity limit

### Sensor

`BB.Servo.PCA9685.Sensor` provides position feedback by subscribing to actuator commands.

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `actuator` | atom | required | Name of the actuator to subscribe to |
| `publish_rate` | unit | 50 Hz | Rate to check for position changes |
| `max_silence` | unit | 5 seconds | Max time between publishes (for sync) |

**Behaviour:**

- Subscribes to actuator position commands
- Publishes `JointState` messages when position changes
- Interpolates position during movement for smooth feedback
- Periodically publishes even when idle to keep subscribers in sync

## How It Works

### Position Mapping

The actuator maps the joint's position limits to the servo's PWM range:

```
Joint lower limit  →  min_pulse (500µs)
Joint upper limit  →  max_pulse (2500µs)
Joint centre       →  mid_pulse (1500µs)
```

For a joint with limits `-45°` to `+45°`:
- `-45°` maps to 500µs
- `0°` maps to 1500µs
- `+45°` maps to 2500µs

### Position Feedback

Since RC servos don't provide position feedback, the sensor estimates position
based on commanded targets and expected arrival times:

1. Actuator sends command and calculates expected arrival time from velocity limit
2. Sensor receives `{:position_commanded, target, arrival_time}`
3. During movement, sensor interpolates between previous and target positions
4. After arrival time, sensor reports the target position

This provides realistic position feedback for trajectory planning and monitoring.

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_servo_pca9685).
