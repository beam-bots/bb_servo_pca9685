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

BB integration for driving RC servos via PCA9685 16-channel PWM controller over I2C.

This library provides a controller and actuator module for controlling RC servos
connected to a PCA9685 board.

## Installation

Add `bb_servo_pca9685` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_servo_pca9685, "~> 0.3.0"}
  ]
end
```

## Requirements

- PCA9685 PWM controller connected via I2C
- BB framework (`~> 0.2`)

## Usage

Define a controller and joints with servo actuators in your robot DSL:

```elixir
defmodule MyRobot do
  use BB

  # Define the PCA9685 controller at robot level
  controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}

  topology do
    link :base do
      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}

        link :upper_arm do
          joint :elbow, type: :revolute do
            limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

            actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685}
            sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}

            link :forearm do
            end
          end
        end
      end
    end
  end
end
```

The actuator automatically derives its configuration from the joint limits - no
need to specify servo rotation range or speed separately.

## Sending Commands

Use the `BB.Actuator` module to send commands to servos. Three delivery methods
are available:

### Pubsub Delivery (for orchestration)

Commands are published via pubsub, enabling logging, replay, and multi-subscriber
patterns:

```elixir
# Send position command via pubsub
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5)

# With options
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5,
  command_id: make_ref()
)
```

### Direct Delivery (for time-critical control)

Commands bypass pubsub for lower latency. Use when responsiveness matters more
than observability:

```elixir
# Fire-and-forget
BB.Actuator.set_position!(MyRobot, :servo, 0.5)
```

### Synchronous Delivery (with acknowledgement)

Wait for the actuator to acknowledge the command:

```elixir
case BB.Actuator.set_position_sync(MyRobot, :servo, 0.5) do
  {:ok, :accepted} -> :ok
  {:error, reason} -> handle_error(reason)
end
```

## Components

### Controller

`BB.Servo.PCA9685.Controller` manages communication with the PCA9685 board.
Define one controller per physical PCA9685 device.

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `bus` | string | required | I2C bus name (e.g. "i2c-1") |
| `address` | integer | 0x40 | I2C address of the PCA9685 |
| `frequency` | integer | 50 | PWM frequency in Hz |
| `oe_pin` | integer | nil | Optional output-enable GPIO pin |

### Actuator

`BB.Servo.PCA9685.Actuator` controls a single servo on one of the 16 channels.

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `channel` | 0-15 | required | PCA9685 channel number |
| `controller` | atom | required | Name of the controller in robot registry |
| `min_pulse` | integer | 500 | Minimum PWM pulse width (microseconds) |
| `max_pulse` | integer | 2500 | Maximum PWM pulse width (microseconds) |
| `reverse?` | boolean | false | Reverse rotation direction |

**Behaviour:**

- Maps joint position limits directly to PWM range
- Clamps commanded positions to joint limits
- Publishes `BB.Message.Actuator.BeginMotion` after each command
- Calculates expected arrival time based on joint velocity limit

### Sensor

Use `BB.Sensor.OpenLoopPositionEstimator` from the BB core library for position
feedback. It subscribes to actuator `BeginMotion` messages and interpolates
position during movement.

```elixir
sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
```

## How It Works

### Architecture

```
Controller (GenServer)
    |
    v wraps
PCA9685.Device (I2C communication)
    ^
    | used by
Actuator (GenServer) --publishes--> BeginMotion --> Sensor (GenServer)
                                                        |
                                                        v publishes
                                                    JointState
```

Multiple actuators share a single controller. Each actuator controls one of the
16 available channels.

### Position Mapping

The actuator maps the joint's position limits to the servo's PWM range:

```
Joint lower limit  ->  min_pulse (500 microseconds)
Joint upper limit  ->  max_pulse (2500 microseconds)
Joint centre       ->  mid_pulse (1500 microseconds)
```

For a joint with limits `-45 degrees` to `+45 degrees`:
- `-45 degrees` maps to 500 microseconds
- `0 degrees` maps to 1500 microseconds
- `+45 degrees` maps to 2500 microseconds

### Position Feedback

Since RC servos don't provide position feedback, the open-loop position
estimator estimates position based on commanded targets and expected arrival
times:

1. Actuator sends command and publishes `BeginMotion` with expected arrival time
2. Sensor receives `BeginMotion` and interpolates position during movement
3. After arrival time, sensor reports the target position

This provides realistic position feedback for trajectory planning and monitoring.

### Motion Lifecycle

When a position command is processed:

1. Actuator clamps position to joint limits
2. Converts angle to PWM pulse width
3. Sends command to controller via `BB.Process.call`
4. Controller writes PWM to the PCA9685 over I2C
5. Publishes `BB.Message.Actuator.BeginMotion` with:
   - `initial_position` - where the servo was
   - `target_position` - where it's going
   - `expected_arrival` - when it should arrive (monotonic milliseconds)
   - `command_id` - correlation ID (if provided)
   - `command_type` - `:position`

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_servo_pca9685).
