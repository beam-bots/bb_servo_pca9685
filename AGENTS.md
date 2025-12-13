<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

BB.Servo.PCA9685 is a Beam Bots integration library for driving RC servos via the PCA9685 16-channel PWM controller over I2C. It provides controller and actuator modules that plug into the BB robotics framework's DSL.

## Build and Test Commands

```bash
mix check --no-retry    # Run all checks (compile, test, format, credo, dialyzer, reuse)
mix test                # Run tests
mix test path/to/test.exs:42  # Run single test at line
mix format              # Format code
mix credo --strict      # Linting
```

The project uses `ex_check` - always prefer `mix check --no-retry` over running individual tools.

## Architecture

### Component Hierarchy

```
Controller (GenServer)
    |
    v wraps
PCA9685.Device (I2C communication)
    ^
    | used by
Actuator (GenServer) --publishes--> BeginMotion --> OpenLoopPositionEstimator
                                                        |
                                                        v publishes
                                                    JointState
```

### Key Modules

- **Controller** (`lib/bb/servo/pca9685/controller.ex`) - GenServer wrapping `PCA9685.Device`. Handles I2C bus connection, PWM frequency, and optional output-enable GPIO. Multiple actuators share one controller via channels 0-15.

- **Actuator** (`lib/bb/servo/pca9685/actuator.ex`) - GenServer that receives position commands (radians), converts to PWM pulse width based on joint limits, sends to controller, and publishes `BB.Message.Actuator.BeginMotion` messages. Handles commands via three delivery methods:
  - `handle_info/2` for pubsub delivery (`BB.Actuator.set_position/4`)
  - `handle_cast/2` for direct delivery (`BB.Actuator.set_position!/4`)
  - `handle_call/3` for synchronous delivery (`BB.Actuator.set_position_sync/5`)

### BB Framework Integration

The library uses BB's:
- `BB.Message` for typed message payloads
- `BB.Actuator` for sending commands to actuators
- `BB.publish`/`BB.subscribe` for hierarchical PubSub by path
- `BB.Process.call` to communicate with sibling processes via the robot registry
- `Spark.Options` for configuration validation
- Joint limits from robot topology to derive servo parameters
- `BB.Sensor.OpenLoopPositionEstimator` for position feedback (from BB core)

### Command Interface

Send commands using the `BB.Actuator` module:

```elixir
# Pubsub delivery (for orchestration/logging)
BB.Actuator.set_position(MyRobot, [:joint, :servo], 0.5)

# Direct delivery (fire-and-forget, lower latency)
BB.Actuator.set_position!(MyRobot, :servo, 0.5)

# Synchronous delivery (with acknowledgement)
{:ok, :accepted} = BB.Actuator.set_position_sync(MyRobot, :servo, 0.5)
```

### Integration Pattern

```elixir
defmodule MyRobot do
  use BB

  controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}

  topology do
    link :base do
      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
      end
    end
  end
end
```

### Testing

Tests use Mimic to mock `BB`, `BB.Process`, `BB.Robot`, `PCA9685`, and `PCA9685.Device`. Test support modules are in `test/support/`.

## Dependencies

- `bb` - The Beam Bots robotics framework
- `pca9685` - Low-level PCA9685 PWM controller driver

### Message Flow

```
BB.Actuator.set_position()
    |
    v
Actuator receives Command.Position
    |
    v
Actuator calls Controller with pulse width
    |
    v
Controller writes to PCA9685 via I2C
    |
    v
Actuator publishes BeginMotion
    |
    v
OpenLoopPositionEstimator interpolates position
    |
    v
Sensor publishes JointState
```
