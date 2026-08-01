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

- **Actuator** (`lib/bb/servo/pca9685/actuator.ex`) - GenServer that receives position commands (radians), converts to PWM pulse width based on joint limits, sends to controller, and publishes `BB.Message.Actuator.BeginMotion` messages. Accepts commands sent via:
  - `BB.Actuator.set_position/4` (pubsub)
  - `BB.Actuator.set_position!/4` (direct)
  - `BB.Actuator.set_position_sync/5` (synchronous)

  All three arrive at `handle_command/2`; `BB.Actuator.Server` checks arm state and applies
  the joint's transmission before the driver sees them.

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
# Arm first — a disarmed robot refuses commands before they reach the driver
{:ok, cmd} = MyRobot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)

# Every function takes either the actuator's unique name or its full path, and
# all three transports arrive at the driver's `handle_command/2`.
BB.Actuator.set_position(MyRobot, :pan_servo, 0.5)                    # pubsub
BB.Actuator.set_position(MyRobot, [:base, :pan, :pan_servo], 0.5)     # by path
BB.Actuator.set_position!(MyRobot, :pan_servo, 0.5)                   # direct
{:ok, :accepted} = BB.Actuator.set_position_sync(MyRobot, :pan_servo, 0.5)
```

### Integration Pattern

```elixir
defmodule MyRobot do
  use BB

  controllers do
    controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
  end

  topology do
    link :base do
      joint :pan do
        type :revolute

        limit lower: ~u(-45 degree), upper: ~u(45 degree),
              velocity: ~u(60 degree_per_second), effort: ~u(1 newton_meter)

        actuator :pan_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}

        sensor :pan_feedback,
               {BB.Sensor.OpenLoopPositionEstimator, actuator: :pan_servo}

        link :head do
        end
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

## Licensing headers

Every source file must carry an SPDX header — a `#`-style comment for code, an
HTML comment for Markdown, or a `<file>.license` sidecar for files that can't
hold comments (binaries, JSON, lockfiles). `mix check` runs `reuse lint` and
fails the build if one is missing.

When you create a new file, its `SPDX-FileCopyrightText` line must credit **the
user you are working for** — not you (the agent), and not this repo's original
author. Take their name from `git config user.name` (add their `user.email` if
you include one) and use the current year. Match the neighbouring files'
`SPDX-License-Identifier` (usually `Apache-2.0`):

```
SPDX-FileCopyrightText: <current year> <your user's name>

SPDX-License-Identifier: Apache-2.0
```

Never copy an existing file's copyright line onto a new file — that credits the
wrong person. When you only edit an existing file, leave its headers unchanged.
