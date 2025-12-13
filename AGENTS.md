<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

BB.Servo.PCA9685 is a Beam Bots integration library for driving RC servos via the PCA9685 16-channel PWM controller over I2C. It provides controller, actuator, and sensor modules that plug into the BB robotics framework's DSL.

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
    ↓ wraps
PCA9685.Device (I2C communication)
    ↑ used by
Actuator (GenServer) ←→ publishes PositionCommand → Sensor (GenServer)
                                                        ↓ publishes
                                                    JointState
```

### Key Modules

- **Controller** (`lib/bb/servo/pca9685/controller.ex`) - GenServer wrapping `PCA9685.Device`. Handles I2C bus connection, PWM frequency, and optional output-enable GPIO. Multiple actuators share one controller via channels 0-15.

- **Actuator** (`lib/bb/servo/pca9685/actuator.ex`) - GenServer that receives position commands (radians), converts to PWM pulse width based on joint limits, sends to controller, and publishes `PositionCommand` messages.

- **Sensor** (`lib/bb/servo/pca9685/sensor.ex`) - GenServer that subscribes to actuator's `PositionCommand`, interpolates position during movement, and publishes `JointState` messages at configurable rate.

### BB Framework Integration

The library uses BB's:
- `BB.Message` for typed message payloads
- `BB.publish`/`BB.subscribe` for hierarchical PubSub by path
- `BB.Process.call` to communicate with sibling processes via the robot registry
- `Spark.Options` for configuration validation
- Joint limits from robot topology to derive servo parameters

### Testing

Tests use Mimic to mock `BB`, `BB.Process`, `BB.Robot`, `PCA9685`, and `PCA9685.Device`. Test support modules are in `test/support/`.

## Dependencies

- `bb` - The Beam Bots robotics framework
- `pca9685` - Low-level PCA9685 PWM controller driver
