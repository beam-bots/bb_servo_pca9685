<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# BB.Servo.PCA9685 Usage Rules

`bb_servo_pca9685` drives RC servos through a PCA9685 16-channel PWM board over
I²C for [Beam Bots](https://hexdocs.pm/bb). It supplies two DSL components: a
robot-level `BB.Servo.PCA9685.Controller` (a `BB.Controller` wrapping one
physical board) and a per-joint `BB.Servo.PCA9685.Actuator` (a `BB.Actuator`
driving one channel). For BB framework basics, see `bb`'s rules
(`mix usage_rules.sync <file> bb:all`); this file covers only what's specific to
this driver.

## Core principles

1. **One controller per board, many actuators per controller.** The controller
   owns the I²C connection; each actuator names it via `controller:` and claims
   one channel (`0..15`). A board has 16 channels — add a second controller (at
   a different `address:`) for more servos.
2. **Servo geometry comes from the joint, not the actuator.** The actuator reads
   the joint's `limit` (via the injected motor profile) and maps that motor
   range linearly onto the PWM pulse range. You never declare rotation range or
   speed on the actuator — only the pulse endpoints if the servo's defaults
   don't fit.
3. **Position feedback is open-loop.** RC servos report nothing back. Pair each
   actuator with core's `BB.Sensor.OpenLoopPositionEstimator`, which
   interpolates position from the `BeginMotion` message the actuator publishes.

## Installing

```sh
mix igniter.install bb_servo_pca9685
```

This adds a controller entry plus a `:config.:pca9685` param group to your robot
and sets the bus on the application child spec. Actuator and sensor entries are
per-joint and are **not** added automatically — the installer prints a snippet
to copy into each joint.

## Wiring it in

The controller goes at robot level; each servo is an actuator inside its joint,
with an open-loop estimator alongside it:

```elixir
defmodule MyRobot.Robot do
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

Command it with `BB.Actuator` once the robot is armed (values are joint-space
radians; BB applies the joint transmission before the driver sees motor-space):

```elixir
BB.Actuator.set_position(MyRobot.Robot, [:shoulder, :servo], ~u(30 degree) |> Localize.Unit.to_base())
BB.Actuator.set_position!(MyRobot.Robot, :servo, 0.5)
```

## Options

**Controller** (`controllers` slot):

| Option | Default | Meaning |
|---|---|---|
| `:bus` | required | I²C bus name, e.g. `"i2c-1"` |
| `:address` | required | I²C address, e.g. `0x40` |
| `:pwm_freq` | `50` | PWM frequency in Hz (50 suits most analogue servos) |
| `:oe_pin` | none | GPIO pin for hardware output-enable |

**Actuator** (per-joint `actuator` slot):

| Option | Default | Meaning |
|---|---|---|
| `:channel` | required | Board channel, `0..15` |
| `:controller` | required | Name of the controller entry to route through |
| `:min_pulse` | `500` | Pulse width (µs) mapped to the joint's lower limit |
| `:max_pulse` | `2500` | Pulse width (µs) mapped to the joint's upper limit |

## Anti-patterns

- **Don't try to set the servo's travel range or speed on the actuator.** There
  is no such option — it derives everything from the joint `limit`. Widen the
  joint limit or adjust `:min_pulse`/`:max_pulse` to match the physical servo.
- **Don't lean on `disarm/1` as a power cut if the controller might be dead.**
  Both the actuator's disarm (pulse → 0) and the controller's disarm (OE pin
  high) route through the live controller process; if it has crashed the disarm
  fails and the robot enters `:error`. For a kill that survives a dead
  controller, wire `:oe_pin` — a clean controller shutdown pulls it high in the
  device's `terminate/2`.
- **Don't expect the controller to run under simulation.** Controllers default
  to `simulation: :omit`, so the real PCA9685 controller does not start; set
  `simulation: :mock` or `:start` on the entry if you need it. Actuators are
  swapped for `BB.Sim.Actuator` and the open-loop estimator works unchanged.

## Further reading

- [bb_servo_pca9685 docs](https://hexdocs.pm/bb_servo_pca9685)
- `bb`'s actuator and safety rules (`bb:actuators`, `bb:safety-and-commands`)
