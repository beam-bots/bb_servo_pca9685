<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Basic Usage

This tutorial shows you how to define a PCA9685 controller and servo-controlled
joints in your BB robot.

## Prerequisites

- Completed [Getting Started](1-getting-started.md)
- PCA9685 connected via I2C
- At least one servo connected to channel 0

## Defining a Robot with PCA9685 Servos

Create a robot module with a controller and servo-controlled joints:

```elixir
defmodule MyRobot do
  use BB

  controllers do
    controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
  end

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end
  end

  topology do
    link :base do
      joint :pan do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(60 degree_per_second)

        actuator :pan_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}

        link :head
      end
    end
  end
end
```

Three sections do the work. `controllers` holds robot-level components — one
entry per physical PCA9685 board. `topology` describes the physical structure as
a tree of links and joints, with each servo attached as an `actuator` inside its
joint. `commands` declares the arm and disarm commands; a robot starts
`:disarmed` and won't move until armed, so a robot without them can't be
commanded at all.

Component names must be unique across the whole robot — BB registers every
process under its name. That's why the actuator is `:pan_servo` rather than
`:servo`: the moment you add a second joint, a second `:servo` fails to compile.

## Understanding the Configuration

### Controller Options

The controller manages the I2C connection to the PCA9685:

```elixir
controller :pca9685, {BB.Servo.PCA9685.Controller,
  bus: "i2c-1",         # Required: I2C bus name
  address: 0x40,        # Required: I2C address; 0x40 is common for an unmodified board
  pwm_freq: 50,         # Optional: PWM frequency in Hz (default: 50)
  oe_pin: 25            # Optional: GPIO pin for output enable
}
```

- `bus` - The I2C bus device name (usually `"i2c-1"` on Raspberry Pi)
- `address` - The required I2C address of the PCA9685 (often `0x40`)
- `pwm_freq` - PWM frequency, 50 Hz is standard for servos
- `oe_pin` - Optional GPIO pin connected to the PCA9685's OE (Output Enable) pin

### Joint Limits

The `limit` entity defines the physical constraints of your joint:

- `effort` - Maximum force or torque (**required**)
- `velocity` - Maximum rotation speed (**required**, used for timing calculations)
- `lower` - Minimum position (maps to servo's minimum pulse)
- `upper` - Maximum position (maps to servo's maximum pulse)
- `acceleration` - Maximum acceleration; when omitted, motion timing assumes a
  rectangular velocity profile

These values are used by the actuator to:
1. Map positions to PWM pulse widths
2. Clamp commanded positions to safe values
3. Calculate expected movement duration

An RC servo won't report or obey a torque limit, but `effort` is required on
every joint, so give it a figure from the servo's datasheet.

### Actuator Options

The actuator controls a single servo channel:

```elixir
actuator :pan_servo, {BB.Servo.PCA9685.Actuator,
  channel: 0,          # Required: PCA9685 channel (0-15)
  controller: :pca9685, # Required: name of the controller
  min_pulse: 500,      # Optional: minimum pulse width in µs (default: 500)
  max_pulse: 2500      # Optional: maximum pulse width in µs (default: 2500)
}
```

Most servos work well with the defaults. Adjust `min_pulse` and `max_pulse` if
your servo has different endpoints.

## Starting the Robot

Start your robot in your application supervision tree:

```elixir
defmodule MyApp.Application do
  use Application

  def start(_type, _args) do
    children = [
      MyRobot
    ]

    opts = [strategy: :one_for_one, name: MyApp.Supervisor]
    Supervisor.start_link(children, opts)
  end
end
```

Or start it manually in IEx:

```elixir
iex> MyRobot.start_link()
{:ok, #PID<0.123.0>}
```

To try the robot without any hardware attached, start it in simulation:

```elixir
iex> MyRobot.start_link(simulation: :kinematic)
{:ok, #PID<0.123.0>}
```

Controllers default to `simulation: :omit`, so the real PCA9685 controller does
not start and no I2C traffic happens. Actuators are swapped for
`BB.Sim.Actuator`, and the open-loop position estimator from
[Position Feedback](3-position-feedback.md) works unchanged.

## Arming the Robot

A robot starts `:disarmed` and will not move. Arming is a command, not a flag —
run it and wait for the result:

```elixir
iex> {:ok, command} = MyRobot.arm()
{:ok, #PID<0.234.0>}

iex> BB.Command.await(command)
{:ok, :armed, [next_state: :idle]}
```

Each command you declare in the `commands` section becomes a function on the
robot module. Disarming works the same way, and pulls the PCA9685's OE pin high
if you've wired one:

```elixir
iex> {:ok, command} = MyRobot.disarm()
iex> BB.Command.await(command)
{:ok, :disarmed, [next_state: :disarmed]}
```

Drive safety state through these commands rather than calling `BB.Safety`
directly — going through the command system is what runs a robot's prearm
checks.

## Commanding the Servo

With the robot armed, send position commands by actuator name. `set_position/4`
publishes via pubsub, `set_position!/4` fires and forgets, and
`set_position_sync/5` waits for the actuator to acknowledge:

```elixir
# Move to centre (0 degrees)
BB.Actuator.set_position(MyRobot, :pan_servo, 0.0)

# Move to -45 degrees (in radians)
BB.Actuator.set_position!(MyRobot, :pan_servo, -0.785)

# Wait for acknowledgement
{:ok, :accepted} = BB.Actuator.set_position_sync(MyRobot, :pan_servo, -0.785)

# Using the unit sigil for degrees
import BB.Unit
BB.Actuator.set_position!(MyRobot, :pan_servo, BB.Robot.Units.to_radians(~u(-45 degree)))
```

> **Note:** The DSL takes `~u` sigil values, but the runtime command functions
> take plain numbers in SI base units — radians here. Convert with
> `BB.Robot.Units.to_radians/1`.

You command joints in **joint-space**. BB applies the joint's transmission and
hands this driver motor-space values, so the driver never does joint-to-motor
maths.

All three take either the actuator's unique name or its full path through the
topology (`[:base, :pan, :pan_servo]` here), and all three arrive at the
driver's `handle_command/2` — which transport you chose isn't something the
driver can see.

## Position Clamping

The actuator automatically clamps positions to the joint limits:

```elixir
# Joint limits are -90° to +90°
# This command will be clamped to +90° (π/2 radians)
BB.Actuator.set_position!(MyRobot, :pan_servo, 3.14)  # Requested: 180°, actual: 90°
```

## Reversing Direction

If your servo rotates in the opposite direction to the joint, reverse the
actuator's joint transmission:

```elixir
actuator :pan_servo, {BB.Servo.PCA9685.Actuator,
  channel: 0,
  controller: :pca9685
} do
  transmission do
    reversed? true
  end
end
```

The driver continues to map motor-space limits to PWM pulse widths; the
transmission reverses the mapping between joint space and motor space.

## Example: Pan-Tilt Head

Here's a complete example with two servos for a pan-tilt mechanism:

```elixir
defmodule PanTiltRobot do
  use BB

  controllers do
    controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
  end

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end
  end

  topology do
    link :base do
      joint :pan do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(90 degree_per_second)

        actuator :pan_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}

        link :pan_platform do
          joint :tilt do
            type :revolute

            limit lower: ~u(-45 degree),
                  upper: ~u(45 degree),
                  effort: ~u(1 newton_meter),
                  velocity: ~u(60 degree_per_second)

            actuator :tilt_servo, {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685}

            link :camera_mount
          end
        end
      end
    end
  end
end
```

Note that each servo has its own name. Naming both `:servo` is the most common
way to get a compile error here — names are global, not scoped to their joint.

Command both servos:

```elixir
{:ok, command} = PanTiltRobot.arm()
{:ok, :armed, _} = BB.Command.await(command)

# Look left and up
BB.Actuator.set_position!(PanTiltRobot, :pan_servo, -0.785)   # -45°
BB.Actuator.set_position!(PanTiltRobot, :tilt_servo, 0.524)   # +30°
```

## Example: Hexapod Leg (6 Servos)

The PCA9685's 16 channels make it ideal for multi-servo robots:

```elixir
defmodule HexapodLeg do
  use BB

  controllers do
    controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
  end

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end
  end

  topology do
    link :body do
      # Leg 1
      joint :leg1_coxa do
        type :revolute

        limit lower: ~u(-45 degree),
              upper: ~u(45 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(90 degree_per_second)

        actuator :leg1_coxa_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}

        sensor :leg1_coxa_feedback,
               {BB.Sensor.OpenLoopPositionEstimator, actuator: :leg1_coxa_servo}

        link :leg1_coxa_link do
          joint :leg1_femur do
            type :revolute

            limit lower: ~u(-90 degree),
                  upper: ~u(30 degree),
                  effort: ~u(1 newton_meter),
                  velocity: ~u(90 degree_per_second)

            actuator :leg1_femur_servo,
                     {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685}

            sensor :leg1_femur_feedback,
                   {BB.Sensor.OpenLoopPositionEstimator, actuator: :leg1_femur_servo}

            link :leg1_femur_link do
              joint :leg1_tibia do
                type :revolute

                limit lower: ~u(-120 degree),
                      upper: ~u(0 degree),
                      effort: ~u(1 newton_meter),
                      velocity: ~u(90 degree_per_second)

                actuator :leg1_tibia_servo,
                         {BB.Servo.PCA9685.Actuator, channel: 2, controller: :pca9685}

                sensor :leg1_tibia_feedback,
                       {BB.Sensor.OpenLoopPositionEstimator, actuator: :leg1_tibia_servo}

                link :leg1_foot
              end
            end
          end
        end
      end

      # Leg 2 uses channels 3, 4, 5
      # Leg 3 uses channels 6, 7, 8
      # ... and so on
    end
  end
end
```

With sixteen channels to name, a `<leg>_<segment>_servo` convention keeps every
actuator name unique without much thought.

## Multiple PCA9685 Boards

For robots with more than 16 servos, define multiple controllers:

```elixir
defmodule BigRobot do
  use BB

  controllers do
    # First board at its unmodified hardware address
    controller :pca9685_a, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}

    # Second board with A0 jumper set
    controller :pca9685_b, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x41}
  end

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end
  end

  topology do
    link :base do
      # First 16 servos use :pca9685_a
      joint :joint_0 do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(60 degree_per_second)

        actuator :servo_0, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685_a}

        link :link_0
      end

      # Servos 17+ use :pca9685_b
      joint :joint_16 do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(60 degree_per_second)

        actuator :servo_16, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685_b}

        link :link_16
      end
    end
  end
end
```

Both actuators sit on channel 0 — of different boards. It's the `controller:`
option that picks the board, and the actuator names that must differ.

## Output Enable Control

If you've connected the PCA9685's OE pin to a GPIO, you can enable/disable all
outputs:

```elixir
controller :pca9685, {BB.Servo.PCA9685.Controller,
  bus: "i2c-1",
  address: 0x40,
  oe_pin: 25  # GPIO 25 connected to OE
}
```

Control outputs via the controller:

```elixir
# Disable all servo outputs (servos go limp)
BB.Process.call(MyRobot, :pca9685, :output_disable)

# Re-enable outputs
BB.Process.call(MyRobot, :pca9685, :output_enable)
```

Without an `oe_pin` configured, both calls return
`{:error, %BB.Error.Hardware.NoOutputEnablePin{}}`.

This is useful for:
- Emergency stops
- Allowing manual positioning of servos
- Reducing power consumption when idle

Disarming pulls OE high for you, and a clean controller shutdown does the same
in the device's `terminate/2`. That makes `oe_pin` the only kill that survives a
dead controller process — both the actuator's disarm and the controller's own
route through the live controller, so if it has crashed, the disarm fails and
the robot enters `:error`.

## Next Steps

To get position feedback from your servos, see [Position Feedback](3-position-feedback.md).
