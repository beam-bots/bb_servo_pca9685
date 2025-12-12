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
  use BB.Robot

  robot do
    # Define the PCA9685 controller
    controller :pca9685, {BB.Servo.PCA9685.Controller,
      bus: "i2c-1",
      address: 0x40
    }

    link :base do
      joint :pan, type: :revolute do
        # Define the joint's motion limits
        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              velocity: ~u(60 degree_per_second)

        # Attach the servo actuator on channel 0
        actuator :servo, {BB.Servo.PCA9685.Actuator,
          channel: 0,
          controller: :pca9685
        }

        link :head do
          # Child links go here
        end
      end
    end
  end
end
```

## Understanding the Configuration

### Controller Options

The controller manages the I2C connection to the PCA9685:

```elixir
controller :pca9685, {BB.Servo.PCA9685.Controller,
  bus: "i2c-1",         # Required: I2C bus name
  address: 0x40,        # Required: I2C address (default for PCA9685)
  pwm_freq: 50,         # Optional: PWM frequency in Hz (default: 50)
  oe_pin: 25            # Optional: GPIO pin for output enable
}
```

- `bus` - The I2C bus device name (usually `"i2c-1"` on Raspberry Pi)
- `address` - The I2C address of the PCA9685 (default `0x40`)
- `pwm_freq` - PWM frequency, 50 Hz is standard for servos
- `oe_pin` - Optional GPIO pin connected to the PCA9685's OE (Output Enable) pin

### Joint Limits

The `limit` block defines the physical constraints of your joint:

- `lower` - Minimum position (maps to servo's minimum pulse)
- `upper` - Maximum position (maps to servo's maximum pulse)
- `velocity` - Maximum rotation speed (used for timing calculations)

These values are used by the actuator to:
1. Map positions to PWM pulse widths
2. Clamp commanded positions to safe values
3. Calculate expected movement duration

### Actuator Options

The actuator controls a single servo channel:

```elixir
actuator :servo, {BB.Servo.PCA9685.Actuator,
  channel: 0,          # Required: PCA9685 channel (0-15)
  controller: :pca9685, # Required: name of the controller
  min_pulse: 500,      # Optional: minimum pulse width in µs (default: 500)
  max_pulse: 2500,     # Optional: maximum pulse width in µs (default: 2500)
  reverse?: false      # Optional: reverse rotation direction (default: false)
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

## Commanding the Servo

Send position commands to the actuator:

```elixir
# Move to centre (0 degrees)
BB.Actuator.set_position(MyRobot, :servo, 0.0)

# Move to -45 degrees (in radians)
BB.Actuator.set_position(MyRobot, :servo, -0.785)

# Using the unit sigil for degrees
import BB.Unit
BB.Actuator.set_position(MyRobot, :servo, ~u(-45 degree) |> BB.Robot.Units.to_radians())
```

> **Note:** BB uses radians internally. Convert degrees to radians when sending
> commands, or use the unit conversion functions.

## Position Clamping

The actuator automatically clamps positions to the joint limits:

```elixir
# Joint limits are -90° to +90°
# This command will be clamped to +90° (π/2 radians)
BB.Actuator.set_position(MyRobot, :servo, 3.14)  # Requested: 180°, actual: 90°
```

## Reversing Direction

If your servo rotates in the opposite direction to what you expect, use the
`reverse?` option:

```elixir
actuator :servo, {BB.Servo.PCA9685.Actuator,
  channel: 0,
  controller: :pca9685,
  reverse?: true
}
```

This inverts the PWM mapping so that:
- Lower limit → maximum pulse
- Upper limit → minimum pulse

## Example: Pan-Tilt Head

Here's a complete example with two servos for a pan-tilt mechanism:

```elixir
defmodule PanTiltRobot do
  use BB.Robot

  robot do
    controller :pca9685, {BB.Servo.PCA9685.Controller,
      bus: "i2c-1",
      address: 0x40
    }

    link :base do
      joint :pan, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(90 degree_per_second)
        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}

        link :pan_platform do
          joint :tilt, type: :revolute do
            limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)
            actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685}

            link :camera_mount do
              # Camera attached here
            end
          end
        end
      end
    end
  end
end
```

Command both servos:

```elixir
# Look left and up
BB.Actuator.set_position(PanTiltRobot, :pan, -0.785)   # -45°
BB.Actuator.set_position(PanTiltRobot, :tilt, 0.524)   # +30°
```

## Example: Hexapod Leg (6 Servos)

The PCA9685's 16 channels make it ideal for multi-servo robots:

```elixir
defmodule HexapodLeg do
  use BB.Robot

  robot do
    controller :pca9685, {BB.Servo.PCA9685.Controller,
      bus: "i2c-1",
      address: 0x40
    }

    link :body do
      # Leg 1
      joint :leg1_coxa, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(90 degree_per_second)
        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}

        link :leg1_coxa_link do
          joint :leg1_femur, type: :revolute do
            limit lower: ~u(-90 degree), upper: ~u(30 degree), velocity: ~u(90 degree_per_second)
            actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685}

            link :leg1_femur_link do
              joint :leg1_tibia, type: :revolute do
                limit lower: ~u(-120 degree), upper: ~u(0 degree), velocity: ~u(90 degree_per_second)
                actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 2, controller: :pca9685}

                link :leg1_foot do
                end
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

## Multiple PCA9685 Boards

For robots with more than 16 servos, define multiple controllers:

```elixir
defmodule BigRobot do
  use BB.Robot

  robot do
    # First board at default address
    controller :pca9685_a, {BB.Servo.PCA9685.Controller,
      bus: "i2c-1",
      address: 0x40
    }

    # Second board with A0 jumper set
    controller :pca9685_b, {BB.Servo.PCA9685.Controller,
      bus: "i2c-1",
      address: 0x41
    }

    link :base do
      # First 16 servos use :pca9685_a
      joint :joint_0, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)
        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685_a}
        link :link_0 do end
      end

      # Servos 17+ use :pca9685_b
      joint :joint_16, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)
        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685_b}
        link :link_16 do end
      end
    end
  end
end
```

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

This is useful for:
- Emergency stops
- Allowing manual positioning of servos
- Reducing power consumption when idle

## Next Steps

To get position feedback from your servos, see [Position Feedback](3-position-feedback.md).
