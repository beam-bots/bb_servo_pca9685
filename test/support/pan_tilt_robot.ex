# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule PanTiltRobot do
  @moduledoc """
  The pan-tilt robot from the tutorials, compiled.

  This mirrors the pan-tilt robot the tutorials build, so the DSL they teach is
  checked by the build rather than by eye - a missing `effort`, a duplicated
  component name or a renamed section fails the suite instead of shipping.

  The tutorials omit brackets on DSL calls, per the house style in `bb`'s
  `usage-rules/dsl-topology.md`. This copy carries whatever `mix format`
  produces, which is not the same thing until `bb` exports its
  `locals_without_parens` under a spelling `import_deps` can read.
  """
  use BB

  controllers do
    controller(:pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40})
  end

  commands do
    command :arm do
      handler(BB.Command.Arm)
      allowed_states([:disarmed])
    end

    command :disarm do
      handler(BB.Command.Disarm)
      allowed_states([:idle])
    end
  end

  topology do
    link :base do
      joint :pan do
        type(:revolute)

        limit(
          lower: ~u(-90 degree),
          upper: ~u(90 degree),
          effort: ~u(1 newton_meter),
          velocity: ~u(60 degree_per_second)
        )

        actuator(:pan_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685})
        sensor(:pan_feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :pan_servo})

        link :pan_platform do
          joint :tilt do
            type(:revolute)

            limit(
              lower: ~u(-45 degree),
              upper: ~u(45 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(60 degree_per_second)
            )

            actuator(:tilt_servo, {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685})
            sensor(:tilt_feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :tilt_servo})

            link(:camera_mount)
          end
        end
      end
    end
  end
end
