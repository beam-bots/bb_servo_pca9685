# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685 do
  @moduledoc """
  BB integration for driving RC servos via the PCA9685 PWM controller.

  This library provides controller, actuator and sensor modules for controlling
  RC servos through a PCA9685 16-channel PWM controller connected via I2C.

  ## Components

  - `BB.Servo.PCA9685.Controller` - Manages the PCA9685 device connection
  - `BB.Servo.PCA9685.Actuator` - Controls servo position via a controller channel
  - `BB.Servo.PCA9685.Sensor` - Provides position feedback by subscribing to actuator commands

  ## Requirements

  - PCA9685 PWM controller connected via I2C
  - The `pca9685` library for communication with the device

  ## Quick Start

  Define a controller and joints with servo actuators in your robot DSL:

      controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}

      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}
        sensor :feedback, {BB.Servo.PCA9685.Sensor, actuator: :servo}
      end

      joint :elbow, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(45 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685}
        sensor :feedback, {BB.Servo.PCA9685.Sensor, actuator: :servo}
      end

  The actuator automatically derives its configuration from the joint limits - no need
  to specify servo rotation range or speed separately.

  ## How It Works

  ### Controller

  The controller wraps a `PCA9685.Device` process and provides a stable reference
  for actuators. Multiple actuators can share a single controller, each using a
  different channel (0-15). The controller:
  - Manages the I2C connection to the PCA9685
  - Sets the PWM frequency (default 50Hz for servos)
  - Optionally controls an output enable pin

  ### Actuator

  The actuator maps the joint's position limits directly to the servo's PWM range:
  - Joint lower limit → minimum pulse width (default 500µs)
  - Joint upper limit → maximum pulse width (default 2500µs)
  - Centre position calculated as midpoint of limits

  When commanded to a position, the actuator:
  1. Clamps the position to joint limits
  2. Converts to PWM pulse width
  3. Sends command to the controller
  4. Publishes `PositionCommand` for sensors

  ### Sensor

  The sensor subscribes to actuator position commands and publishes `JointState`
  messages. It provides:
  - Position interpolation during movement
  - Configurable publish rate (default 50Hz)
  - Periodic sync publishing even when idle (default every 5 seconds)

  ## Multiple PCA9685 Boards

  For robots with more than 16 servos, you can define multiple controllers:

      controller :pca9685_a, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
      controller :pca9685_b, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x41}

      joint :shoulder, type: :revolute do
        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685_a}
        # ...
      end

      joint :gripper, type: :revolute do
        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685_b}
        # ...
      end
  """
end
