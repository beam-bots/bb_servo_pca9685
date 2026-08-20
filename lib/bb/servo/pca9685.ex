# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685 do
  @moduledoc """
  BB integration for driving RC servos via the PCA9685 PWM controller.

  This library provides controller and actuator modules for controlling RC
  servos through a PCA9685 16-channel PWM controller connected via I2C. The
  hardware reports nothing back, so pair every actuator with
  `BB.Sensor.OpenLoopPositionEstimator` from BB core - it is the only thing that
  publishes `BB.Message.Sensor.JointState` for a servo joint, and
  `BB.Robot.State` is written from those messages and from nothing else.

  ## Components

  - `BB.Servo.PCA9685.Controller` - Manages the PCA9685 device connection
  - `BB.Servo.PCA9685.Actuator` - Controls servo position via a controller channel
  - `BB.Sensor.OpenLoopPositionEstimator` - Estimates position from actuator motion messages

  ## Requirements

  - PCA9685 PWM controller connected via I2C
  - The `pca9685` library for communication with the device

  ## Quick Start

  Define a controller and joints with servo actuators in your robot DSL:

      controllers do
        controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
      end

      joint :shoulder do
        type :revolute

        limit lower: ~u(-45 degree),
              upper: ~u(45 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(60 degree_per_second)

        actuator :shoulder_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}

        sensor :shoulder_feedback,
               {BB.Sensor.OpenLoopPositionEstimator, actuator: :shoulder_servo}
      end

      joint :elbow do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(45 degree_per_second)

        actuator :elbow_servo, {BB.Servo.PCA9685.Actuator, channel: 1, controller: :pca9685}

        sensor :elbow_feedback,
               {BB.Sensor.OpenLoopPositionEstimator, actuator: :elbow_servo}
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
  4. Publishes `BB.Message.Actuator.BeginMotion` for sensors

  ### Open-loop position estimator

  Core's `BB.Sensor.OpenLoopPositionEstimator` subscribes to the actuator's
  `BB.Message.Actuator.BeginMotion` messages and publishes `JointState` messages.
  It provides:
  - Position interpolation during movement
  - Configurable publish rate (default 50Hz)
  - Periodic sync publishing even when idle (default every 5 seconds)

  Those `JointState` messages are what `BB.Robot.State` is built from, so a joint
  without an estimator stays at its initial configuration forever and anything
  reading joint positions - forward kinematics, the URDF visualisers, inverse
  kinematics - works from a robot that never moved. `BB.Dsl` warns at compile time
  about a joint nothing reports on.

  ## Multiple PCA9685 Boards

  For robots with more than 16 servos, you can define multiple controllers:

      controllers do
        controller :pca9685_a, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
        controller :pca9685_b, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x41}
      end

      joint :shoulder do
        type :revolute
        actuator :shoulder_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685_a}
        # ...
      end

      joint :gripper do
        type :revolute
        actuator :gripper_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685_b}
        # ...
      end

  Both actuators use channel 0 - of different boards. The `controller:` option
  picks the board; the actuator names must still differ, since names are unique
  across the whole robot.
  """
end
