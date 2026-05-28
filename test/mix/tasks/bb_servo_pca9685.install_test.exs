# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.BbServoPca9685.InstallTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  defp project_with_robot do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> apply_igniter!()
  end

  describe "controller" do
    test "uses param refs for bus and address" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    controller(
      + |      :pca9685,
      + |      {BB.Servo.PCA9685.Controller,
      + |       bus: param([:config, :pca9685, :bus]), address: param([:config, :pca9685, :address])}
      + |    )
      """)
    end

    test "uses a custom controller name when --name is given" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install", ["--name", "pwm"])
      |> assert_has_patch("lib/test/robot.ex", """
      + |    controller(
      + |      :pwm,
      """)
    end
  end

  describe "parameters group" do
    test "adds a :config.:pca9685 param group with bus and address" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    group :config do
      + |      group :pca9685 do
      + |        param(:bus, type: :string, doc: "I2C bus name (e.g. \\"i2c-1\\")")
      """)
    end
  end

  describe "application config" do
    test "writes the bus default to config/config.exs" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install")
      |> assert_creates("config/config.exs", """
      import Config
      config :test, Test.Robot, params: [config: [pca9685: [bus: "i2c-1"]]]
      """)
    end

    test "honours a custom --bus option" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install", ["--bus", "i2c-2"])
      |> assert_creates("config/config.exs", """
      import Config
      config :test, Test.Robot, params: [config: [pca9685: [bus: "i2c-2"]]]
      """)
    end

    test "reads the robot child opts from the application env" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install")
      |> assert_has_patch("lib/test/application.ex", ~s'''
      + |    children = [{Test.Robot, Application.get_env(:test, Test.Robot, [])}]
      ''')
    end
  end

  describe "formatter" do
    test "imports bb_servo_pca9685 into .formatter.exs" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install")
      |> assert_has_patch(".formatter.exs", """
      + |  import_deps: [:bb_servo_pca9685, :bb]
      """)
    end
  end

  describe "notice" do
    test "prints a topology snippet for the user to paste" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install")
      |> assert_has_notice(&String.contains?(&1, "BB.Servo.PCA9685.Actuator"))
    end
  end

  describe "idempotency" do
    test "running twice produces no further changes" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_pca9685.install")
      |> apply_igniter!()
      |> Igniter.compose_task("bb_servo_pca9685.install")
      |> assert_unchanged()
    end
  end
end
