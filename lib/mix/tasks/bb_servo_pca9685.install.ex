# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbServoPca9685.Install do
    @shortdoc "Installs BB.Servo.PCA9685 into a robot"
    @moduledoc """
    #{@shortdoc}

    Adds a `BB.Servo.PCA9685.Controller` to your robot module, defines a
    `:config.:pca9685` param group for the I2C bus and address, and sets the
    bus name on the robot's child spec in your application module.

    Actuator and sensor entries belong on individual joints and are not added
    automatically — a snippet is printed for you to copy.

    ## Example

    ```bash
    mix igniter.install bb_servo_pca9685
    mix igniter.install bb_servo_pca9685 --bus i2c-2
    ```

    ## Options

    * `--robot` - The robot module (defaults to `{AppPrefix}.Robot`).
    * `--name` - The controller name (default `pca9685`).
    * `--bus` - The I2C bus name (default `i2c-1`).
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Formatter

    @param_group :pca9685
    @default_bus "i2c-1"

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [
          robot: :string,
          name: :string,
          bus: :string
        ],
        aliases: [r: :robot, n: :name]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      options = igniter.args.options
      robot_module = BB.Igniter.robot_module(igniter)
      name = options |> Keyword.get(:name, "pca9685") |> String.to_atom()
      bus = Keyword.get(options, :bus, @default_bus)

      igniter
      |> Formatter.import_dep(:bb_servo_pca9685)
      |> BB.Igniter.add_controller(robot_module, name, controller_code(name))
      |> BB.Igniter.add_param_group(robot_module, [:config, @param_group], param_group_body())
      |> BB.Igniter.set_robot_param_default(robot_module, [:config, @param_group, :bus], bus)
      |> Igniter.add_notice(topology_snippet(name))
    end

    defp controller_code(name) do
      """
      controller :#{name}, {BB.Servo.PCA9685.Controller,
        bus: param([:config, :#{@param_group}, :bus]),
        address: param([:config, :#{@param_group}, :address])}
      """
    end

    defp param_group_body do
      """
      param :bus, type: :string, doc: "I2C bus name (e.g. \\"i2c-1\\")"

      param :address,
        type: :integer,
        default: 0x40,
        doc: "I2C address of the PCA9685"
      """
    end

    defp topology_snippet(controller_name) do
      """
      bb_servo_pca9685: add actuators/sensors to your joints. Example:

          joint :shoulder, type: :revolute do
            limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

            actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :#{controller_name}}
            sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
          end
      """
    end
  end
else
  defmodule Mix.Tasks.BbServoPca9685.Install do
    @shortdoc "Installs BB.Servo.PCA9685 into a robot"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb_servo_pca9685.install task requires igniter.

          mix igniter.install bb_servo_pca9685
      """)

      exit({:shutdown, 1})
    end
  end
end
