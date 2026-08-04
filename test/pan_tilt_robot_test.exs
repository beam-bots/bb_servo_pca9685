# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule PanTiltRobotTest do
  @moduledoc """
  Checks the claims the tutorials make about `PanTiltRobot`.

  Runs under `:kinematic` simulation, so the PCA9685 controller is omitted and
  the actuators are swapped for `BB.Sim.Actuator` - what is under test here is
  the framework wiring the tutorials describe (paths, topics, message shapes),
  not the I2C driver, which `BB.Servo.PCA9685.ActuatorTest` covers.
  """
  use ExUnit.Case, async: false

  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Robot.Runtime

  setup do
    start_supervised!({PanTiltRobot, simulation: :kinematic})
    :ok
  end

  defp arm do
    {:ok, command} = PanTiltRobot.arm()
    {:ok, :armed, _} = BB.Command.await(command)
    :ok
  end

  test "the robot starts disarmed and arms via the declared command" do
    assert BB.Safety.state(PanTiltRobot) == :disarmed

    :ok = arm()

    assert BB.Safety.state(PanTiltRobot) == :armed
  end

  test "actuators are reachable by name" do
    :ok = arm()

    assert {:ok, :accepted} = BB.Actuator.set_position_sync(PanTiltRobot, :pan_servo, 0.5)
    assert {:ok, :accepted} = BB.Actuator.set_position_sync(PanTiltRobot, :tilt_servo, -0.2)
  end

  test "feedback arrives on the joint's full topology path" do
    :ok = arm()
    BB.subscribe(PanTiltRobot, [:sensor, :base, :pan, :pan_feedback])

    {:ok, :accepted} = BB.Actuator.set_position_sync(PanTiltRobot, :pan_servo, 0.5)

    assert_receive {:bb, [:sensor, :base, :pan, :pan_feedback],
                    %Message{frame_id: :pan_feedback, payload: %JointState{} = joint_state}},
                   1000

    assert joint_state.names == [:pan]
    assert [position] = joint_state.positions
    assert is_float(position)
  end

  test "runtime reports positions keyed by joint name" do
    :ok = arm()

    {:ok, :accepted} = BB.Actuator.set_position_sync(PanTiltRobot, :pan_servo, 0.5)
    assert_position_settles(:pan, 0.5)

    assert %{pan: _, tilt: _} = Runtime.configurations(PanTiltRobot)
  end

  defp assert_position_settles(joint, target, attempts \\ 40) do
    position = Map.get(Runtime.configurations(PanTiltRobot), joint)

    cond do
      abs(position - target) < 0.01 ->
        :ok

      attempts > 0 ->
        Process.sleep(50)
        assert_position_settles(joint, target, attempts - 1)

      true ->
        flunk("#{joint} settled at #{position}, expected #{target}")
    end
  end
end
