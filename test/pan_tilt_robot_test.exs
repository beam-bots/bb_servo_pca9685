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

    assert BB.Actuator.set_position(PanTiltRobot, :pan_servo, 0.5) == :ok
    assert BB.Actuator.set_position(PanTiltRobot, :tilt_servo, -0.2) == :ok
  end

  test "feedback arrives on the joint's full topology path" do
    :ok = arm()
    BB.subscribe(PanTiltRobot, [:sensor, :base, :pan, :pan_feedback])

    :ok = BB.Actuator.set_position(PanTiltRobot, :pan_servo, 0.5)

    assert_receive {:bb, [:sensor, :base, :pan, :pan_feedback],
                    %Message{frame_id: :pan_feedback, payload: %JointState{} = joint_state}},
                   1000

    assert joint_state.names == [:pan]
    assert [position] = joint_state.positions
    assert is_float(position)
  end

  test "runtime reports positions keyed by joint name" do
    :ok = arm()
    BB.subscribe(PanTiltRobot, [:sensor, :base, :pan, :pan_feedback])

    :ok = BB.Actuator.set_position(PanTiltRobot, :pan_servo, 0.5)
    await_pan_feedback_at(0.5)

    # The runtime is another subscriber to the same feedback, so the estimator
    # has to finish publishing before syncing the runtime proves it has the
    # reading too.
    :sys.get_state(BB.Process.whereis(PanTiltRobot, :pan_feedback))
    :sys.get_state(Runtime.via(PanTiltRobot))

    assert %{pan: pan, tilt: _} = Runtime.configurations(PanTiltRobot)
    assert_in_delta pan, 0.5, 0.01
  end

  defp await_pan_feedback_at(target) do
    assert_receive {:bb, [:sensor, :base, :pan, :pan_feedback],
                    %Message{payload: %JointState{positions: [position]}}},
                   2_000

    if abs(position - target) >= 0.01, do: await_pan_feedback_at(target)
  end
end
