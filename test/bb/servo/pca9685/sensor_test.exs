# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685.SensorTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Message
  alias BB.Servo.PCA9685.Message.PositionCommand
  alias BB.Servo.PCA9685.Sensor

  import BB.Unit

  @joint_name :test_joint
  @sensor_name :test_feedback
  @actuator_name :test_servo

  defp position_command(target, expected_arrival) do
    Message.new!(PositionCommand, @joint_name,
      target: target,
      expected_arrival: expected_arrival
    )
  end

  defp default_bb_context do
    %{robot: TestRobot, path: [@joint_name, @sensor_name]}
  end

  defp init_sensor(opts \\ []) do
    stub(BB, :subscribe, fn _robot, _path -> :ok end)

    default_opts = [bb: default_bb_context(), actuator: @actuator_name]
    {:ok, state} = Sensor.init(Keyword.merge(default_opts, opts))

    state
  end

  describe "init/1" do
    test "subscribes to actuator topic" do
      test_pid = self()

      expect(BB, :subscribe, fn robot, path ->
        send(test_pid, {:subscribed, robot, path})
        :ok
      end)

      opts = [bb: default_bb_context(), actuator: @actuator_name]
      {:ok, _state} = Sensor.init(opts)

      assert_receive {:subscribed, TestRobot, [:actuator, @joint_name, @actuator_name]}
    end

    test "schedules first publish" do
      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      opts = [bb: default_bb_context(), actuator: @actuator_name]
      {:ok, _state} = Sensor.init(opts)

      assert_receive :publish, 100
    end

    test "calculates correct publish interval from rate" do
      state = init_sensor(publish_rate: ~u(100 hertz))

      assert state.publish_interval_ms == 10
    end

    test "calculates correct max silence interval" do
      state = init_sensor(max_silence: ~u(10 second))

      assert state.max_silence_ms == 10_000
    end
  end

  describe "position_commanded handling" do
    test "stores target and expected arrival" do
      state = init_sensor()
      expected_arrival = System.monotonic_time(:millisecond) + 500

      {:noreply, new_state} =
        Sensor.handle_info(position_command(0.5, expected_arrival), state)

      assert new_state.target == 0.5
      assert new_state.expected_arrival == expected_arrival
    end

    test "stores previous position for interpolation" do
      state = init_sensor()
      arrival1 = System.monotonic_time(:millisecond) + 500

      {:noreply, state} = Sensor.handle_info(position_command(0.5, arrival1), state)

      arrival2 = System.monotonic_time(:millisecond) + 500
      {:noreply, new_state} = Sensor.handle_info(position_command(1.0, arrival2), state)

      assert new_state.previous_position == 0.5
      assert new_state.target == 1.0
    end
  end

  describe "publish behaviour" do
    test "does not publish when no target set" do
      state = init_sensor()

      reject(&BB.publish/3)

      {:noreply, _state} = Sensor.handle_info(:publish, state)

      refute_receive {:published, _, _, _}
    end

    test "publishes JointState when position changes" do
      state = init_sensor()
      arrival = System.monotonic_time(:millisecond) - 100

      {:noreply, state} = Sensor.handle_info(position_command(0.5, arrival), state)

      test_pid = self()

      expect(BB, :publish, fn robot, path, message ->
        send(test_pid, {:published, robot, path, message})
        :ok
      end)

      {:noreply, _state} = Sensor.handle_info(:publish, state)

      assert_receive {:published, TestRobot, [:sensor, @joint_name, @sensor_name], message}
      assert message.payload.positions == [0.5]
    end

    test "does not publish when position unchanged" do
      state = init_sensor()
      arrival = System.monotonic_time(:millisecond) - 100

      {:noreply, state} = Sensor.handle_info(position_command(0.5, arrival), state)

      stub(BB, :publish, fn _robot, _path, _message -> :ok end)

      {:noreply, state} = Sensor.handle_info(:publish, state)

      reject(&BB.publish/3)

      {:noreply, _state} = Sensor.handle_info(:publish, state)
    end

    test "publishes on max_silence timeout even when position unchanged" do
      state = init_sensor(max_silence: ~u(0.01 second))
      arrival = System.monotonic_time(:millisecond) - 100

      {:noreply, state} = Sensor.handle_info(position_command(0.5, arrival), state)

      stub(BB, :publish, fn _robot, _path, _message -> :ok end)
      {:noreply, state} = Sensor.handle_info(:publish, state)

      Process.sleep(15)

      test_pid = self()

      expect(BB, :publish, fn _robot, _path, _message ->
        send(test_pid, :published_on_timeout)
        :ok
      end)

      {:noreply, _state} = Sensor.handle_info(:publish, state)

      assert_receive :published_on_timeout
    end
  end

  describe "position interpolation" do
    test "returns target when movement complete" do
      state = init_sensor()
      arrival = System.monotonic_time(:millisecond) - 100

      {:noreply, state} = Sensor.handle_info(position_command(0.5, arrival), state)

      test_pid = self()

      expect(BB, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      {:noreply, _state} = Sensor.handle_info(:publish, state)

      assert_receive {:position, 0.5}
    end

    test "interpolates position during movement" do
      state = init_sensor()
      now = System.monotonic_time(:millisecond)
      arrival = now + 1000

      state = %{
        state
        | target: 1.0,
          expected_arrival: arrival,
          previous_position: 0.0,
          command_time: now
      }

      Process.sleep(100)

      test_pid = self()

      expect(BB, :publish, fn _robot, _path, message ->
        send(test_pid, {:position, hd(message.payload.positions)})
        :ok
      end)

      {:noreply, _state} = Sensor.handle_info(:publish, state)

      assert_receive {:position, position}
      assert position > 0.0
      assert position < 1.0
    end
  end

  describe "reschedules publish" do
    test "schedules next publish after handling" do
      state = init_sensor()

      {:noreply, _state} = Sensor.handle_info(:publish, state)

      assert_receive :publish, 100
    end
  end
end
