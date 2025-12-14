# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685.ActuatorTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Servo.PCA9685.Actuator

  @joint_name :test_joint
  @actuator_name :test_servo
  @controller_name :test_pca9685

  defp position_command(position, opts \\ []) do
    message_opts =
      [position: position * 1.0]
      |> maybe_add_opt(:command_id, opts[:command_id])

    {:command, Message.new!(Command.Position, @joint_name, message_opts)}
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp default_bb_context do
    %{robot: TestRobot, path: [@joint_name, @actuator_name]}
  end

  defp joint_with_limits(lower, upper, velocity) do
    %{
      type: :revolute,
      limits: %{
        lower: lower,
        upper: upper,
        velocity: velocity,
        effort: 1.0
      }
    }
  end

  defp stub_controller_success do
    stub(BB.Process, :call, fn _robot, _name, _msg, _timeout -> :ok end)
    stub(BB.Process, :call, fn _robot, _name, _msg -> :ok end)
    stub(BB.Safety, :register, fn _module, _opts -> :ok end)
  end

  describe "init/1" do
    test "succeeds with valid joint limits" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-0.5, 0.5, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      assert {:ok, state} = Actuator.init(opts)

      assert state.lower_limit == -0.5
      assert state.upper_limit == 0.5
      assert state.velocity_limit == 1.0
      assert state.range == 1.0
      assert state.center_angle == 0.0
    end

    test "stores channel and controller name" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-0.5, 0.5, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), channel: 5, controller: @controller_name]
      assert {:ok, state} = Actuator.init(opts)

      assert state.channel == 5
      assert state.controller == @controller_name
    end

    test "fails when joint not found" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name -> nil end)

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      assert {:stop, {:joint_not_found, @joint_name}} = Actuator.init(opts)
    end

    test "fails for continuous joints" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        %{type: :continuous, limits: %{lower: nil, upper: nil, velocity: 1.0, effort: 1.0}}
      end)

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      assert {:stop, {:unsupported_joint_type, :continuous, @joint_name}} = Actuator.init(opts)
    end

    test "fails when limits not defined" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        %{type: :revolute, limits: nil}
      end)

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      assert {:stop, {:no_limits_defined, @joint_name}} = Actuator.init(opts)
    end

    test "fails when lower limit missing" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        %{type: :revolute, limits: %{lower: nil, upper: 0.5, velocity: 1.0, effort: 1.0}}
      end)

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      assert {:stop, {:missing_limit, :lower, @joint_name}} = Actuator.init(opts)
    end

    test "fails when upper limit missing" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        %{type: :revolute, limits: %{lower: -0.5, upper: nil, velocity: 1.0, effort: 1.0}}
      end)

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      assert {:stop, {:missing_limit, :upper, @joint_name}} = Actuator.init(opts)
    end

    test "initialises servo at center position" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      assert {:ok, state} = Actuator.init(opts)

      assert state.current_angle == 0.0
      assert state.current_pulse == 1500.0
    end

    test "sends initial pulse to controller" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 3, 1500} ->
        send(test_pid, :initial_pulse_sent)
        :ok
      end)

      opts = [bb: default_bb_context(), channel: 3, controller: @controller_name]
      {:ok, _state} = Actuator.init(opts)

      assert_receive :initial_pulse_sent
    end

    test "initialises asymmetric joint at correct center" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(0.0, 2.0, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      assert {:ok, state} = Actuator.init(opts)

      assert state.center_angle == 1.0
      assert state.current_angle == 1.0
    end
  end

  describe "angle_to_pulse conversion" do
    setup do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()
      stub(BB, :publish, fn _robot, _path, _msg -> :ok end)

      opts = [
        bb: default_bb_context(),
        channel: 0,
        controller: @controller_name,
        min_pulse: 500,
        max_pulse: 2500
      ]

      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "lower limit maps to min_pulse", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 0, pulse} ->
        send(test_pid, {:pulse, pulse})
        :ok
      end)

      Actuator.handle_cast(position_command(-1.0), state)

      assert_receive {:pulse, 500}
    end

    test "upper limit maps to max_pulse", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 0, pulse} ->
        send(test_pid, {:pulse, pulse})
        :ok
      end)

      Actuator.handle_cast(position_command(1.0), state)

      assert_receive {:pulse, 2500}
    end

    test "center maps to mid_pulse", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 0, pulse} ->
        send(test_pid, {:pulse, pulse})
        :ok
      end)

      Actuator.handle_cast(position_command(0.0), state)

      assert_receive {:pulse, 1500}
    end
  end

  describe "reverse mode" do
    setup do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()
      stub(BB, :publish, fn _robot, _path, _msg -> :ok end)

      opts = [
        bb: default_bb_context(),
        channel: 0,
        controller: @controller_name,
        min_pulse: 500,
        max_pulse: 2500,
        reverse?: true
      ]

      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "lower limit maps to max_pulse when reversed", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 0, pulse} ->
        send(test_pid, {:pulse, pulse})
        :ok
      end)

      Actuator.handle_cast(position_command(-1.0), state)

      assert_receive {:pulse, 2500}
    end

    test "upper limit maps to min_pulse when reversed", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 0, pulse} ->
        send(test_pid, {:pulse, pulse})
        :ok
      end)

      Actuator.handle_cast(position_command(1.0), state)

      assert_receive {:pulse, 500}
    end
  end

  describe "position clamping" do
    setup do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()
      stub(BB, :publish, fn _robot, _path, _msg -> :ok end)

      opts = [
        bb: default_bb_context(),
        channel: 0,
        controller: @controller_name,
        min_pulse: 500,
        max_pulse: 2500
      ]

      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "clamps position below lower limit", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 0, pulse} ->
        send(test_pid, {:pulse, pulse})
        :ok
      end)

      Actuator.handle_cast(position_command(-5.0), state)

      assert_receive {:pulse, 500}
    end

    test "clamps position above upper limit", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 0, pulse} ->
        send(test_pid, {:pulse, pulse})
        :ok
      end)

      Actuator.handle_cast(position_command(5.0), state)

      assert_receive {:pulse, 2500}
    end
  end

  describe "position_commanded publishing" do
    setup do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "publishes position_commanded message", %{state: state} do
      test_pid = self()

      expect(BB, :publish, fn robot, path, message ->
        send(test_pid, {:published, robot, path, message})
        :ok
      end)

      Actuator.handle_cast(position_command(0.5), state)

      assert_receive {:published, TestRobot, [:actuator, @joint_name, @actuator_name], message}

      assert %BB.Message{payload: %BB.Message.Actuator.BeginMotion{} = cmd} = message
      assert cmd.initial_position == 0.0
      assert cmd.target_position == 0.5
      assert is_integer(cmd.expected_arrival)
      assert cmd.expected_arrival > System.monotonic_time(:millisecond)
    end

    test "calculates expected arrival based on velocity", %{state: state} do
      test_pid = self()

      expect(BB, :publish, fn _robot, _path, %BB.Message{payload: cmd} ->
        send(test_pid, {:arrival, cmd.expected_arrival})
        :ok
      end)

      before = System.monotonic_time(:millisecond)
      Actuator.handle_cast(position_command(1.0), state)

      assert_receive {:arrival, expected_arrival}

      travel_time_ms = round(1.0 / 1.0 * 1000)
      assert_in_delta expected_arrival, before + travel_time_ms, 50
    end
  end

  describe "integer position handling" do
    setup do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()
      stub(BB, :publish, fn _robot, _path, _msg -> :ok end)

      opts = [bb: default_bb_context(), channel: 0, controller: @controller_name]
      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "accepts integer position and converts to float", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot, @controller_name, {:pulse_width, 0, _pulse} ->
        send(test_pid, :called)
        :ok
      end)

      assert {:noreply, new_state} = Actuator.handle_cast(position_command(0), state)
      assert new_state.current_angle == 0.0

      assert_receive :called
    end
  end
end
