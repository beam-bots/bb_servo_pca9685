# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685.Actuator do
  @moduledoc """
  An actuator that uses a PCA9685 controller to drive a servo.

  Configuration is derived from the joint's `motor_profile` injected by
  `BB.Actuator.Server`:

  - Position limits from `motor_profile.motor_lower` / `motor_upper`
  - Velocity limit from `motor_profile.motor_velocity_limit`
  - PWM range maps linearly to the motor's position range
    (`motor_lower → min_pulse`, `motor_upper → max_pulse`)

  When a position command is received, the actuator:
  1. Clamps the position to motor limits
  2. Converts to PWM pulse width
  3. Sends PWM command to the PCA9685 controller
  4. Publishes a `BB.Message.Actuator.BeginMotion` via
     `BB.Actuator.publish_begin_motion/3` (which handles the
     motor → joint-space conversion)

  ## Example DSL Usage

      controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}

      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
      end
  """
  use BB.Actuator,
    options_schema: [
      channel: [
        type: {:in, 0..15},
        doc: "The PCA9685 channel (0-15)",
        required: true
      ],
      controller: [
        type: :atom,
        doc: "Name of the PCA9685 controller in the robot's registry",
        required: true
      ],
      min_pulse: [
        type: :pos_integer,
        doc: "The minimum PWM pulse that can be sent to the servo (µs)",
        default: 500
      ],
      max_pulse: [
        type: :pos_integer,
        doc: "The maximum PWM pulse that can be sent to the servo (µs)",
        default: 2500
      ]
    ]

  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess

  @doc """
  Disable the servo by setting pulse width to 0.

  Called by `BB.Safety.Controller` when the robot is disarmed or crashes.
  It needs no actuator state - the robot module, controller name, and channel
  come from the opts provided during registration - but the write is still routed
  through the controller, so it requires the controller process to be alive. If the
  controller is unreachable the call exits and the disarm fails, driving the robot
  into the `:error` state.
  """
  @impl BB.Actuator
  def disarm(opts) do
    robot = Keyword.fetch!(opts, :robot)
    controller = Keyword.fetch!(opts, :controller)
    channel = Keyword.fetch!(opts, :channel)

    case BBProcess.call(robot, controller, {:pulse_width, channel, 0}) do
      :ok -> :ok
      {:ok, _} -> :ok
      {:error, reason} -> {:error, reason}
    end
  end

  @impl BB.Actuator
  def init(opts) do
    with {:ok, state} <- build_state(opts),
         :ok <- set_initial_position(state) do
      BB.Safety.register(__MODULE__,
        robot: state.bb.robot,
        path: state.bb.path,
        opts: [robot: state.bb.robot, controller: state.controller, channel: state.channel]
      )

      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  @impl BB.Actuator
  def handle_options(new_opts, state) do
    motor_profile = Keyword.fetch!(new_opts, :motor_profile)
    motor_range = motor_profile.motor_upper - motor_profile.motor_lower

    {:ok,
     %{
       state
       | motor_profile: motor_profile,
         motor_range: motor_range,
         current_motor_angle: clamp_motor_angle(state.current_motor_angle, motor_profile)
     }}
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    motor_profile = opts.motor_profile

    min_pulse = Map.get(opts, :min_pulse, 500)
    max_pulse = Map.get(opts, :max_pulse, 2500)

    with :ok <- validate_motor_profile(motor_profile, joint_name) do
      motor_range = motor_profile.motor_upper - motor_profile.motor_lower
      initial_pulse = (max_pulse + min_pulse) / 2

      state = %{
        bb: opts.bb,
        channel: opts.channel,
        controller: opts.controller,
        min_pulse: min_pulse,
        max_pulse: max_pulse,
        motor_profile: motor_profile,
        motor_range: motor_range,
        pulse_range: max_pulse - min_pulse,
        current_pulse: initial_pulse,
        current_motor_angle: motor_profile.motor_initial_position,
        name: name,
        joint_name: joint_name
      }

      {:ok, state}
    end
  end

  defp validate_motor_profile(%{motor_lower: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :lower,
       value: nil,
       message: "Joint must have a lower limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_upper: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :upper,
       value: nil,
       message: "Joint must have an upper limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_velocity_limit: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :velocity,
       value: nil,
       message: "Joint must have a velocity limit defined for servo control"
     }}
  end

  defp validate_motor_profile(_profile, _joint_name), do: :ok

  defp set_initial_position(state) do
    pulse = round(state.current_pulse)

    case BBProcess.call(state.bb.robot, state.controller, {:pulse_width, state.channel, pulse}) do
      :ok -> :ok
      {:ok, _} -> :ok
      {:error, _} = error -> error
    end
  end

  @impl BB.Actuator
  def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
    {:noreply, state} = do_set_position(cmd.position, cmd.command_id, state)
    {:noreply, state}
  end

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    do_set_position(cmd.position, cmd.command_id, state)
  end

  @impl BB.Actuator
  def handle_call({:command, %Message{payload: %Command.Position{} = cmd}}, _from, state) do
    {:noreply, new_state} = do_set_position(cmd.position, cmd.command_id, state)
    {:reply, {:ok, :accepted}, new_state}
  end

  defp do_set_position(motor_angle, command_id, state) when is_integer(motor_angle),
    do: do_set_position(motor_angle * 1.0, command_id, state)

  defp do_set_position(motor_angle, command_id, state) do
    clamped_motor_angle = clamp_motor_angle(motor_angle, state.motor_profile)
    new_pulse = motor_angle_to_pulse(clamped_motor_angle, state)

    case BBProcess.call(
           state.bb.robot,
           state.controller,
           {:pulse_width, state.channel, new_pulse}
         ) do
      result when result == :ok or (is_tuple(result) and elem(result, 0) == :ok) ->
        travel_distance = abs(state.current_motor_angle - clamped_motor_angle)

        travel_time_ms =
          round(travel_distance / state.motor_profile.motor_velocity_limit * 1000)

        expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

        message_opts =
          [
            initial_position: state.current_motor_angle,
            target_position: clamped_motor_angle,
            expected_arrival: expected_arrival,
            command_type: :position
          ]
          |> maybe_add_opt(:command_id, command_id)

        BB.Actuator.publish_begin_motion(state.bb.robot, state.bb.path, message_opts)

        {:noreply, %{state | current_pulse: new_pulse, current_motor_angle: clamped_motor_angle}}

      {:error, _reason} ->
        {:noreply, state}
    end
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp clamp_motor_angle(motor_angle, %{motor_lower: lower, motor_upper: upper}) do
    motor_angle
    |> max(lower)
    |> min(upper)
  end

  # Motor-space angle maps linearly to PWM pulse width: motor_lower → min_pulse,
  # motor_upper → max_pulse.
  defp motor_angle_to_pulse(motor_angle, state) do
    normalised = (motor_angle - state.motor_profile.motor_lower) / state.motor_range
    round(state.min_pulse + normalised * state.pulse_range)
  end
end
