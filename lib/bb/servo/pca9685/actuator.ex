# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685.Actuator do
  @moduledoc """
  An actuator that uses a PCA9685 controller to drive a servo.

  This actuator derives its configuration from the joint constraints defined in the robot:
  - Position limits from `joint.limits.lower` and `joint.limits.upper`
  - Velocity limit from `joint.limits.velocity`
  - PWM range maps linearly to the joint's position range

  When a position command is received, the actuator:
  1. Clamps the position to joint limits
  2. Converts to PWM pulse width
  3. Sends PWM command to the PCA9685 controller
  4. Publishes a `BB.Message.Actuator.BeginMotion` for sensors to consume

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

  alias BB.Actuator, as: ActuatorApi
  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess
  alias BB.Transmission

  @doc """
  Disable the servo by setting pulse width to 0.

  Called by `BB.Safety.Controller` when the robot is disarmed or crashes.
  This function works without GenServer state - it receives the robot module,
  controller name, and channel from the opts provided during registration.
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

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    robot = opts.bb.robot.robot()
    transmission = ActuatorApi.current_transmission()

    min_pulse = Map.get(opts, :min_pulse, 500)
    max_pulse = Map.get(opts, :max_pulse, 2500)

    with {:ok, joint} <- fetch_joint(robot, joint_name),
         {:ok, limits} <- validate_joint_limits(joint, joint_name) do
      {motor_lower, motor_upper} = motor_position_limits(limits, transmission)
      motor_range = motor_upper - motor_lower
      motor_velocity_limit = motor_velocity_limit(limits.velocity, transmission)
      pulse_range = max_pulse - min_pulse

      initial_pulse = (max_pulse + min_pulse) / 2
      current_motor_angle = (motor_lower + motor_upper) / 2

      state = %{
        bb: opts.bb,
        channel: opts.channel,
        controller: opts.controller,
        min_pulse: min_pulse,
        max_pulse: max_pulse,
        motor_lower: motor_lower,
        motor_upper: motor_upper,
        motor_range: motor_range,
        motor_velocity_limit: motor_velocity_limit,
        pulse_range: pulse_range,
        current_pulse: initial_pulse,
        current_motor_angle: current_motor_angle,
        name: name,
        joint_name: joint_name
      }

      {:ok, state}
    end
  end

  defp motor_position_limits(limits, nil), do: {limits.lower, limits.upper}

  defp motor_position_limits(limits, transmission) do
    a = Transmission.apply_position(limits.lower, transmission)
    b = Transmission.apply_position(limits.upper, transmission)
    {min(a, b), max(a, b)}
  end

  defp motor_velocity_limit(velocity, nil), do: velocity

  defp motor_velocity_limit(velocity, transmission) do
    abs(Transmission.apply_rate(velocity, transmission))
  end

  defp fetch_joint(robot, joint_name) do
    case BB.Robot.get_joint(robot, joint_name) do
      nil ->
        {:error,
         %JointConfigError{joint: joint_name, field: nil, message: "Joint not found in robot"}}

      joint ->
        {:ok, joint}
    end
  end

  defp validate_joint_limits(%{type: :continuous}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :type,
       value: :continuous,
       expected: [:revolute, :prismatic],
       message: "Continuous joints require position limits for servo control"
     }}
  end

  defp validate_joint_limits(%{limits: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :limits,
       value: nil,
       message: "Joint must have limits defined for servo control"
     }}
  end

  defp validate_joint_limits(%{limits: %{lower: nil}}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :lower,
       value: nil,
       message: "Joint must have lower limit defined"
     }}
  end

  defp validate_joint_limits(%{limits: %{upper: nil}}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :upper,
       value: nil,
       message: "Joint must have upper limit defined"
     }}
  end

  defp validate_joint_limits(%{limits: limits}, _joint_name) do
    {:ok, limits}
  end

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
    clamped_motor_angle = clamp_motor_angle(motor_angle, state)
    new_pulse = motor_angle_to_pulse(clamped_motor_angle, state)

    case BBProcess.call(
           state.bb.robot,
           state.controller,
           {:pulse_width, state.channel, new_pulse}
         ) do
      result when result == :ok or (is_tuple(result) and elem(result, 0) == :ok) ->
        travel_distance = abs(state.current_motor_angle - clamped_motor_angle)
        travel_time_ms = round(travel_distance / state.motor_velocity_limit * 1000)
        expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

        message_opts =
          [
            initial_position: state.current_motor_angle,
            target_position: clamped_motor_angle,
            expected_arrival: expected_arrival,
            command_type: :position
          ]
          |> maybe_add_opt(:command_id, command_id)

        message = Message.new!(BeginMotion, state.joint_name, message_opts)

        BB.publish(state.bb.robot, [:actuator | state.bb.path], message)

        {:noreply, %{state | current_pulse: new_pulse, current_motor_angle: clamped_motor_angle}}

      {:error, _reason} ->
        {:noreply, state}
    end
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp clamp_motor_angle(motor_angle, state) do
    motor_angle
    |> max(state.motor_lower)
    |> min(state.motor_upper)
  end

  # Motor-space angle maps linearly to PWM pulse width: motor_lower → min_pulse,
  # motor_upper → max_pulse.
  defp motor_angle_to_pulse(motor_angle, state) do
    normalised = (motor_angle - state.motor_lower) / state.motor_range
    round(state.min_pulse + normalised * state.pulse_range)
  end
end
