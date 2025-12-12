# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685.Sensor do
  @moduledoc """
  A sensor GenServer that subscribes to servo actuator position commands
  and publishes JointState messages at a controlled rate.

  This sensor receives `BB.Servo.PCA9685.Message.PositionCommand` messages from a
  `BB.Servo.PCA9685.Actuator` and publishes JointState feedback.

  ## Example DSL Usage

      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}
        sensor :feedback, {BB.Servo.PCA9685.Sensor, actuator: :servo}
      end
  """
  use GenServer
  import BB.Unit
  import BB.Unit.Option

  alias BB.Cldr.Unit, as: CldrUnit
  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Robot.Units
  alias BB.Servo.PCA9685.Message.PositionCommand

  @options Spark.Options.new!(
             bb: [
               type: :map,
               doc: "Automatically set by the robot supervisor",
               required: true
             ],
             actuator: [
               type: :atom,
               doc: "Name of the actuator to subscribe to",
               required: true
             ],
             publish_rate: [
               type: unit_type(compatible: :hertz),
               doc: "Rate at which to check for position changes",
               default: ~u(50 hertz)
             ],
             max_silence: [
               type: unit_type(compatible: :second),
               doc: "Maximum time between publishes even if position unchanged (for sync)",
               default: ~u(5 second)
             ]
           )

  @impl GenServer
  def init(opts) do
    with {:ok, opts} <- Spark.Options.validate(opts, @options),
         {:ok, state} <- build_state(opts) do
      BB.subscribe(state.bb.robot, [:actuator | state.actuator_path])
      schedule_publish(state)
      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)

    publish_interval_ms =
      opts.publish_rate
      |> CldrUnit.convert!(:hertz)
      |> Units.extract_float()
      |> then(&round(1000 / &1))

    max_silence_ms =
      opts.max_silence
      |> CldrUnit.convert!(:second)
      |> Units.extract_float()
      |> then(&round(&1 * 1000))

    actuator_path = build_actuator_path(opts.bb.path, opts.actuator)

    state = %{
      bb: opts.bb,
      actuator: opts.actuator,
      actuator_path: actuator_path,
      publish_interval_ms: publish_interval_ms,
      max_silence_ms: max_silence_ms,
      name: name,
      joint_name: joint_name,
      target: nil,
      expected_arrival: nil,
      previous_position: nil,
      command_time: nil,
      last_published: nil,
      last_publish_time: nil
    }

    {:ok, state}
  end

  defp build_actuator_path(sensor_path, actuator_name) do
    [_sensor_name, joint_name | rest] = Enum.reverse(sensor_path)
    Enum.reverse([actuator_name, joint_name | rest])
  end

  @impl GenServer
  def handle_info(%Message{payload: %PositionCommand{} = cmd}, state) do
    new_state = %{
      state
      | target: cmd.target,
        expected_arrival: cmd.expected_arrival,
        previous_position: state.target || cmd.target,
        command_time: System.monotonic_time(:millisecond)
    }

    {:noreply, new_state}
  end

  def handle_info(:publish, state) do
    state = maybe_publish(state)
    schedule_publish(state)
    {:noreply, state}
  end

  defp maybe_publish(%{target: nil} = state), do: state

  defp maybe_publish(state) do
    position = estimate_position(state)
    now = System.monotonic_time(:millisecond)

    position_changed = position != state.last_published
    silence_exceeded = silence_exceeded?(state, now)

    if position_changed or silence_exceeded do
      message =
        Message.new!(JointState, state.name, names: [state.joint_name], positions: [position])

      BB.publish(state.bb.robot, [:sensor | state.bb.path], message)
      %{state | last_published: position, last_publish_time: now}
    else
      state
    end
  end

  defp silence_exceeded?(%{last_publish_time: nil}, _now), do: false

  defp silence_exceeded?(state, now) do
    now - state.last_publish_time >= state.max_silence_ms
  end

  defp estimate_position(%{target: nil} = _state), do: nil

  defp estimate_position(state) do
    now = System.monotonic_time(:millisecond)

    if now >= state.expected_arrival do
      state.target
    else
      interpolate_position(state, now)
    end
  end

  defp interpolate_position(state, now) do
    total_duration = state.expected_arrival - state.command_time

    if total_duration <= 0 do
      state.target
    else
      elapsed = now - state.command_time
      progress = elapsed / total_duration
      state.previous_position + progress * (state.target - state.previous_position)
    end
  end

  defp schedule_publish(state) do
    Process.send_after(self(), :publish, state.publish_interval_ms)
  end
end
