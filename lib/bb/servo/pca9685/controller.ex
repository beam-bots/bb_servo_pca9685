# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685.Controller do
  @moduledoc """
  A controller GenServer that manages a PCA9685 PWM device.

  This controller wraps a `PCA9685.Device` process and provides an interface
  for actuators to set servo pulse widths. Multiple actuators can share a
  single controller, with each actuator controlling a different channel (0-15).

  ## Configuration

  The controller is typically defined in the robot DSL:

      controller :pca9685, {BB.Servo.PCA9685.Controller,
        bus: "i2c-1",
        address: 0x40,
        pwm_freq: 50
      }

  ## Options

  - `:bus` - (required) The I2C bus name, e.g., `"i2c-1"`
  - `:address` - (required) The I2C address of the PCA9685, e.g., `0x40`
  - `:pwm_freq` - PWM frequency in Hz (default: 50, suitable for servos)
  - `:oe_pin` - Optional GPIO pin for output enable control

  ## Safety

  This controller implements the `BB.Safety` behaviour. When the robot is disarmed
  or crashes, the OE pin is pulled high to disable all servo outputs. If no OE pin
  is configured, the disarm callback returns `:ok` (individual actuators handle
  their own channel disarm).
  """
  use GenServer
  @behaviour BB.Safety

  @doc """
  Disable all servo outputs by pulling the OE pin high.

  Called by `BB.Safety.Controller` when the robot is disarmed or crashes.
  If no OE pin is configured, returns :ok (individual actuators handle their channels).

  Note: This callback calls through the Controller GenServer, so it only works
  if the Controller process is still alive. For crash scenarios, individual
  Actuator disarm callbacks provide per-channel safety.
  """
  @impl BB.Safety
  def disarm(opts) do
    name = Keyword.fetch!(opts, :name)

    try do
      case GenServer.call(name, :output_disable) do
        :ok -> :ok
        {:error, :no_oe_pin_configured} -> :ok
        {:error, reason} -> {:error, reason}
      end
    catch
      :exit, _ -> :ok
    end
  end

  @options Spark.Options.new!(
             bb: [
               type: :map,
               doc: "Automatically set by the robot supervisor",
               required: true
             ],
             bus: [
               type: :string,
               doc: "The I2C bus name (e.g., \"i2c-1\")",
               required: true
             ],
             address: [
               type: :integer,
               doc: "The I2C address of the PCA9685 (e.g., 0x40)",
               required: true
             ],
             pwm_freq: [
               type: :pos_integer,
               doc: "PWM frequency in Hz",
               default: 50
             ],
             oe_pin: [
               type: :pos_integer,
               doc: "GPIO pin for output enable control",
               required: false
             ]
           )

  @impl GenServer
  def init(opts) do
    with {:ok, opts} <- Spark.Options.validate(opts, @options),
         {:ok, device} <- start_device(opts) do
      state = %{
        bb: opts[:bb],
        device: device,
        oe_pin: opts[:oe_pin],
        name: opts[:bb].name
      }

      BB.Safety.register(__MODULE__,
        robot: state.bb.robot,
        path: state.bb.path,
        opts: [name: state.name]
      )

      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  defp start_device(opts) do
    device_opts =
      [
        bus: opts[:bus],
        address: opts[:address],
        pwm_freq: opts[:pwm_freq]
      ]
      |> maybe_add_oe_pin(opts[:oe_pin])

    PCA9685.acquire(device_opts)
  end

  defp maybe_add_oe_pin(opts, nil), do: opts
  defp maybe_add_oe_pin(opts, oe_pin), do: Keyword.put(opts, :oe_pin, oe_pin)

  @impl GenServer
  def handle_call({:pulse_width, channel, microseconds}, _from, state) do
    result = PCA9685.Device.pulse_width(state.device, channel, microseconds)
    {:reply, result, state}
  end

  def handle_call(:output_enable, _from, %{oe_pin: nil} = state) do
    {:reply, {:error, :no_oe_pin_configured}, state}
  end

  def handle_call(:output_enable, _from, state) do
    result = PCA9685.Device.output_enable(state.device)
    {:reply, result, state}
  end

  def handle_call(:output_disable, _from, %{oe_pin: nil} = state) do
    {:reply, {:error, :no_oe_pin_configured}, state}
  end

  def handle_call(:output_disable, _from, state) do
    result = PCA9685.Device.output_disable(state.device)
    {:reply, result, state}
  end

  @impl GenServer
  def terminate(_reason, state) do
    PCA9685.release(state.device)
    :ok
  end
end
