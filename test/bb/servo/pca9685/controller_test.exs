# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685.ControllerTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Error.Hardware.NoOutputEnablePin
  alias BB.Servo.PCA9685.Controller

  @controller_name :test_pca9685

  defp default_bb_context do
    %{robot: TestRobot, path: [@controller_name], name: @controller_name}
  end

  defp default_opts do
    [
      bb: default_bb_context(),
      bus: "i2c-1",
      address: 0x40
    ]
  end

  defp stub_pca9685_success do
    stub(PCA9685, :acquire, fn _opts -> {:ok, self()} end)
    stub(PCA9685, :release, fn _pid -> :ok end)
    stub(PCA9685.Device, :pulse_width, fn _pid, _channel, _us -> :ok end)
    stub(PCA9685.Device, :output_enable, fn _pid -> :ok end)
    stub(PCA9685.Device, :output_disable, fn _pid -> :ok end)
    stub(BB.Safety, :register, fn _module, _opts -> :ok end)
  end

  describe "init/1" do
    test "succeeds with valid options" do
      stub_pca9685_success()

      assert {:ok, state} = Controller.init(default_opts())

      assert state.bb == default_bb_context()
      assert is_pid(state.device)
      assert state.oe_pin == nil
    end

    test "passes correct options to PCA9685.acquire" do
      test_pid = self()

      expect(PCA9685, :acquire, fn opts ->
        send(test_pid, {:acquire_opts, opts})
        {:ok, self()}
      end)

      Controller.init(default_opts())

      assert_receive {:acquire_opts, opts}
      assert opts[:bus] == "i2c-1"
      assert opts[:address] == 0x40
      assert opts[:pwm_freq] == 50
    end

    test "uses custom pwm_freq when provided" do
      test_pid = self()

      expect(PCA9685, :acquire, fn opts ->
        send(test_pid, {:acquire_opts, opts})
        {:ok, self()}
      end)

      opts = Keyword.put(default_opts(), :pwm_freq, 60)
      Controller.init(opts)

      assert_receive {:acquire_opts, acquire_opts}
      assert acquire_opts[:pwm_freq] == 60
    end

    test "includes oe_pin when provided" do
      test_pid = self()

      expect(PCA9685, :acquire, fn opts ->
        send(test_pid, {:acquire_opts, opts})
        {:ok, self()}
      end)

      opts = Keyword.put(default_opts(), :oe_pin, 25)
      {:ok, state} = Controller.init(opts)

      assert_receive {:acquire_opts, acquire_opts}
      assert acquire_opts[:oe_pin] == 25
      assert state.oe_pin == 25
    end

    test "fails when PCA9685.acquire fails" do
      stub(PCA9685, :acquire, fn _opts -> {:error, :device_not_found} end)

      assert {:stop, :device_not_found} = Controller.init(default_opts())
    end

    test "fails with missing required options" do
      stub_pca9685_success()

      assert_raise KeyError, fn ->
        Controller.init(bb: default_bb_context())
      end
    end
  end

  describe "handle_call {:pulse_width, channel, microseconds}" do
    setup do
      stub_pca9685_success()
      {:ok, state} = Controller.init(default_opts())
      {:ok, state: state}
    end

    test "delegates to PCA9685.Device.pulse_width", %{state: state} do
      test_pid = self()

      expect(PCA9685.Device, :pulse_width, fn pid, channel, us ->
        send(test_pid, {:pulse_width, pid, channel, us})
        :ok
      end)

      Controller.handle_call({:pulse_width, 5, 1500}, self(), state)

      assert_receive {:pulse_width, _pid, 5, 1500}
    end

    test "returns the result from PCA9685.Device", %{state: state} do
      stub(PCA9685.Device, :pulse_width, fn _pid, _ch, _us -> :ok end)

      assert {:reply, :ok, _state} =
               Controller.handle_call({:pulse_width, 0, 1000}, self(), state)
    end
  end

  describe "handle_call :output_enable" do
    test "returns error when oe_pin not configured" do
      stub_pca9685_success()
      {:ok, state} = Controller.init(default_opts())

      assert {:reply, {:error, %NoOutputEnablePin{controller: @controller_name}}, _state} =
               Controller.handle_call(:output_enable, self(), state)
    end

    test "delegates to PCA9685.Device when oe_pin configured" do
      test_pid = self()

      stub(PCA9685, :acquire, fn _opts -> {:ok, self()} end)

      expect(PCA9685.Device, :output_enable, fn _pid ->
        send(test_pid, :output_enable_called)
        :ok
      end)

      opts = Keyword.put(default_opts(), :oe_pin, 25)
      {:ok, state} = Controller.init(opts)

      Controller.handle_call(:output_enable, self(), state)

      assert_receive :output_enable_called
    end
  end

  describe "handle_call :output_disable" do
    test "returns error when oe_pin not configured" do
      stub_pca9685_success()
      {:ok, state} = Controller.init(default_opts())

      assert {:reply, {:error, %NoOutputEnablePin{controller: @controller_name}}, _state} =
               Controller.handle_call(:output_disable, self(), state)
    end

    test "delegates to PCA9685.Device when oe_pin configured" do
      test_pid = self()

      stub(PCA9685, :acquire, fn _opts -> {:ok, self()} end)

      expect(PCA9685.Device, :output_disable, fn _pid ->
        send(test_pid, :output_disable_called)
        :ok
      end)

      opts = Keyword.put(default_opts(), :oe_pin, 25)
      {:ok, state} = Controller.init(opts)

      Controller.handle_call(:output_disable, self(), state)

      assert_receive :output_disable_called
    end
  end

  describe "terminate/2" do
    test "releases the PCA9685 device" do
      test_pid = self()

      stub(PCA9685, :acquire, fn _opts -> {:ok, self()} end)

      expect(PCA9685, :release, fn pid ->
        send(test_pid, {:release, pid})
        :ok
      end)

      {:ok, state} = Controller.init(default_opts())
      Controller.terminate(:normal, state)

      assert_receive {:release, _pid}
    end
  end
end
