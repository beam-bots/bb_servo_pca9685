<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Position Feedback

RC servos don't provide position feedback, but `BB.Servo.PCA9685.Sensor` can
estimate position based on commanded targets and timing. This tutorial shows you
how to set up and use position feedback.

## Prerequisites

- Completed [Basic Usage](2-basic-usage.md)
- A working servo joint with PCA9685 controller

## Adding a Feedback Sensor

Add the sensor to your joint definition:

```elixir
defmodule MyRobot do
  use BB.Robot

  robot do
    controller :pca9685, {BB.Servo.PCA9685.Controller,
      bus: "i2c-1",
      address: 0x40
    }

    link :base do
      joint :pan, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}
        sensor :feedback, {BB.Servo.PCA9685.Sensor, actuator: :servo}

        link :head do
        end
      end
    end
  end
end
```

The sensor requires the `actuator` option to know which actuator to subscribe to.

## How Position Feedback Works

Since RC servos don't report their actual position, the sensor estimates it:

1. **Actuator publishes commands** - When you call `set_position`, the actuator
   publishes a `PositionCommand` message with the target angle and expected
   arrival time

2. **Sensor subscribes** - The sensor receives these messages and tracks the
   target position and expected arrival time

3. **Position interpolation** - During movement, the sensor interpolates between
   the previous position and target based on elapsed time

4. **JointState publishing** - The sensor publishes `JointState` messages with
   the estimated position

### Timeline Example

```
Time 0ms:    Command sent (target: 45°, arrival: 500ms)
             Sensor receives command, starts interpolating

Time 100ms:  Estimated position: 9° (20% of the way)
Time 250ms:  Estimated position: 22.5° (50% of the way)
Time 500ms:  Estimated position: 45° (arrived)

Time 600ms:  Position stable at 45° (no change published)
Time 5000ms: Sync publish at 45° (max_silence reached)
```

## Sensor Options

```elixir
sensor :feedback, {BB.Servo.PCA9685.Sensor,
  actuator: :servo,           # Required: actuator to subscribe to
  publish_rate: ~u(50 hertz), # Optional: how often to check for changes (default: 50 Hz)
  max_silence: ~u(5 second)   # Optional: max time between publishes (default: 5s)
}
```

### publish_rate

How often the sensor checks for position changes. Higher rates give smoother
feedback during movement but use more resources.

- `~u(50 hertz)` - Default, good for most applications
- `~u(100 hertz)` - Smoother feedback for fast movements
- `~u(10 hertz)` - Lower resource usage for slow-moving joints

### max_silence

Even when the position hasn't changed, the sensor publishes periodically to keep
subscribers in sync. This handles:

- Late subscribers that missed earlier updates
- Recovery from dropped messages
- Monitoring systems that expect regular updates

Set to a higher value if you want less traffic when idle.

## Subscribing to Position Updates

Subscribe to the sensor's JointState messages:

```elixir
# Subscribe to the sensor topic
BB.subscribe(MyRobot, [:sensor, :pan, :feedback])

# In your GenServer or process
def handle_info(%BB.Message{payload: %BB.Message.Sensor.JointState{} = joint_state}, state) do
  [position] = joint_state.positions
  IO.puts("Pan position: #{position} radians")
  {:noreply, state}
end
```

## Reading Current Position

You can also query the robot's state directly:

```elixir
# Get current joint positions
state = BB.Robot.State.get(MyRobot)
pan_position = BB.Robot.State.get_joint_position(state, :pan)
```

## Example: Position Logger

Here's a complete example that logs position changes:

```elixir
defmodule PositionLogger do
  use GenServer

  def start_link(robot) do
    GenServer.start_link(__MODULE__, robot, name: __MODULE__)
  end

  def init(robot) do
    BB.subscribe(robot, [:sensor, :pan, :feedback])
    {:ok, %{robot: robot}}
  end

  def handle_info(%BB.Message{payload: %BB.Message.Sensor.JointState{} = js}, state) do
    [position] = js.positions
    degrees = position * 180 / :math.pi()
    IO.puts("[#{DateTime.utc_now()}] Pan: #{Float.round(degrees, 1)}°")
    {:noreply, state}
  end
end
```

Start the logger:

```elixir
{:ok, _} = MyRobot.start_link()
{:ok, _} = PositionLogger.start_link(MyRobot)

# Move the servo and watch the logs
BB.Actuator.set_position(MyRobot, :servo, 0.785)
# Output:
# [2025-01-15 10:30:00.000000Z] Pan: 9.0°
# [2025-01-15 10:30:00.020000Z] Pan: 18.0°
# [2025-01-15 10:30:00.040000Z] Pan: 27.0°
# ... (interpolated positions during movement)
# [2025-01-15 10:30:00.500000Z] Pan: 45.0°
```

## Example: Wait for Movement Complete

Wait for the servo to reach its target position:

```elixir
defmodule ServoHelper do
  def move_and_wait(robot, actuator, target, timeout \\ 5000) do
    # Subscribe to sensor updates
    BB.subscribe(robot, [:sensor, :pan, :feedback])

    # Send the command
    BB.Actuator.set_position(robot, actuator, target)

    # Wait for position to match target
    wait_for_position(target, timeout)
  end

  defp wait_for_position(target, timeout) do
    receive do
      %BB.Message{payload: %BB.Message.Sensor.JointState{positions: [position]}}
      when abs(position - target) < 0.01 ->
        :ok
    after
      timeout -> {:error, :timeout}
    end
  end
end

# Usage
:ok = ServoHelper.move_and_wait(MyRobot, :servo, 0.785)
IO.puts("Servo reached target!")
```

## Example: Multi-Joint Position Monitor

Monitor all servos in a hexapod leg:

```elixir
defmodule LegMonitor do
  use GenServer

  @joints [:leg1_coxa, :leg1_femur, :leg1_tibia]

  def start_link(robot) do
    GenServer.start_link(__MODULE__, robot, name: __MODULE__)
  end

  def init(robot) do
    # Subscribe to all leg joint sensors
    for joint <- @joints do
      BB.subscribe(robot, [:sensor, joint, :feedback])
    end

    {:ok, %{robot: robot, positions: %{}}}
  end

  def handle_info(%BB.Message{frame_id: joint, payload: %BB.Message.Sensor.JointState{} = js}, state) do
    [position] = js.positions
    positions = Map.put(state.positions, joint, position)

    # Print all positions when we have updates for all joints
    if map_size(positions) == length(@joints) do
      print_leg_position(positions)
    end

    {:noreply, %{state | positions: positions}}
  end

  defp print_leg_position(positions) do
    coxa = rad_to_deg(positions[:leg1_coxa])
    femur = rad_to_deg(positions[:leg1_femur])
    tibia = rad_to_deg(positions[:leg1_tibia])

    IO.puts("Leg position: coxa=#{coxa}° femur=#{femur}° tibia=#{tibia}°")
  end

  defp rad_to_deg(rad), do: Float.round(rad * 180 / :math.pi(), 1)
end
```

## Limitations

Remember that this is **estimated** position, not actual position:

- The servo might not reach the target (blocked, insufficient torque)
- The servo might overshoot or oscillate
- The timing might not match the real servo exactly

For applications requiring precise position feedback, consider:

- Adding a physical sensor (potentiometer, encoder) to your servo
- Using a servo with built-in feedback (smart servos)
- Using stepper motors with encoders

## Comparison with BB.Servo.Pigpio

The sensor works identically to `BB.Servo.Pigpio.Sensor`. If you're migrating
from pigpio to PCA9685, simply:

1. Replace `BB.Servo.Pigpio.Sensor` with `BB.Servo.PCA9685.Sensor`
2. Ensure the `actuator` option points to a PCA9685 actuator

The message types and behaviour are the same.

## Next Steps

You now have a complete servo setup with position feedback. Explore the BB
framework documentation to learn about:

- Trajectory planning for smooth multi-joint movements
- Inverse kinematics for end-effector positioning
- State machines for complex behaviours
