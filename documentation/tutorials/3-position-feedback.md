<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Position Feedback

RC servos don't provide position feedback, but core's
`BB.Sensor.OpenLoopPositionEstimator` can estimate position based on commanded
targets and timing. This tutorial shows you how to set up and use estimated
position feedback.

## Prerequisites

- Completed [Basic Usage](2-basic-usage.md)
- A working servo joint with PCA9685 controller

## Adding a Feedback Sensor

Add the sensor to your joint definition:

```elixir
defmodule MyRobot do
  use BB

  controllers do
    controller :pca9685, {BB.Servo.PCA9685.Controller, bus: "i2c-1", address: 0x40}
  end

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end
  end

  topology do
    link :base do
      joint :pan do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(1 newton_meter),
              velocity: ~u(60 degree_per_second)

        actuator :pan_servo, {BB.Servo.PCA9685.Actuator, channel: 0, controller: :pca9685}
        sensor :pan_feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :pan_servo}

        link :head
      end
    end
  end
end
```

The sensor requires the `actuator` option to know which actuator to subscribe to.

> **Under simulation only**, BB adds an estimator for any actuator that doesn't
> already have one, named `:<actuator>_position_estimator`. On real hardware you
> declare it yourself, as above.

## How Position Feedback Works

Since RC servos don't report their actual position, the sensor estimates it:

1. **Actuator publishes motion** - When you command a position, the actuator
   publishes a `BB.Message.Actuator.BeginMotion` message with the initial and
   target positions and expected arrival time

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
sensor :pan_feedback, {BB.Sensor.OpenLoopPositionEstimator,
  actuator: :pan_servo,       # Required: actuator to subscribe to
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
BB.subscribe(MyRobot, [:sensor, :base, :pan, :pan_feedback])

# In your GenServer or process
def handle_info({:bb, _path, %BB.Message{payload: %BB.Message.Sensor.JointState{} = joint_state}}, state) do
  [position] = joint_state.positions
  IO.puts("Pan position: #{position} radians")
  {:noreply, state}
end
```

Two things to get right here.

**The topic is the component's full path through the topology** — link, then
joint, then sensor. `[:sensor, :pan, :pan_feedback]` looks plausible but matches
nothing, because it omits the `:base` link. Paths are hierarchical, so you can
subscribe to any prefix to get a whole subtree: `[:sensor, :base]` catches every
sensor below the base link, and `[:sensor]` catches all of them.

**Messages arrive wrapped in a three-element tuple**, `{:bb, path, %BB.Message{}}`,
not as a bare `%BB.Message{}`. The `path` tells you which component published,
which matters once you're subscribed to a subtree rather than one sensor.

## Reading Current Position

You can also ask the runtime, which keeps a live map of joint positions built
from these same messages:

```elixir
iex> BB.Robot.Runtime.positions(MyRobot)
%{pan: 0.7853981633974483}
```

`BB.Robot.Runtime.velocities/1` returns the same shape for velocities. Both read
straight from ETS, so they're cheap to call from anywhere and don't block on the
sensor.

## Example: Position Logger

Here's a complete example that logs position changes:

```elixir
defmodule PositionLogger do
  use GenServer

  def start_link(robot) do
    GenServer.start_link(__MODULE__, robot, name: __MODULE__)
  end

  def init(robot) do
    BB.subscribe(robot, [:sensor, :base, :pan, :pan_feedback])
    {:ok, %{robot: robot}}
  end

  def handle_info({:bb, _path, %BB.Message{payload: %BB.Message.Sensor.JointState{} = js}}, state) do
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

{:ok, command} = MyRobot.arm()
{:ok, :armed, _} = BB.Command.await(command)

# Move the servo and watch the logs
BB.Actuator.set_position!(MyRobot, :pan_servo, 0.785)
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
  def move_and_wait(robot, sensor_path, actuator, target, timeout \\ 5000) do
    BB.subscribe(robot, sensor_path)

    BB.Actuator.set_position!(robot, actuator, target)

    wait_for_position(target, timeout)
  end

  defp wait_for_position(target, timeout) do
    receive do
      {:bb, _path, %BB.Message{payload: %BB.Message.Sensor.JointState{positions: [position]}}} ->
        if abs(position - target) < 0.01 do
          :ok
        else
          wait_for_position(target, timeout)
        end
    after
      timeout -> {:error, :timeout}
    end
  end
end

# Usage
:ok = ServoHelper.move_and_wait(MyRobot, [:sensor, :base, :pan, :pan_feedback], :pan_servo, 0.785)
IO.puts("Servo reached target!")
```

The proximity test belongs in the body rather than in a guard on the `receive`
clause. A guard would leave every interpolated position sitting unmatched in the
mailbox, to be re-scanned on each subsequent `receive`.

> The timeout restarts on each message rather than bounding total wait. Track a
> deadline with `System.monotonic_time/1` if you need a hard ceiling.

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
    # One subscription covers every sensor below the body link
    BB.subscribe(robot, [:sensor, :body])

    {:ok, %{robot: robot, positions: %{}}}
  end

  def handle_info({:bb, _path, %BB.Message{payload: %BB.Message.Sensor.JointState{} = js}}, state) do
    positions = Enum.into(Enum.zip(js.names, js.positions), state.positions)

    # Print all positions when we have updates for all joints
    if Enum.all?(@joints, &Map.has_key?(positions, &1)) do
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

**Joint identity comes from `JointState.names`, not from the message's
`frame_id`.** The frame id names the coordinate frame the reading was taken in —
conventionally the publishing sensor, so `:leg1_coxa_feedback` here, not the
joint. `names` and `positions` are parallel lists, which is why zipping them into
the accumulator handles a sensor reporting several joints at once.

## Limitations

Remember that this is **estimated** position, not actual position:

- The servo might not reach the target (blocked, insufficient torque)
- The servo might overshoot or oscillate
- The timing might not match the real servo exactly

For applications requiring precise position feedback, consider:

- Adding a physical sensor (potentiometer, encoder) to your servo
- Using a servo with built-in feedback (smart servos)
- Using stepper motors with encoders

## Using the estimator with other servo drivers

`BB.Sensor.OpenLoopPositionEstimator` is a core BB sensor rather than a
driver-specific module. When changing servo drivers, keep the estimator and
make sure its `actuator` option names the replacement actuator. Any compatible
actuator that publishes `BB.Message.Actuator.BeginMotion` can use it.

## Next Steps

You now have a complete servo setup with position feedback. Explore the BB
framework documentation to learn about:

- Trajectory planning for smooth multi-joint movements
- Inverse kinematics for end-effector positioning
- State machines for complex behaviours
