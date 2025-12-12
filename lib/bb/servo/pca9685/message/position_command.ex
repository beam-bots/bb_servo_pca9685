# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.PCA9685.Message.PositionCommand do
  @moduledoc """
  Message published by the actuator when a position command is sent.

  Used by `BB.Servo.PCA9685.Sensor` to track commanded positions and estimate
  current position during movement.

  ## Fields

  - `target` - Target position in radians
  - `expected_arrival` - Expected arrival time as monotonic milliseconds
  """

  defstruct [:target, :expected_arrival]

  use BB.Message,
    schema: [
      target: [type: :float, required: true, doc: "Target position in radians"],
      expected_arrival: [
        type: :integer,
        required: true,
        doc: "Expected arrival time (monotonic ms)"
      ]
    ]

  @type t :: %__MODULE__{
          target: float(),
          expected_arrival: integer()
        }
end
