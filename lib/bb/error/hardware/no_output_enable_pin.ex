# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Hardware.NoOutputEnablePin do
  @moduledoc """
  Output enable pin not configured.

  The PCA9685 controller was asked to enable or disable outputs, but no
  output enable (OE) GPIO pin was configured. Without an OE pin, the controller
  cannot globally enable or disable all servo outputs.

  Individual actuators can still be disabled by setting their pulse width to 0.
  """
  use BB.Error,
    class: :hardware,
    fields: [:controller]

  @type t :: %__MODULE__{
          controller: atom() | nil
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{controller: nil}) do
    "No output enable pin configured on PCA9685 controller"
  end

  def message(%{controller: controller}) do
    "No output enable pin configured on PCA9685 controller #{inspect(controller)}"
  end
end
