// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

namespace robotick
{

	// === Field registrations ===

	struct HelloConfig
	{
		float multiplier = 1.0;
	};

	struct HelloInputs
	{
		float a = 0.0;
		float b = 0.0;
	};

	enum class HelloStatus
	{
		NORMAL,
		MAGIC
	};

	struct HelloOutputs
	{
		float sum = 0.0;
		FixedString32 message = "Waiting...";
		HelloStatus status = HelloStatus::NORMAL;
	};

	// === Workload ===

	struct HelloWorkload
	{
		HelloInputs inputs;
		HelloOutputs outputs;
		HelloConfig config;

		void tick(const TickInfo&)
		{
			outputs.sum = (inputs.a + inputs.b) * config.multiplier;

			if (outputs.sum == 42.0)
			{
				outputs.message = "The Answer!";
				outputs.status = HelloStatus::MAGIC;
			}
			else
			{
				outputs.message.format("Sum = %.2f", outputs.sum);
				outputs.status = HelloStatus::NORMAL;
			}
		}
	};

} // namespace robotick
