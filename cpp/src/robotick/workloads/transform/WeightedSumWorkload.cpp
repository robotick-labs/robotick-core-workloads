// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

namespace robotick
{
	struct WeightedSumInputs
	{
		float a = 0.0f;
		float b = 0.0f;

		// Default to even mix:
		float weight_a = 0.5f;
		float weight_b = 0.5f;
	};

	struct WeightedSumOutputs
	{
		float result = 0.0f;
	};

	struct WeightedSumWorkload
	{
		WeightedSumInputs inputs;
		WeightedSumOutputs outputs;

		void tick(const TickInfo&) { outputs.result = inputs.a * inputs.weight_a + inputs.b * inputs.weight_b; }
	};
} // namespace robotick
