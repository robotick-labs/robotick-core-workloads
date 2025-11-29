// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

#include <algorithm>

namespace robotick
{
	struct BoolToFloatConfig
	{
		float float_value_when_true = 1.0f;
		float float_value_when_false = 0.0f;
	};

	struct BoolToFloatInputs
	{
		bool bool_value = false;
	};

	struct BoolToFloatOutputs
	{
		float float_value = 0.0f;
	};

	struct BoolToFloatWorkload
	{
		BoolToFloatConfig config;
		BoolToFloatInputs inputs;
		BoolToFloatOutputs outputs;

		void evaluate() { outputs.float_value = inputs.bool_value ? config.float_value_when_true : config.float_value_when_false; }

		void start() { evaluate(); }

		void tick(const TickInfo& info)
		{
			(void)info; // unused

			evaluate();
		}
	};
} // namespace robotick
