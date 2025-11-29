#include "robotick/api.h"

namespace robotick
{
	struct LowPassFilterConfig
	{
		// Time constant in seconds. Smaller = faster response (less smoothing).
		// Typical range: 0.05f .. 1.0f
		float tau_seconds = 0.25f;

		// Guard to avoid numeric issues when tau is tiny/zero.
		float min_tau_seconds = 1e-4f;
	};

	struct LowPassFilterInputs
	{
		float value = 0.0f; // input signal
		bool reset = false; // when true, snap output to value this tick
	};

	struct LowPassFilterOutputs
	{
		float result = 0.0f; // filtered output
	};

	struct LowPassFilterWorkload
	{
		LowPassFilterConfig config;
		LowPassFilterInputs inputs;
		LowPassFilterOutputs outputs;

		void tick(const TickInfo& ti)
		{
			// Derive alpha from dt and tau: alpha = 1 - exp(-dt / tau)
			// Assumes ti.dt_seconds is available. If your TickInfo uses a different name,
			// map it here.
			const float dt = ti.delta_time;

			float tau = config.tau_seconds;
			if (tau < config.min_tau_seconds)
			{
				tau = config.min_tau_seconds;
			}

			// Handle degenerate dt safely (e.g., first frame).
			float alpha = 0.0f;
			if (dt > 0.0f)
			{
				const float u = -dt / tau;
				// For very small |u|, expf(u) ~ 1 + u; alpha stays well-behaved.
				alpha = 1.0f - std::exp(u);
				if (alpha < 0.0f)
				{
					alpha = 0.0f;
				}
				else if (alpha > 1.0f)
				{
					alpha = 1.0f;
				}
			}

			if (inputs.reset)
			{
				outputs.result = inputs.value; // hard snap
			}
			else
			{
				// Exponential smoothing step
				outputs.result += alpha * (inputs.value - outputs.result);
			}
		}
	};
} // namespace robotick
