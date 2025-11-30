// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/data/Blackboard.h"

namespace robotick
{

	//------------------------------------------------------------------------------
	// Template field structs with all supported field types
	//------------------------------------------------------------------------------

	struct TemplateConfig
	{
		float gain;
		int threshold;
		FixedString32 label;
		Blackboard blackboard;
		// ^- you would tend to only use a Blackboard on a scripting Workload where you don't know your fields in advance - more indirection means
		// they are a bit slower to use
	};

	struct TemplateInputs
	{
		float angle;
		int sensor_value;
		FixedString16 sensor_label;
		Blackboard blackboard; // (see note on TemplateConfig::blackboard)
	};

	struct TemplateOutputs
	{
		float command;
		FixedString64 status;
		bool sensor_ok = true;
		Blackboard blackboard;		// (see note on TemplateConfig::blackboard)
		uint32_t warning_count = 0; // example metric counter you can emit to Hub/telemetry

		bool has_called_set_children = false;
		bool has_called_set_engine = false;
		bool has_called_pre_load = false;
		bool has_called_load = false;
		bool has_called_setup = false;
		bool has_called_start = false;
		bool has_called_tick = false;
		bool has_called_stop = false;
	};

	struct TemplateState
	{
		HeapVector<FieldDescriptor> blackboard_fields_config;
		HeapVector<FieldDescriptor> blackboard_fields_input;
		HeapVector<FieldDescriptor> blackboard_fields_output;
	};

	//------------------------------------------------------------------------------
	// TemplateWorkload: demonstrates full function pointer coverage
	//------------------------------------------------------------------------------

	struct TemplateWorkload
	{
		TemplateConfig config;
		TemplateInputs inputs;
		TemplateOutputs outputs;

		State<TemplateState> state;

		void set_children(const HeapVector<const WorkloadInstanceInfo*>& children, HeapVector<DataConnectionInfo>& connections)
		{
			// Handle child linkage (if applicable - typically only used for compositional workloads
			// e.g. AsyncPairWorkload, SyncedGroupWorkloads, SequencedGroupWorkload)

			outputs.has_called_set_children = true; // (for unit-testing of this template - not for illustrating suggested usage!)
			(void)children;
			(void)connections; // (just to stop compiler warning is about unused args)
		}

		void set_engine(const Engine& engine)
		{
			// Access engine if needed (avoid unless you know what you're doing - typically only used on telemetry- and compositional workloads)

			outputs.has_called_set_engine = true; // (for unit-testing of this template - not for illustrating suggested usage!)
			(void)engine;						  // (just to stop compiler warning is about unused args)
		}

		void pre_load()
		{
			// Called before blackboards or memory allocated...

			// ... meaning it is the correct place to set the schema for each of our blackboards, for example:

			state->blackboard_fields_config.initialize(1);
			state->blackboard_fields_config[0] = FieldDescriptor{"my_config_float", GET_TYPE_ID(float)};
			config.blackboard.initialize_fields(state->blackboard_fields_config);

			state->blackboard_fields_input.initialize(1);
			state->blackboard_fields_input[0] = FieldDescriptor{"my_config_int", GET_TYPE_ID(int)};
			state->blackboard_fields_input[1] = FieldDescriptor{"my_config_string", GET_TYPE_ID(FixedString64)};
			config.blackboard.initialize_fields(state->blackboard_fields_input);

			state->blackboard_fields_output.initialize(1);
			state->blackboard_fields_output[0] = FieldDescriptor{"my_config_string", GET_TYPE_ID(FixedString64)};
			config.blackboard.initialize_fields(state->blackboard_fields_output);

			outputs.has_called_pre_load = true; // (for unit-testing of this template - not for illustrating suggested usage!)
		}

		void load()
		{
			// Called after blackboards allocated, safe to inspect config/inputs/outputs

			outputs.has_called_load = true; // (for unit-testing of this template - not for illustrating suggested usage!)
		}

		void setup()
		{
			// Called once before first tick

			outputs.has_called_setup = true; // (for unit-testing of this template - not for illustrating suggested usage!)
		}

		void start(float tick_rate_hz)
		{
			// Called once when ticking begins

			outputs.has_called_start = true; // (for unit-testing of this template - not for illustrating suggested usage!)
			(void)tick_rate_hz;				 // (just to stop compiler warning is about unused args)
		}

		void tick(const TickInfo& tick_info)
		{
			// Main tick loop

			outputs.has_called_tick = true; // (for unit-testing of this template - not for illustrating suggested usage!)
			(void)tick_info;				// (just to stop compiler warning is about unused args)

			// --- Recommended error/reporting pattern -----------------------------------
			// 1. Inspect inputs/config and compute a status
			const bool sensor_ok = inputs.sensor_value >= config.threshold;

			// 2. Emit an INFO/WARNING/ERROR log entry, depending on severity
			if (!sensor_ok)
			{
				ROBOTICK_WARNING(
					"TemplateWorkload sensor '%s' below threshold (%d < %d)", inputs.sensor_label.c_str(), inputs.sensor_value, config.threshold);
				// 3. Update outputs/telemetry counters so Hub/CLI can surface the issue
				outputs.warning_count++;
			}

			if (outputs.sensor_ok != sensor_ok)
			{
				outputs.sensor_ok = sensor_ok;
				outputs.status = sensor_ok ? "sensor nominal" : "sensor below threshold";
			}
		}

		void stop()
		{
			// Called after ticking has stopped

			outputs.has_called_stop = true; // (for unit-testing of this template - not for illustrating suggested usage!)
		}
	};

} // namespace robotick
