// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/data/Blackboard.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits.h>
#include <string>
#include <unistd.h>
#include <vector>

// YAML
#include <yaml-cpp/yaml.h>

// MuJoCo
#include <mujoco/mujoco.h> // assume system has MuJoCo 3.x headers available

namespace robotick
{
	// ---------- Config / IO ----------

	struct MuJoCoConfig
	{
		FixedString256 workload_config_file_path;
		FixedString256 model_path;

		float sim_tick_rate_hz = -1.0f;

		// Optional simple pause trigger (example)
		FixedString64 pause_body_name;
		float pause_body_z_threshold = -1.0f;

		Blackboard mujoco_initial;
		// ^- config/initial-conditions snapshot read from sim at setup
	};

	struct MuJoCoInputs
	{
		Blackboard mujoco;
		// ^- values written into sim each tick (e.g., actuator ctrl)
	};

	struct MuJoCoOutputs
	{
		Blackboard mujoco;
		// ^- values read from sim each tick
	};

	// ---------- Binding model ----------

	enum class MjEntityType : uint8_t
	{
		Joint,
		Actuator,
		Body,
		Sensor,
		Unknown
	};
	enum class MjField : uint8_t
	{
		// joints:
		QPos,
		QVel,
		QPosDeg,
		QPosTarget,
		QPosTargetDeg,
		// actuators:
		Ctrl,
		// bodies:
		XPosX,
		XPosY,
		XPosZ,
		XQuatW,
		XQuatX,
		XQuatY,
		XQuatZ,
		// sensors (flat mjData->sensordata):
		SensorData,
		Unknown
	};

	struct MuJoCoBinding
	{
		FixedString64 alias; // blackboard field alias
		FixedString64 name;	 // MJ name (joint/actuator/body/sensor)
		MjEntityType entity_type = MjEntityType::Unknown;
		MjField field = MjField::Unknown;

		// resolved indices:
		int mj_id = -1;			   // e.g., joint id, actuator id, body id, sensor id
		int sensor_datastart = -1; // for sensors: start index into sensordata
		int sensor_dim = 0;		   // for sensors: dimension

		FieldDescriptor* blackboard_field = nullptr;
	};

	// ---------- State ----------

	struct MuJoCoState
	{
		mjModel* m = nullptr;
		mjData* d = nullptr;

		uint32_t sim_num_sub_ticks = 1;

		std::vector<MuJoCoBinding> config_bindings;
		std::vector<MuJoCoBinding> input_bindings;
		std::vector<MuJoCoBinding> output_bindings;

		HeapVector<FieldDescriptor> config_fields;
		HeapVector<FieldDescriptor> input_fields;
		HeapVector<FieldDescriptor> output_fields;
	};

	// ---------- Workload ----------

	struct MuJoCoWorkload
	{
		MuJoCoConfig config;
		MuJoCoInputs inputs;
		MuJoCoOutputs outputs;

		State<MuJoCoState> state;

		// --- helpers: field parsing ---

		static MjEntityType parse_entity_type(const std::string& s)
		{
			if (s == "joint")
				return MjEntityType::Joint;
			if (s == "actuator")
				return MjEntityType::Actuator;
			if (s == "body")
				return MjEntityType::Body;
			if (s == "sensor")
				return MjEntityType::Sensor;
			return MjEntityType::Unknown;
		}

		static MjField parse_field(const std::string& s)
		{
			if (s == "qpos")
				return MjField::QPos;
			if (s == "qvel")
				return MjField::QVel;
			if (s == "qpos_deg")
				return MjField::QPosDeg;
			if (s == "qpos_target")
				return MjField::QPosTarget;
			if (s == "qpos_target_deg")
				return MjField::QPosTargetDeg;
			if (s == "ctrl")
				return MjField::Ctrl;
			if (s == "xpos_x")
				return MjField::XPosX;
			if (s == "xpos_y")
				return MjField::XPosY;
			if (s == "xpos_z")
				return MjField::XPosZ;
			if (s == "xquat_w")
				return MjField::XQuatW;
			if (s == "xquat_x")
				return MjField::XQuatX;
			if (s == "xquat_y")
				return MjField::XQuatY;
			if (s == "xquat_z")
				return MjField::XQuatZ;
			if (s == "sensor")
				return MjField::SensorData;
			return MjField::Unknown;
		}

		// --- YAML → binding set up ---

		void configure_io_fields(YAML::Node yaml_node,
			std::vector<MuJoCoBinding>& bindings,
			HeapVector<FieldDescriptor>& fields,
			bool allow_defaults /*kept for API symmetry; unused here*/)
		{
			const size_t num_entries = yaml_node ? yaml_node.size() : 0;
			bindings.reserve(num_entries);
			fields.initialize(num_entries);

			size_t index = 0;
			for (const auto& item : yaml_node)
			{
				const std::string alias = item.first.as<std::string>();
				const auto& val = item.second;

				FieldDescriptor& fd = fields[index];

				MuJoCoBinding& b = bindings.emplace_back();
				b.alias = alias.c_str();
				b.blackboard_field = &fd;

				fd.name = b.alias.c_str();

				// Expect sequences like: ["joint","hinge_pitch","qpos_deg"]
				ROBOTICK_ASSERT_MSG(val.IsSequence() && val.size() >= 3, "Malformed YAML for '%s' (expect [entity,name,field]).", alias.c_str());

				const std::string ent = val[0].as<std::string>();
				const std::string nam = val[1].as<std::string>();
				const std::string fld = val[2].as<std::string>();

				b.entity_type = parse_entity_type(ent);
				b.name = nam.c_str();
				b.field = parse_field(fld);

				// Type selection: everything here resolves to numeric scalar (float)
				fd.type_id = TypeId(GET_TYPE_ID(float));
				ROBOTICK_ASSERT(TypeRegistry::get().find_by_id(fd.type_id) != nullptr);

				index++;
			}
		}

		// --- model loading ---

		void pre_load()
		{
			// parse YAML first so engine can size blackboards
			configure_from_config_file();
		}

		void configure_from_config_file()
		{
			const char* path = config.workload_config_file_path.c_str();

			std::ifstream fin(path);
			if (!fin.is_open())
			{
				char cwd[512] = "?";
				getcwd(cwd, sizeof(cwd));

				ROBOTICK_FATAL_EXIT("Failed to open YAML config file: %s (cwd: %s)", path, cwd);
			}
			YAML::Node root = YAML::Load(fin);
			if (!root || !root.IsMap())
			{
				ROBOTICK_FATAL_EXIT("Invalid YAML root: %s", path);
			}

			YAML::Node mujoco = root["mujoco"];
			if (!mujoco || !mujoco.IsMap())
			{
				ROBOTICK_FATAL_EXIT("Missing 'mujoco' map in: %s", path);
			}

			config.model_path = mujoco["model_path"].as<std::string>("").c_str();
			ROBOTICK_ASSERT_MSG(!config.model_path.empty(), "mujoco.model_path is required.");

			config.sim_tick_rate_hz = mujoco["sim_tick_rate_hz"].as<float>(-1.0f);

			// Build binding lists and field descriptors
			configure_io_fields(mujoco["config"], state->config_bindings, state->config_fields, /*allow_defaults*/ true);
			configure_io_fields(mujoco["inputs"], state->input_bindings, state->input_fields, /*allow_defaults*/ true);
			configure_io_fields(mujoco["outputs"], state->output_bindings, state->output_fields, /*allow_defaults*/ false);

			// Initialize blackboards with those descriptors
			config.mujoco_initial.initialize_fields(state->config_fields);
			inputs.mujoco.initialize_fields(state->input_fields);
			outputs.mujoco.initialize_fields(state->output_fields);
		}

		void resolve_binding_ids(MuJoCoBinding& b)
		{
			mjModel* m = state->m;
			ROBOTICK_ASSERT(m != nullptr);

			switch (b.entity_type)
			{
			case MjEntityType::Joint:
				b.mj_id = mj_name2id(m, mjOBJ_JOINT, b.name.c_str());
				ROBOTICK_ASSERT_MSG(b.mj_id >= 0, "Joint '%s' not found.", b.name.c_str());
				break;

			case MjEntityType::Actuator:
				b.mj_id = mj_name2id(m, mjOBJ_ACTUATOR, b.name.c_str());
				ROBOTICK_ASSERT_MSG(b.mj_id >= 0, "Actuator '%s' not found.", b.name.c_str());
				break;

			case MjEntityType::Body:
				b.mj_id = mj_name2id(m, mjOBJ_BODY, b.name.c_str());
				ROBOTICK_ASSERT_MSG(b.mj_id >= 0, "Body '%s' not found.", b.name.c_str());
				break;

			case MjEntityType::Sensor:
				b.mj_id = mj_name2id(m, mjOBJ_SENSOR, b.name.c_str());
				ROBOTICK_ASSERT_MSG(b.mj_id >= 0, "Sensor '%s' not found.", b.name.c_str());
				// sensor data slice
				b.sensor_datastart = m->sensor_adr[b.mj_id];
				b.sensor_dim = m->sensor_dim[b.mj_id];
				break;

			default:
				ROBOTICK_FATAL_EXIT("Unknown entity type for alias '%s'", b.alias.c_str());
			}
		}

		void load_model()
		{
			// Load model
			char error[512] = {0};
			mjModel* m = mj_loadXML(config.model_path.c_str(), nullptr, error, sizeof(error));
			if (!m)
			{
				ROBOTICK_FATAL_EXIT("mj_loadXML failed: %s", error);
			}
			state->m = m;

			// Allocate mjData
			state->d = mj_makeData(m);
			ROBOTICK_ASSERT(state->d != nullptr);

			// Resolve all bindings to IDs
			for (auto& b : state->config_bindings)
				resolve_binding_ids(b);
			for (auto& b : state->input_bindings)
				resolve_binding_ids(b);
			for (auto& b : state->output_bindings)
				resolve_binding_ids(b);
		}

		// --- Blackboard <-> MuJoCo ---

		static float rad_to_deg(float r) { return r * (180.0f / static_cast<float>(M_PI)); }
		static float deg_to_rad(float d) { return d * (static_cast<float>(M_PI) / 180.0f); }

		void assign_blackboard_from_mujoco(const MuJoCoBinding& b, Blackboard& bb)
		{
			const FieldDescriptor& fd = *b.blackboard_field;
			mjModel* m = state->m;
			mjData* d = state->d;

			float value = 0.0f;

			switch (b.entity_type)
			{
			case MjEntityType::Joint:
			{
				const int j = b.mj_id;
				const int dof_qposadr = m->jnt_qposadr[j];
				const int dof_dofadr = m->jnt_dofadr[j]; // for qvel

				if (b.field == MjField::QPos || b.field == MjField::QPosDeg || b.field == MjField::QPosTarget || b.field == MjField::QPosTargetDeg)
				{
					value = static_cast<float>(d->qpos[dof_qposadr]);
					if (b.field == MjField::QPosDeg || b.field == MjField::QPosTargetDeg)
						value = rad_to_deg(value);
				}
				else if (b.field == MjField::QVel)
				{
					value = static_cast<float>(d->qvel[dof_dofadr]);
				}
				else
				{
					ROBOTICK_FATAL_EXIT("Unsupported joint field %i for '%s'", (int)b.field, b.alias.c_str());
				}
				break;
			}

			case MjEntityType::Actuator:
			{
				const int a = b.mj_id;
				if (b.field == MjField::Ctrl)
				{
					value = static_cast<float>(d->ctrl[a]);
				}
				else
				{
					ROBOTICK_FATAL_EXIT("Unsupported actuator field for '%s'", b.alias.c_str());
				}
				break;
			}

			case MjEntityType::Body:
			{
				const int bidx = b.mj_id;
				if (b.field == MjField::XPosX)
					value = static_cast<float>(d->xpos[3 * bidx + 0]);
				else if (b.field == MjField::XPosY)
					value = static_cast<float>(d->xpos[3 * bidx + 1]);
				else if (b.field == MjField::XPosZ)
					value = static_cast<float>(d->xpos[3 * bidx + 2]);
				else
				{
					ROBOTICK_FATAL_EXIT("Unsupported body field for '%s'", b.alias.c_str());
				}
				break;
			}

			case MjEntityType::Sensor:
			{
				ROBOTICK_ASSERT(b.sensor_datastart >= 0 && b.sensor_dim > 0);
				value = static_cast<float>(d->sensordata[b.sensor_datastart]); // take first component by convention
				break;
			}

			default:
				ROBOTICK_FATAL_EXIT("Unknown entity type in assign_blackboard_from_mujoco()");
			}

			bb.set<float>(fd, value);
		}

		void assign_mujoco_from_blackboard(const MuJoCoBinding& b, const Blackboard& bb)
		{
			const FieldDescriptor& fd = *b.blackboard_field;
			const float v = bb.get<float>(fd);

			mjModel* m = state->m;
			mjData* d = state->d;

			switch (b.entity_type)
			{
			case MjEntityType::Joint:
			{
				const int j = b.mj_id;
				const int qpos_adr = m->jnt_qposadr[j];
				if (b.field == MjField::QPosTarget || b.field == MjField::QPosTargetDeg)
				{
					float rad = (b.field == MjField::QPosTargetDeg) ? deg_to_rad(v) : v;
					d->qpos[qpos_adr] = rad; // direct set (no PD); consider mj_kinematics if changing positions directly
				}
				else
				{
					ROBOTICK_FATAL_EXIT("Unsupported joint input field for '%s'", b.alias.c_str());
				}
				break;
			}

			case MjEntityType::Actuator:
			{
				const int a = b.mj_id;
				if (b.field == MjField::Ctrl)
				{
					d->ctrl[a] = v;
				}
				else
				{
					ROBOTICK_FATAL_EXIT("Unsupported actuator input field for '%s'", b.alias.c_str());
				}
				break;
			}

			default:
				ROBOTICK_FATAL_EXIT("Unsupported entity type for inputs on '%s'", b.alias.c_str());
			}
		}

		void initialize_blackboard_from_mujoco(const std::vector<MuJoCoBinding>& bindings, Blackboard& bb)
		{
			for (const auto& b : bindings)
			{
				ROBOTICK_ASSERT(b.blackboard_field != nullptr);
				assign_blackboard_from_mujoco(b, bb);
			}
		}

		// --- lifecycle ---

		void setup()
		{
			load_model();

			// Optionally run forward to make derived quantities valid
			mj_forward(state->m, state->d);

			// Initialize blackboards from sim snapshots
			initialize_blackboard_from_mujoco(state->config_bindings, config.mujoco_initial);
			initialize_blackboard_from_mujoco(state->input_bindings, inputs.mujoco);
			initialize_blackboard_from_mujoco(state->output_bindings, outputs.mujoco);
		}

		void start(float tick_rate_hz)
		{
			// Decide physics sub-stepping
			float sim_rate = (config.sim_tick_rate_hz > 0.0f) ? config.sim_tick_rate_hz : tick_rate_hz;
			state->sim_num_sub_ticks = static_cast<uint32_t>(std::round(sim_rate / tick_rate_hz));
			if (state->sim_num_sub_ticks == 0)
				state->sim_num_sub_ticks = 1;

			// MuJoCo uses dt in the model; you can override it by scaling m->opt.timestep if needed
			const float final_sim_rate = tick_rate_hz * static_cast<float>(state->sim_num_sub_ticks);
			const double dt = 1.0 / static_cast<double>(final_sim_rate);
			state->m->opt.timestep = dt;
		}

		void tick(const TickInfo& tick_info)
		{
			mjData* d = state->d;

			// Write inputs to sim
			for (const auto& b : state->input_bindings)
			{
				assign_mujoco_from_blackboard(b, inputs.mujoco);
			}

			const bool should_pause = false;

			// Advance physics
			if (!should_pause)
			{
				for (uint32_t i = 0; i < state->sim_num_sub_ticks; ++i)
				{
					mj_step(state->m, d);
				}
			}

			// Read outputs from sim
			for (const auto& b : state->output_bindings)
			{
				assign_blackboard_from_mujoco(b, outputs.mujoco);
			}
		}

		// --- teardown ---

		~MuJoCoWorkload()
		{
			if (state->d)
			{
				mj_deleteData(state->d);
				state->d = nullptr;
			}
			if (state->m)
			{
				mj_deleteModel(state->m);
				state->m = nullptr;
			}
		}
	};

} // namespace robotick
