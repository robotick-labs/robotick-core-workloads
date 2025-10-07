// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/data/Blackboard.h"

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits.h>
#include <string>
#include <unistd.h>
#include <vector>

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

		Blackboard mj_initial;
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
		QPos,
		QVel,
		QPosTarget,
		QPosDeg,
		QPosTargetDeg,
		Ctrl,
		XPos,  // → Vec3f
		XQuat, // → Quatf (Vec4f)
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
		mjModel* mujoco_model = nullptr;
		mjData* mujoco_data = nullptr;

		uint32_t sim_num_sub_ticks = 1;

		HeapVector<MuJoCoBinding> config_bindings;
		HeapVector<MuJoCoBinding> input_bindings;
		HeapVector<MuJoCoBinding> output_bindings;

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

		MuJoCoWorkload(){};

		~MuJoCoWorkload()
		{
			if (state->mujoco_data)
			{
				mj_deleteData(state->mujoco_data);
				state->mujoco_data = nullptr;
			}
			if (state->mujoco_model)
			{
				mj_deleteModel(state->mujoco_model);
				state->mujoco_model = nullptr;
			}
		}

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
			if (s == "xpos")
				return MjField::XPos;
			if (s == "xquat")
				return MjField::XQuat;
			if (s == "sensor")
				return MjField::SensorData;
			return MjField::Unknown;
		}

		// --- YAML → binding set up ---

		void configure_io_fields(YAML::Node yaml_node,
			HeapVector<MuJoCoBinding>& bindings,
			HeapVector<FieldDescriptor>& fields,
			bool allow_defaults /*kept for API symmetry; unused here*/)
		{
			const size_t num_entries = yaml_node ? yaml_node.size() : 0;
			bindings.initialize(num_entries);
			fields.initialize(num_entries);

			size_t index = 0;
			for (const auto& item : yaml_node)
			{
				const std::string alias = item.first.as<std::string>();
				const auto& val = item.second;

				FieldDescriptor& fd = fields[index];
				MuJoCoBinding& b = bindings[index];

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

				switch (b.field)
				{
				case MjField::XPos:
					fd.type_id = TypeId(GET_TYPE_ID(Vec3f));
					break;
				case MjField::XQuat:
					fd.type_id = TypeId(GET_TYPE_ID(Quatf));
					break;
				default:
					fd.type_id = TypeId(GET_TYPE_ID(float));
					break;
				}

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
			config.mj_initial.initialize_fields(state->config_fields);
			inputs.mujoco.initialize_fields(state->input_fields);
			outputs.mujoco.initialize_fields(state->output_fields);
		}

		void resolve_binding_ids(MuJoCoBinding& b)
		{
			mjModel* mujoco_model = state->mujoco_model;
			ROBOTICK_ASSERT(mujoco_model != nullptr);

			switch (b.entity_type)
			{
			case MjEntityType::Joint:
				b.mj_id = mj_name2id(mujoco_model, mjOBJ_JOINT, b.name.c_str());
				ROBOTICK_ASSERT_MSG(b.mj_id >= 0, "Joint '%s' not found.", b.name.c_str());
				break;

			case MjEntityType::Actuator:
				b.mj_id = mj_name2id(mujoco_model, mjOBJ_ACTUATOR, b.name.c_str());
				ROBOTICK_ASSERT_MSG(b.mj_id >= 0, "Actuator '%s' not found.", b.name.c_str());
				break;

			case MjEntityType::Body:
				b.mj_id = mj_name2id(mujoco_model, mjOBJ_BODY, b.name.c_str());
				ROBOTICK_ASSERT_MSG(b.mj_id >= 0, "Body '%s' not found.", b.name.c_str());
				break;

			case MjEntityType::Sensor:
				b.mj_id = mj_name2id(mujoco_model, mjOBJ_SENSOR, b.name.c_str());
				ROBOTICK_ASSERT_MSG(b.mj_id >= 0, "Sensor '%s' not found.", b.name.c_str());
				// sensor data slice
				b.sensor_datastart = mujoco_model->sensor_adr[b.mj_id];
				b.sensor_dim = mujoco_model->sensor_dim[b.mj_id];
				break;

			default:
				ROBOTICK_FATAL_EXIT("Unknown entity type for alias '%s'", b.alias.c_str());
			}
		}

		void load_model()
		{
			// Load model
			char error[512] = {0};
			mjModel* mujoco_model = mj_loadXML(config.model_path.c_str(), nullptr, error, sizeof(error));
			if (!mujoco_model)
			{
				ROBOTICK_FATAL_EXIT("mj_loadXML failed: %s", error);
			}
			state->mujoco_model = mujoco_model;

			// Allocate mjData
			state->mujoco_data = mj_makeData(mujoco_model);
			ROBOTICK_ASSERT(state->mujoco_data != nullptr);

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
		static float deg_to_rad(float mujoco_data) { return mujoco_data * (static_cast<float>(M_PI) / 180.0f); }

		void assign_blackboard_from_mujoco(const MuJoCoBinding& b, Blackboard& bb)
		{
			const FieldDescriptor& fd = *b.blackboard_field;
			mjModel* mujoco_model = state->mujoco_model;
			mjData* mujoco_data = state->mujoco_data;

			float value = 0.0f;

			switch (b.entity_type)
			{
			case MjEntityType::Joint:
			{
				const int j = b.mj_id;
				const int dof_qposadr = mujoco_model->jnt_qposadr[j];
				const int dof_dofadr = mujoco_model->jnt_dofadr[j]; // for qvel

				if (b.field == MjField::QPos || b.field == MjField::QPosDeg || b.field == MjField::QPosTarget || b.field == MjField::QPosTargetDeg)
				{
					value = static_cast<float>(mujoco_data->qpos[dof_qposadr]);
					if (b.field == MjField::QPosDeg || b.field == MjField::QPosTargetDeg)
						value = rad_to_deg(value);
				}
				else if (b.field == MjField::QVel)
				{
					value = static_cast<float>(mujoco_data->qvel[dof_dofadr]);
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
					value = static_cast<float>(mujoco_data->ctrl[a]);
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
				if (b.field == MjField::XPos)
				{
					Vec3f v;
					const int i = 3 * b.mj_id;
					v.x = static_cast<float>(mujoco_data->xpos[i + 0]);
					v.y = static_cast<float>(mujoco_data->xpos[i + 1]);
					v.z = static_cast<float>(mujoco_data->xpos[i + 2]);
					bb.set<Vec3f>(fd, v);
					return;
				}
				else if (b.field == MjField::XQuat)
				{
					Quatf q;
					const int i = 4 * b.mj_id;
					q.w = static_cast<float>(mujoco_data->xquat[i + 0]);
					q.x = static_cast<float>(mujoco_data->xquat[i + 1]);
					q.y = static_cast<float>(mujoco_data->xquat[i + 2]);
					q.z = static_cast<float>(mujoco_data->xquat[i + 3]);
					bb.set<Quatf>(fd, q);
					return;
				}
				break;
			}

			case MjEntityType::Sensor:
			{
				ROBOTICK_ASSERT(b.sensor_datastart >= 0 && b.sensor_dim > 0);
				value = static_cast<float>(mujoco_data->sensordata[b.sensor_datastart]); // take first component by convention
				break;
			}

			default:
				ROBOTICK_FATAL_EXIT("Unknown entity type in assign_blackboard_from_mujoco()");
			}

			bb.set<float>(fd, value);
		}

		void assign_mj_from_blackboard(const MuJoCoBinding& b, const Blackboard& bb)
		{
			const FieldDescriptor& fd = *b.blackboard_field;
			const float v = bb.get<float>(fd);

			mjModel* mujoco_model = state->mujoco_model;
			mjData* mujoco_data = state->mujoco_data;

			switch (b.entity_type)
			{
			case MjEntityType::Joint:
			{
				const int j = b.mj_id;
				const int qpos_adr = mujoco_model->jnt_qposadr[j];
				if (b.field == MjField::QPosTarget || b.field == MjField::QPosTargetDeg)
				{
					float rad = (b.field == MjField::QPosTargetDeg) ? deg_to_rad(v) : v;
					mujoco_data->qpos[qpos_adr] = rad; // direct set (no PD); consider mj_kinematics if changing positions directly
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
					mujoco_data->ctrl[a] = v;
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

		void initialize_blackboard_from_mujoco(const HeapVector<MuJoCoBinding>& bindings, Blackboard& bb)
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
			mj_forward(state->mujoco_model, state->mujoco_data);

			mjModel* m = state->mujoco_model;
			mjData* d = state->mujoco_data;

			// hard-reset all controls this tick
			if (m->nu > 0)
				std::fill(d->ctrl, d->ctrl + m->nu, 0.0);

			// Initialize blackboards from sim snapshots
			initialize_blackboard_from_mujoco(state->output_bindings, outputs.mujoco);
		}

		void start(float tick_rate_hz)
		{
			// Decide physics sub-stepping
			float sim_rate = (config.sim_tick_rate_hz > 0.0f) ? config.sim_tick_rate_hz : tick_rate_hz;
			state->sim_num_sub_ticks = static_cast<uint32_t>(std::round(sim_rate / tick_rate_hz));
			if (state->sim_num_sub_ticks == 0)
				state->sim_num_sub_ticks = 1;

			// MuJoCo uses dt in the model; you can override it by scaling mujoco_model->opt.timestep if needed
			const float final_sim_rate = tick_rate_hz * static_cast<float>(state->sim_num_sub_ticks);
			const double dt = 1.0 / static_cast<double>(final_sim_rate);
			state->mujoco_model->opt.timestep = dt;
		}

		void dump_mujoco_state_debug(const mjModel* m, const mjData* d, int tick_count)
		{
			char filename[256];
			snprintf(filename, sizeof(filename), "mujoco_log/mj_dump_tick_%05d.txt", tick_count);

			std::ofstream out(filename);
			if (!out.is_open())
				return;

			out << "Tick: " << tick_count << "\n";
			out << "Time: " << d->time << "\n\n";

			out << "[qpos]\n";
			for (int i = 0; i < m->nq; ++i)
				out << "  qpos[" << i << "]: " << d->qpos[i] << "\n";

			out << "\n[qvel]\n";
			for (int i = 0; i < m->nv; ++i)
				out << "  qvel[" << i << "]: " << d->qvel[i] << "\n";

			out << "\n[ctrl]\n";
			for (int i = 0; i < m->nu; ++i)
				out << "  ctrl[" << i << "]: " << d->ctrl[i] << "\n";

			out << "\n[actuator_force]\n";
			for (int i = 0; i < m->nu; ++i)
				out << "  actuator_force[" << i << "]: " << d->actuator_force[i] << "\n";

			out << "\n[xpos]\n";
			for (int i = 0; i < m->nbody; ++i)
			{
				out << "  xpos[" << i << "]: " << d->xpos[3 * i + 0] << ", " << d->xpos[3 * i + 1] << ", " << d->xpos[3 * i + 2] << "\n";
			}

			out << "\n[xquat]\n";
			for (int i = 0; i < m->nbody; ++i)
			{
				out << "  xquat[" << i << "]: " << d->xquat[4 * i + 0] << ", " << d->xquat[4 * i + 1] << ", " << d->xquat[4 * i + 2] << ", "
					<< d->xquat[4 * i + 3] << "\n";
			}

			out << "\n[sensordata]\n";
			for (int i = 0; i < m->nsensor; ++i)
			{
				const int start = m->sensor_adr[i];
				const int dim = m->sensor_dim[i];
				out << "  sensor[" << i << "]: ";
				for (int j = 0; j < dim; ++j)
					out << d->sensordata[start + j] << " ";
				out << "\n";
			}

			out.close();
		}

		void tick(const TickInfo& tick_info)
		{
			mjData* mujoco_data = state->mujoco_data;

			// Write inputs to sim
			for (const auto& b : state->input_bindings)
			{
				assign_mj_from_blackboard(b, inputs.mujoco);
			}

			const bool should_pause = false;

			// Advance physics
			if (!should_pause)
			{
				for (uint32_t i = 0; i < state->sim_num_sub_ticks; ++i)
				{
					mj_step(state->mujoco_model, mujoco_data);
				}
			}

			// Read outputs from sim
			for (const auto& b : state->output_bindings)
			{
				assign_blackboard_from_mujoco(b, outputs.mujoco);
			}

			static int tick_debug_counter = 0;
			dump_mujoco_state_debug(state->mujoco_model, state->mujoco_data, tick_debug_counter++);
		}
	};

} // namespace robotick
