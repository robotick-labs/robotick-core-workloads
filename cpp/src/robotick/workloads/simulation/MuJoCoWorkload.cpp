// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/data/Blackboard.h"
#include "robotick/framework/utility/Algorithm.h"

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>

namespace robotick
{
	// ---------- Config / IO ----------

	struct MuJoCoConfig
	{
		FixedString256 workload_config_file_path;
		FixedString256 model_path;

		float sim_tick_rate_hz = -1.0f;

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

		static MjEntityType parse_entity_type(const StringView& s)
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

		static MjField parse_field(const StringView& s)
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

		void configure_io_fields(YAML::Node yaml_node, HeapVector<MuJoCoBinding>& bindings, HeapVector<FieldDescriptor>& fields)
		{
			const size_t num_entries = yaml_node ? yaml_node.size() : 0;
			bindings.initialize(num_entries);
			fields.initialize(num_entries);

			size_t index = 0;
			for (const auto& item : yaml_node)
			{
				const char* alias_scalar = item.first.Scalar().c_str();
				const auto& val = item.second;

				FieldDescriptor& fd = fields[index];
				MuJoCoBinding& b = bindings[index];

				b.alias = alias_scalar;
				b.blackboard_field = &fd;

				fd.name = b.alias.c_str();

				// Expect sequences like: ["joint","hinge_pitch","qpos_deg"]
				ROBOTICK_ASSERT_MSG(val.IsSequence() && val.size() >= 3, "Malformed YAML for '%s' (expect [entity,name,field]).", b.alias.c_str());

				b.entity_type = parse_entity_type(StringView(val[0].Scalar().c_str()));
				b.name = val[1].Scalar().c_str();
				b.field = parse_field(StringView(val[2].Scalar().c_str()));

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
			// 1) Parse YAML first (so fields exist)
			configure_from_config_file();

			// 2) Load model now so we can query sensor_dims before blackboard sizing lock-in
			load_model();

			// 3) After ids are resolved by load_model(), adjust sensor field types and re-init outputs BB
			finalize_sensor_output_field_types();
		}

		void finalize_sensor_output_field_types()
		{
			mjModel* m = state->mujoco_model;
			ROBOTICK_ASSERT(m != nullptr);

			bool changed = false;

			for (auto& b : state->output_bindings)
			{
				if (b.entity_type != MjEntityType::Sensor)
					continue;
				ROBOTICK_ASSERT(b.mj_id >= 0);
				const int dim = m->sensor_dim[b.mj_id];

				TypeId desired = TypeId(GET_TYPE_ID(float));
				if (dim == 3)
					desired = TypeId(GET_TYPE_ID(Vec3f));
				else if (dim == 4)
					desired = TypeId(GET_TYPE_ID(Quatf));
				else if (dim == 1)
					desired = TypeId(GET_TYPE_ID(float));
				else
				{
					ROBOTICK_FATAL_EXIT("Sensor '%s' has unsupported dimension %d. "
										"Currently supported: 1->float, 3->Vec3f, 4->Quatf.",
						b.name.c_str(),
						dim);
				}

				FieldDescriptor* fd = b.blackboard_field;
				ROBOTICK_ASSERT(fd != nullptr);
				if (fd->type_id != desired)
				{
					fd->type_id = desired;
					changed = true;
				}
			}

			// If any types changed, reinitialize outputs blackboard with updated descriptors
			if (changed)
			{
				outputs.mujoco.initialize_fields(state->output_fields);
			}
		}

		void configure_from_config_file()
		{
			const StringView path = config.workload_config_file_path.c_str();

			YAML::Node root;
			try
			{
				root = YAML::LoadFile(path.c_str());
			}
			catch (const YAML::BadFile&)
			{
				ROBOTICK_FATAL_EXIT("Failed to open YAML config file: %s", path.c_str());
			}
			if (!root || !root.IsMap())
			{
				ROBOTICK_FATAL_EXIT("Invalid YAML root: %s", path.c_str());
			}

			YAML::Node mujoco = root["mujoco"];
			if (!mujoco || !mujoco.IsMap())
			{
				ROBOTICK_FATAL_EXIT("Missing 'mujoco' map in: %s", path.c_str());
			}

			const YAML::Node model_path_node = mujoco["model_path"];
			if (model_path_node && model_path_node.IsScalar())
			{
				config.model_path = model_path_node.Scalar().c_str();
			}
			else
			{
				config.model_path.clear();
			}
			ROBOTICK_ASSERT_MSG(!config.model_path.empty(), "mujoco.model_path is required.");

			config.sim_tick_rate_hz = mujoco["sim_tick_rate_hz"].as<float>(-1.0f);

			// Build binding lists and field descriptors
			configure_io_fields(mujoco["config"], state->config_bindings, state->config_fields);
			configure_io_fields(mujoco["inputs"], state->input_bindings, state->input_fields);
			configure_io_fields(mujoco["outputs"], state->output_bindings, state->output_fields);

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

			state->mujoco_data = mj_makeData(mujoco_model);
			ROBOTICK_ASSERT(state->mujoco_data != nullptr);

			for (auto& b : state->config_bindings)
				resolve_binding_ids(b);
			for (auto& b : state->input_bindings)
				resolve_binding_ids(b);
			for (auto& b : state->output_bindings)
				resolve_binding_ids(b);
		}

		// --- Blackboard <-> MuJoCo ---

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

				const TypeId tV3 = TypeId(GET_TYPE_ID(Vec3f));
				const TypeId tQuat = TypeId(GET_TYPE_ID(Quatf));

				const int start = b.sensor_datastart;

				if (fd.type_id == tV3)
				{
					ROBOTICK_ASSERT(b.sensor_dim >= 3);
					Vec3f v;
					v.x = static_cast<float>(mujoco_data->sensordata[start + 0]);
					v.y = static_cast<float>(mujoco_data->sensordata[start + 1]);
					v.z = static_cast<float>(mujoco_data->sensordata[start + 2]);
					bb.set<Vec3f>(fd, v);
					return;
				}
				else if (fd.type_id == tQuat)
				{
					ROBOTICK_ASSERT(b.sensor_dim >= 4);
					Quatf q;
					q.w = static_cast<float>(mujoco_data->sensordata[start + 0]);
					q.x = static_cast<float>(mujoco_data->sensordata[start + 1]);
					q.y = static_cast<float>(mujoco_data->sensordata[start + 2]);
					q.z = static_cast<float>(mujoco_data->sensordata[start + 3]);
					bb.set<Quatf>(fd, q);
					return;
				}
				else // float fallback (dim==1)
				{
					float value = static_cast<float>(mujoco_data->sensordata[start + 0]);
					bb.set<float>(fd, value);
					return;
				}
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
					mujoco_data->qpos[qpos_adr] = rad;
					mj_kinematics(mujoco_model, mujoco_data);
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
			// Optionally run forward to make derived quantities valid
			mj_forward(state->mujoco_model, state->mujoco_data);

			mjModel* m = state->mujoco_model;
			mjData* d = state->mujoco_data;

			// hard-reset all controls this tick
			if (m->nu > 0)
			{
				robotick::fill(d->ctrl, d->ctrl + m->nu, 0.0);
			}

			// Initialize blackboards from sim snapshots
			initialize_blackboard_from_mujoco(state->output_bindings, outputs.mujoco);
		}

		void start(float tick_rate_hz)
		{
			// Decide physics sub-stepping
			float sim_rate = (config.sim_tick_rate_hz > 0.0f) ? config.sim_tick_rate_hz : tick_rate_hz;
			state->sim_num_sub_ticks = static_cast<uint32_t>(roundf(sim_rate / tick_rate_hz));
			if (state->sim_num_sub_ticks == 0)
			{
				state->sim_num_sub_ticks = 1;
			}

			// MuJoCo uses dt in the model; you can override it by scaling mujoco_model->opt.timestep if needed
			const float final_sim_rate = tick_rate_hz * static_cast<float>(state->sim_num_sub_ticks);
			const double dt = 1.0 / static_cast<double>(final_sim_rate);
			state->mujoco_model->opt.timestep = dt;
		}

		void tick(const TickInfo& tick_info)
		{
			(void)tick_info;

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
		}
	};

} // namespace robotick
