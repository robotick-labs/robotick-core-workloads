// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/WorkloadInstanceInfo.h"
#include "robotick/framework/data/DataConnection.h"
#include "robotick/framework/time/Clock.h"

#include <string>
namespace robotick
{
	struct SequencedGroupWorkloadImpl
	{
		struct ChildWorkloadInfo
		{
			const WorkloadInstanceInfo* workload_info = nullptr;
			void* workload_ptr = nullptr;

			List<const DataConnectionInfo*> connections_in;
		};

		const Engine* engine = nullptr;
		HeapVector<ChildWorkloadInfo> children;

		void set_engine(const Engine& engine_in) { engine = &engine_in; }

		void start(float)
		{
			for (auto& child : children)
			{
				if (!child.workload_info || !child.workload_info->workload_descriptor)
					continue;

				if (child.workload_info->workload_descriptor->start_fn)
				{
					child.workload_info->workload_descriptor->start_fn(child.workload_ptr, child.workload_info->seed->tick_rate_hz);
				}
			}
		}

		ChildWorkloadInfo* find_child_workload(const WorkloadInstanceInfo& query_child)
		{
			for (ChildWorkloadInfo& child : children)
			{
				if (child.workload_info == &query_child)
				{
					return &child;
				}
			}

			return nullptr;
		}

		void set_children(const HeapVector<const WorkloadInstanceInfo*>& child_workloads, HeapVector<DataConnectionInfo>& pending_connections)
		{
			ROBOTICK_ASSERT(engine != nullptr && "Engine should have been set by now");

			children.initialize(child_workloads.size());
			size_t child_index = 0;

			// add child workloads and call set_children_fn on each, if present:
			for (const WorkloadInstanceInfo* child_workload : child_workloads)
			{
				ChildWorkloadInfo& info = children[child_index];
				child_index++;

				info.workload_info = child_workload;
				info.workload_ptr = child_workload->get_ptr(*engine);

				ROBOTICK_ASSERT(info.workload_info && info.workload_info->type);
				const WorkloadDescriptor* workload_desc = info.workload_info->type->get_workload_desc();
				ROBOTICK_ASSERT(workload_desc);

				if (workload_desc->set_children_fn)
				{
					workload_desc->set_children_fn(info.workload_ptr, info.workload_info->children, pending_connections);
				}
			}

			// iterate + classify connections
			for (DataConnectionInfo& conn : pending_connections)
			{
				if (conn.expected_handler != DataConnectionInfo::ExpectedHandler::Unassigned)
				{
					continue;
				}

				const bool src_is_local = find_child_workload(*conn.source_workload) != nullptr;

				ChildWorkloadInfo* dest_child_info = find_child_workload(*conn.dest_workload);
				const bool dst_is_local = (dest_child_info != nullptr);

				if (src_is_local && dst_is_local)
				{
					ROBOTICK_ASSERT(dest_child_info != nullptr);
					dest_child_info->connections_in.push_back(&conn);
					conn.expected_handler = DataConnectionInfo::ExpectedHandler::SequencedGroupWorkload;
				}
				else
				{
					if (dst_is_local)
					{
						conn.expected_handler = DataConnectionInfo::ExpectedHandler::DelegateToParent;
					}
				}
			}
		}

		void tick(const TickInfo& tick_info)
		{
			ROBOTICK_ASSERT(engine != nullptr && "Engine should have been set by now");

			for (auto& child_info : children)
			{
				if (child_info.workload_info != nullptr && child_info.workload_info->workload_descriptor->tick_fn != nullptr)
				{
					// process any incoming data-connections:
					for (auto connection_in : child_info.connections_in)
					{
						connection_in->do_data_copy();
					}

					TickInfo child_tick_info(tick_info);
					child_tick_info.workload_stats = child_info.workload_info->workload_stats;

					const auto budget_duration = Clock::from_seconds(1.0f / child_info.workload_info->seed->tick_rate_hz);
					const uint32_t budget_ns = detail::clamp_to_uint32(Clock::to_nanoseconds(budget_duration).count());

					const auto now_pre_tick = Clock::now();
					child_info.workload_info->workload_descriptor->tick_fn(child_info.workload_ptr, child_tick_info);
					const auto now_post_tick = Clock::now();

					const uint32_t duration_ns = detail::clamp_to_uint32(Clock::to_nanoseconds(now_post_tick - now_pre_tick).count());

					child_info.workload_info->workload_stats->last_time_delta_ns = static_cast<uint32_t>(child_tick_info.delta_time * 1e9f);
					child_info.workload_info->workload_stats->record_tick_duration_ns(duration_ns, budget_ns);
				}
			}
		}
	};

	struct SequencedGroupWorkload
	{
		SequencedGroupWorkloadImpl* impl = nullptr;

		SequencedGroupWorkload()
			: impl(new SequencedGroupWorkloadImpl())
		{
		}
		~SequencedGroupWorkload()
		{
			stop();
			delete impl;
		}

		void set_engine(const Engine& engine_in) { impl->set_engine(engine_in); }

		void set_children(const HeapVector<const WorkloadInstanceInfo*>& children, HeapVector<DataConnectionInfo>& pending_connections)
		{
			impl->set_children(children, pending_connections);
		}

		void start(float tick_rate_hz) { impl->start(tick_rate_hz); }

		void tick(const TickInfo& tick_info) { impl->tick(tick_info); }

		void stop() { /* placeholder for consistency with SequencedGroup*/ }
	};

#ifdef ROBOTICK_BUILD_CORE_WORKLOAD_TESTS
	ROBOTICK_REGISTER_WORKLOAD(SequencedGroupWorkload)
#endif // #ifdef ROBOTICK_BUILD_CORE_WORKLOAD_TESTS

} // namespace robotick
