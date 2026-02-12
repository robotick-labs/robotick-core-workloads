// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoSceneRegistry.h"

#include "robotick/api.h"

namespace robotick
{
	MuJoCoSceneRegistry& MuJoCoSceneRegistry::get()
	{
		static MuJoCoSceneRegistry registry;
		return registry;
	}

	uint32_t MuJoCoSceneRegistry::register_scene(MuJoCoPhysics* physics)
	{
		ROBOTICK_ASSERT(physics != nullptr);

		LockGuard lock(mutex_);
		// Small fixed registry keeps allocation predictable.
		for (uint32_t i = 0; i < kMaxScenes; ++i)
		{
			SceneEntry& entry = entries_[i];
			if (!entry.active)
			{
				entry.active = true;
				entry.physics = physics;
				return make_handle(i);
			}
		}

		ROBOTICK_FATAL_EXIT("MuJoCoSceneRegistry capacity exceeded (%lu scenes)", static_cast<unsigned long>(kMaxScenes));
		return 0;
	}

	void MuJoCoSceneRegistry::unregister_scene(uint32_t scene_id)
	{
		uint32_t index = 0;
		if (!decode_handle(scene_id, index))
			return;

		LockGuard lock(mutex_);
		SceneEntry& entry = entries_[index];
		if (entry.active)
		{
			entry.active = false;
			entry.physics = nullptr;
		}
	}

	bool MuJoCoSceneRegistry::is_valid(uint32_t scene_id) const
	{
		uint32_t index = 0;
		if (!decode_handle(scene_id, index))
			return false;

		LockGuard lock(mutex_);
		const SceneEntry& entry = entries_[index];
		return entry.active && entry.physics != nullptr;
	}

	const mjModel* MuJoCoSceneRegistry::get_model(uint32_t scene_id) const
	{
		uint32_t index = 0;
		if (!decode_handle(scene_id, index))
			return nullptr;

		LockGuard lock(mutex_);
		const SceneEntry& entry = entries_[index];
		if (!entry.active || entry.physics == nullptr)
			return nullptr;

		// Expose the model pointer for contexts that need immutable access.
		return entry.physics->model();
	}

	bool MuJoCoSceneRegistry::alloc_render_snapshot(uint32_t scene_id, ::mjData*& data_out, const ::mjModel*& model_out, double& time_out) const
	{
		uint32_t index = 0;
		if (!decode_handle(scene_id, index))
			return false;

		LockGuard lock(mutex_);
		const SceneEntry& entry = entries_[index];
		if (!entry.active || entry.physics == nullptr)
			return false;

		// Allocation happens inside MuJoCoPhysics under its own mutex.
		return entry.physics->alloc_render_snapshot(data_out, model_out, time_out);
	}

	bool MuJoCoSceneRegistry::copy_render_snapshot(uint32_t scene_id, ::mjData* dst, const ::mjModel*& model_out, double& time_out) const
	{
		uint32_t index = 0;
		if (!decode_handle(scene_id, index))
			return false;

		LockGuard lock(mutex_);
		const SceneEntry& entry = entries_[index];
		if (!entry.active || entry.physics == nullptr)
			return false;

		return entry.physics->copy_render_snapshot(dst, model_out, time_out);
	}

	void MuJoCoSceneRegistry::destroy_render_snapshot(::mjData*& data_out) const
	{
		// Free through the centralized helper to keep lifetime rules consistent.
		MuJoCoPhysics::destroy_snapshot(data_out);
	}

	uint32_t MuJoCoSceneRegistry::make_handle(uint32_t index)
	{
		return index + 1u;
	}

	bool MuJoCoSceneRegistry::decode_handle(uint32_t handle, uint32_t& index_out)
	{
		if (handle == 0)
			return false;
		index_out = handle - 1u;
		return index_out < kMaxScenes;
	}
} // namespace robotick
