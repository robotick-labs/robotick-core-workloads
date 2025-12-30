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
		for (uint32_t i = 0; i < kMaxScenes; ++i)
		{
			SceneEntry& entry = entries_[i];
			if (!entry.active)
			{
				entry.active = true;
				entry.physics = physics;
				entry.generation = (entry.generation == 0) ? 1 : entry.generation + 1;
				return make_handle(i, entry.generation);
			}
		}

		ROBOTICK_FATAL_EXIT("MuJoCoSceneRegistry capacity exceeded (%u scenes)", kMaxScenes);
		return 0;
	}

	void MuJoCoSceneRegistry::unregister_scene(uint32_t scene_id)
	{
		uint32_t index = 0;
		uint32_t generation = 0;
		decode_handle(scene_id, index, generation);

		if (index >= kMaxScenes)
			return;

		LockGuard lock(mutex_);
		SceneEntry& entry = entries_[index];
		if (entry.active && entry.generation == generation)
		{
			entry.active = false;
			entry.physics = nullptr;
		}
	}

	bool MuJoCoSceneRegistry::is_valid(uint32_t scene_id) const
	{
		uint32_t index = 0;
		uint32_t generation = 0;
		decode_handle(scene_id, index, generation);

		if (index >= kMaxScenes)
			return false;

		LockGuard lock(mutex_);
		const SceneEntry& entry = entries_[index];
		return entry.active && entry.generation == generation && entry.physics != nullptr;
	}

	MuJoCoRenderSnapshot MuJoCoSceneRegistry::get_render_snapshot(uint32_t scene_id) const
	{
		uint32_t index = 0;
		uint32_t generation = 0;
		decode_handle(scene_id, index, generation);

		if (index >= kMaxScenes)
			return {};

		LockGuard lock(mutex_);
		const SceneEntry& entry = entries_[index];
		if (!entry.active || entry.generation != generation || entry.physics == nullptr)
			return {};

		return entry.physics->get_render_snapshot();
	}

	uint32_t MuJoCoSceneRegistry::make_handle(uint32_t index, uint32_t generation)
	{
		return (static_cast<uint32_t>(generation) << 16) | (index & 0xffffu);
	}

	void MuJoCoSceneRegistry::decode_handle(uint32_t handle, uint32_t& index_out, uint32_t& generation_out)
	{
		index_out = handle & 0xffffu;
		generation_out = (handle >> 16) & 0xffffu;
	}
} // namespace robotick
