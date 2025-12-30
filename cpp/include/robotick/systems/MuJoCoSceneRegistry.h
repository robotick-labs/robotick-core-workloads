// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/concurrency/Sync.h"
#include "robotick/systems/MuJoCoPhysics.h"

#include <cstdint>

namespace robotick
{
	class MuJoCoSceneRegistry
	{
	  public:
		// Process-local singleton for mapping scene handles to MuJoCoPhysics instances.
		static MuJoCoSceneRegistry& get();

		// Register a physics scene and receive an opaque handle for render snapshots.
		uint32_t register_scene(MuJoCoPhysics* physics);
		// Unregister a scene handle; safe to call multiple times.
		void unregister_scene(uint32_t scene_id);

		// Check whether a handle is still valid.
		bool is_valid(uint32_t scene_id) const;
		// Fetch a render snapshot for a valid handle; returns empty snapshot if invalid.
		MuJoCoRenderSnapshot get_render_snapshot(uint32_t scene_id) const;

	  private:
		struct SceneEntry
		{
			// Pointer owned elsewhere; registry does not manage lifetime.
			MuJoCoPhysics* physics = nullptr;
			// Monotonic generation to invalidate stale handles.
			uint32_t generation = 0;
			bool active = false;
		};

		// Fixed-size registry to keep allocation simple and deterministic.
		static constexpr uint32_t kMaxScenes = 32;

		// Protects all registry operations and entry access.
		mutable Mutex mutex_;
		SceneEntry entries_[kMaxScenes]{};

		// Pack/unpack handle as {generation|index}.
		static uint32_t make_handle(uint32_t index, uint32_t generation);
		static void decode_handle(uint32_t handle, uint32_t& index_out, uint32_t& generation_out);
	};
} // namespace robotick
