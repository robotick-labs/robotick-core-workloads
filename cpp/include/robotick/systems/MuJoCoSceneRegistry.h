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
		// Fetch the model pointer for a valid handle; returns nullptr if invalid.
		const ::mjModel* get_model(uint32_t scene_id) const;
		// Allocate a render snapshot; returns false on invalid handle or alloc failure.
		bool alloc_render_snapshot(uint32_t scene_id, ::mjData*& data_out, const ::mjModel*& model_out, double& time_out) const;
		// Copy into a caller-owned mjData buffer; returns false on invalid handle.
		bool copy_render_snapshot(uint32_t scene_id, ::mjData* dst, const ::mjModel*& model_out, double& time_out) const;
		// Release a snapshot obtained from alloc_render_snapshot().
		void destroy_render_snapshot(::mjData*& data_out) const;

	  private:
		struct SceneEntry
		{
			// Pointer owned elsewhere; registry does not manage lifetime.
			MuJoCoPhysics* physics = nullptr;
			bool active = false;
		};

		// Fixed-size registry to keep allocation simple and deterministic.
		static constexpr uint32_t kMaxScenes = 32;

		// Protects all registry operations and entry access.
		mutable Mutex mutex_;
		SceneEntry entries_[kMaxScenes]{};

		// Handle encoding is a 1-based index into entries_.
		static uint32_t make_handle(uint32_t index);
		static bool decode_handle(uint32_t handle, uint32_t& index_out);
	};
} // namespace robotick
