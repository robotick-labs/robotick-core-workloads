// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/concurrency/Sync.h"

// Forward declarations to avoid requiring MuJoCo headers in all translation units.
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

namespace robotick
{
	// MuJoCoPhysics owns the per-scene mjModel/mjData lifecycle and provides
	// thread-safe render snapshots. MuJoCoPhysicsWorkload runs physics via this class,
	// then registers the instance with MuJoCoSceneRegistry so camera workloads
	// can request snapshots via a scene_id handle.

	class MuJoCoPhysics
	{
	  public:
		MuJoCoPhysics() = default;
		~MuJoCoPhysics();
		MuJoCoPhysics(const MuJoCoPhysics&) = delete;
		MuJoCoPhysics& operator=(const MuJoCoPhysics&) = delete;
		MuJoCoPhysics(MuJoCoPhysics&&) = delete;
		MuJoCoPhysics& operator=(MuJoCoPhysics&&) = delete;

		// Load model/data from an MJCF XML file path; returns false on failure.
		bool load_from_xml(const char* model_path);
		void unload();
		bool is_loaded() const { return model_ != nullptr && data_ != nullptr; }

		// Advance internal derived quantities without stepping time.
		void forward();
		// Step physics by the model timestep.
		void step();

		const ::mjModel* model() const { return model_; }
		::mjModel* model_mutable() { return model_; }
		const ::mjData* data() const { return data_; }
		::mjData* data_mutable() { return data_; }

		// Thread-safe copy of mjData for rendering; caller must free via destroy_render_snapshot().
		bool alloc_render_snapshot(::mjData*& data_out, const ::mjModel*& model_out, double& time_out) const;
		// Frees a snapshot allocated by alloc_render_snapshot(). Safe to call with nullptr.
		void destroy_render_snapshot(::mjData*& data_out) const;
		// Thread-safe copy into a caller-owned mjData buffer; no allocation.
		bool copy_render_snapshot(::mjData* dst, const ::mjModel*& model_out, double& time_out) const;

		// Static helper for freeing mjData without requiring an instance.
		static void destroy_snapshot(::mjData*& data_out);

		// Acquire the internal MuJoCo lock for safe external access.
		UniqueLock lock() const { return UniqueLock(mutex_); }

	  private:
		// Guards model/data access and snapshot creation.
		mutable Mutex mutex_;
		::mjModel* model_ = nullptr;
		::mjData* data_ = nullptr;
	};
} // namespace robotick
