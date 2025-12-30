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
	// thread-safe render snapshots. MuJoCoWorkload runs physics via this class,
	// then registers the instance with MuJoCoSceneRegistry so camera workloads
	// can request snapshots via a scene_id handle.

	struct MuJoCoRenderSnapshot
	{
		const ::mjModel* model = nullptr;
		::mjData* data = nullptr;
		double time = 0.0;
	};

	class MuJoCoPhysics
	{
	  public:
		MuJoCoPhysics() = default;
		~MuJoCoPhysics();

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
		::mjData* data() const { return data_; }

		// Thread-safe copy of mjData for rendering; caller must free via free_render_snapshot().
		MuJoCoRenderSnapshot get_render_snapshot() const;
		void free_render_snapshot(MuJoCoRenderSnapshot& snapshot) const;

	  private:
		// Guards model/data access and snapshot creation.
		mutable Mutex mutex_;
		::mjModel* model_ = nullptr;
		::mjData* data_ = nullptr;
	};
} // namespace robotick
