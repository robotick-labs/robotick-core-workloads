// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoPhysics.h"

#include "robotick/api.h"
#include "robotick/systems/MuJoCoCallbacks.h"

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include <mujoco/mujoco.h>

namespace robotick
{
	MuJoCoPhysics::~MuJoCoPhysics()
	{
		unload();
	}

	bool MuJoCoPhysics::load_from_xml(const char* model_path)
	{
		if (!model_path || model_path[0] == '\0')
			return false;

		mujoco_callbacks::install();

		unload();

		char error[512] = {0};
		// MuJoCo allocates the model; we own and free it on unload().
		mjModel* model = mj_loadXML(model_path, nullptr, error, sizeof(error));
		if (!model)
		{
			ROBOTICK_WARNING("MuJoCoPhysics::load_from_xml failed: %s", error);
			return false;
		}

		// Pre-allocate the primary simulation state buffer.
		mjData* data = mj_makeData(model);
		if (!data)
		{
			mj_deleteModel(model);
			return false;
		}

		LockGuard lock(mutex_);
		model_ = model;
		data_ = data;
		return true;
	}

	void MuJoCoPhysics::unload()
	{
		LockGuard lock(mutex_);
		if (data_)
		{
			mj_deleteData(data_);
			data_ = nullptr;
		}
		if (model_)
		{
			mj_deleteModel(model_);
			model_ = nullptr;
		}
	}

	void MuJoCoPhysics::forward()
	{
		LockGuard lock(mutex_);
		if (model_ && data_)
			mj_forward(model_, data_);
	}

	void MuJoCoPhysics::step()
	{
		LockGuard lock(mutex_);
		if (model_ && data_)
			mj_step(model_, data_);
	}

	bool MuJoCoPhysics::alloc_render_snapshot(::mjData*& data_out, const ::mjModel*& model_out, double& time_out) const
	{
		// Allocate a scratch mjData and copy the current sim state into it.
		LockGuard lock(mutex_);
		if (!model_ || !data_)
		{
			data_out = nullptr;
			model_out = nullptr;
			time_out = 0.0;
			return false;
		}

		data_out = mj_makeData(model_);
		if (!data_out)
		{
			model_out = nullptr;
			time_out = 0.0;
			return false;
		}

		mj_copyData(data_out, model_, data_);
		mj_forward(model_, data_out);
		model_out = model_;
		time_out = data_->time;
		return true;
	}

	void MuJoCoPhysics::destroy_render_snapshot(::mjData*& data_out) const
	{
		// Convenience wrapper for instance callers.
		destroy_snapshot(data_out);
	}

	bool MuJoCoPhysics::copy_render_snapshot(::mjData* dst, const ::mjModel*& model_out, double& time_out) const
	{
		if (!dst)
			return false;

		LockGuard lock(mutex_);
		if (!model_ || !data_)
			return false;

		// Reset the destination stack before copying; required for reuse across frames.
		mj_resetData(model_, dst);
		// Copy the live simulation state into a pre-allocated buffer.
		mj_copyData(dst, model_, data_);
		model_out = model_;
		time_out = data_->time;
		return true;
	}

	void MuJoCoPhysics::destroy_snapshot(::mjData*& data_out)
	{
		// This is intentionally standalone so other systems can free snapshots.
		if (data_out)
		{
			mj_deleteData(data_out);
			data_out = nullptr;
		}
	}
} // namespace robotick

#else

namespace robotick
{
	MuJoCoPhysics::~MuJoCoPhysics() = default;

	bool MuJoCoPhysics::load_from_xml(const char*)
	{
		return false;
	}

	void MuJoCoPhysics::unload()
	{
	}

	void MuJoCoPhysics::forward()
	{
	}

	void MuJoCoPhysics::step()
	{
	}

	bool MuJoCoPhysics::alloc_render_snapshot(::mjData*&, const ::mjModel*&, double&) const
	{
		return false;
	}

	void MuJoCoPhysics::destroy_render_snapshot(::mjData*&) const
	{
	}

	bool MuJoCoPhysics::copy_render_snapshot(::mjData*, const ::mjModel*&, double&) const
	{
		return false;
	}
} // namespace robotick

#endif
