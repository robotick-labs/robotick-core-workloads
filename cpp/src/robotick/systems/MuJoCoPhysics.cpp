// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoPhysics.h"

#include "robotick/api.h"

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

		unload();

		char error[512] = {0};
		mjModel* model = mj_loadXML(model_path, nullptr, error, sizeof(error));
		if (!model)
		{
			ROBOTICK_WARNING("MuJoCoPhysics::load_from_xml failed: %s", error);
			return false;
		}

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

	MuJoCoRenderSnapshot MuJoCoPhysics::get_render_snapshot() const
	{
		LockGuard lock(mutex_);
		if (!model_ || !data_)
			return {};

		MuJoCoRenderSnapshot snapshot;
		snapshot.model = model_;
		snapshot.data = mj_makeData(model_);
		if (snapshot.data)
		{
			mj_copyData(snapshot.data, model_, data_);
			snapshot.time = data_->time;
		}
		return snapshot;
	}

	void MuJoCoPhysics::free_render_snapshot(MuJoCoRenderSnapshot& snapshot) const
	{
		if (snapshot.data)
		{
			mj_deleteData(snapshot.data);
			snapshot.data = nullptr;
		}
		snapshot.model = nullptr;
		snapshot.time = 0.0;
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

	MuJoCoRenderSnapshot MuJoCoPhysics::get_render_snapshot() const
	{
		return {};
	}

	void MuJoCoPhysics::free_render_snapshot(MuJoCoRenderSnapshot&)
	{
	}
} // namespace robotick

#endif
