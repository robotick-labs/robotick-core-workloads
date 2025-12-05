// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

#include "robotick/boards/m5/BoardSupport.h"

#if defined(ROBOTICK_PLATFORM_ESP32S3) && defined(ROBOTICK_PLATFORM_ESP32S3_M5)
#include <M5Unified.h>
#define ROBOTICK_IMU_HAS_M5 1
#else
#define ROBOTICK_IMU_HAS_M5 0
#endif

namespace robotick
{

	struct ImuConfig
	{
		bool enable_debug_info = false;
	};

	struct ImuInputs
	{
		// Currently unused, placeholder for future features (e.g. calibration trigger)
	};

	struct ImuOutputs
	{
		Vec3 accel;
		Vec3 gyro;
		Vec3 mag;
	};

	struct ImuWorkload
	{
		ImuConfig config;
		ImuInputs inputs;
		ImuOutputs outputs;

#if ROBOTICK_IMU_HAS_M5
		void setup()
		{
			if (!boards::m5::ensure_initialized())
			{
				ROBOTICK_FATAL_EXIT("ImuWorkload requires M5 support but initialization failed.");
			}

			if (!M5.Imu.isEnabled())
			{
				ROBOTICK_INFO("IMU not enabled — attempting init...");
				if (!M5.Imu.begin())
				{
					ROBOTICK_FATAL_EXIT("IMU begin() failed.");
				}
			}

			if (!M5.Imu.isEnabled())
			{
				ROBOTICK_FATAL_EXIT("IMU still not enabled after init.");
			}
			else
			{
				ROBOTICK_INFO("IMU initialized successfully");
			}
		}

		void tick(const TickInfo& tick_info)
		{
			M5.Imu.update();
			const auto& imu_data = M5.Imu.getImuData();

			outputs.accel.x = imu_data.accel.x;
			outputs.accel.y = imu_data.accel.y;
			outputs.accel.z = imu_data.accel.z;

			outputs.gyro.x = imu_data.gyro.x;
			outputs.gyro.y = imu_data.gyro.y;
			outputs.gyro.z = imu_data.gyro.z;

			outputs.mag.x = imu_data.mag.x;
			outputs.mag.y = imu_data.mag.y;
			outputs.mag.z = imu_data.mag.z;

			if (config.enable_debug_info)
			{
				ROBOTICK_INFO(
					"IMU: accel[%.2f %.2f %.2f] g\tgyro[%.2f %.2f %.2f] °/s\tmag[%.2f %.2f %.2f] µT\t| tick_duration %.2f ms\t| tick_delta %.2f ms",
					outputs.accel.x,
					outputs.accel.y,
					outputs.accel.z,
					outputs.gyro.x,
					outputs.gyro.y,
					outputs.gyro.z,
					outputs.mag.x,
					outputs.mag.y,
					outputs.mag.z,
					tick_info.workload_stats->get_last_tick_duration_ms(),
					tick_info.workload_stats->get_last_time_delta_ms());
			}
		}
#else
		void setup() { ROBOTICK_WARNING("ImuWorkload requires ROBOTICK_PLATFORM_ESP32S3_M5; outputs will remain zero."); }
		void tick(const TickInfo& /*tick_info*/)
		{
			outputs.accel = Vec3();
			outputs.gyro = Vec3();
			outputs.mag = Vec3();
		}
#endif // ROBOTICK_IMU_HAS_M5
	};

} // namespace robotick
