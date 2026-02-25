// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

#include <chrono>

#include "robotick/boards/m5/BoardSupport.h"

#if defined(ROBOTICK_PLATFORM_ESP32S3) && defined(ROBOTICK_PLATFORM_ESP32S3_M5)
#include "driver/i2c.h"
#include <M5Unified.h>
#define ROBOTICK_BASEX_HAS_M5 1
#else
#define ROBOTICK_BASEX_HAS_M5 0
#endif

namespace robotick
{

	constexpr uint8_t BASEX_I2C_ADDR = 0x22;
	constexpr uint8_t BASEX_PWM_DUTY_ADDR = 0x20;

	struct BaseXConfig
	{
		float max_motor_speed = 1.0f;
	};

	struct BaseXInputs
	{
		float motor1_speed = 0.0f;
		float motor2_speed = 0.0f;
		float motor3_speed = 0.0f;
		float motor4_speed = 0.0f;
	};

	struct BaseXOutputs
	{
		float motor1_speed = 0.0f;
		float motor2_speed = 0.0f;
		float motor3_speed = 0.0f;
		float motor4_speed = 0.0f;
	};

	struct BaseXWorkload
	{
		BaseXInputs inputs;
		BaseXOutputs outputs;
		BaseXConfig config;

		~BaseXWorkload()
		{
			inputs.motor1_speed = 0.0f;
			inputs.motor2_speed = 0.0f;
			inputs.motor3_speed = 0.0f;
			inputs.motor4_speed = 0.0f;

			set_motor_speeds();
		}

		void tick(const TickInfo&) { set_motor_speeds(); }

		void set_motor_speeds()
		{
#if ROBOTICK_BASEX_HAS_M5
			if (!boards::m5::ensure_initialized())
			{
				ROBOTICK_WARNING("BaseXWorkload: M5 initialization failed, skipping motor update.");
				return;
			}

			// Create duty values
			uint8_t duties[4] = {
				static_cast<uint8_t>(clamp(inputs.motor1_speed, -config.max_motor_speed, config.max_motor_speed) * 127.0f),
				static_cast<uint8_t>(clamp(inputs.motor2_speed, -config.max_motor_speed, config.max_motor_speed) * 127.0f),
				static_cast<uint8_t>(clamp(inputs.motor3_speed, -config.max_motor_speed, config.max_motor_speed) * 127.0f),
				static_cast<uint8_t>(clamp(inputs.motor4_speed, -config.max_motor_speed, config.max_motor_speed) * 127.0f),
			};

			constexpr uint32_t BASEX_I2C_FREQ = 400000;

			// Begin I2C transaction manually
			m5::In_I2C.writeRegister(BASEX_I2C_ADDR, BASEX_PWM_DUTY_ADDR, duties, sizeof(duties), BASEX_I2C_FREQ);
#else
			static bool warned = false;
			if (!warned)
			{
				ROBOTICK_WARNING("BaseXWorkload requires ROBOTICK_PLATFORM_ESP32S3_M5; outputs are mirrored without hardware control.");
				warned = true;
			}
#endif // ROBOTICK_BASEX_HAS_M5

			outputs.motor1_speed = inputs.motor1_speed;
			outputs.motor2_speed = inputs.motor2_speed;
			outputs.motor3_speed = inputs.motor3_speed;
			outputs.motor4_speed = inputs.motor4_speed;
		}
	};

} // namespace robotick
