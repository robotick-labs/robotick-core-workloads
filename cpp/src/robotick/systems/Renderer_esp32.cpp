// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_ESP32)

#include "robotick/framework/containers/HeapVector.h"
#include "robotick/systems/Renderer.h"
#include <M5Unified.h>

namespace robotick
{
	struct Renderer::RendererImpl
	{
		M5Canvas* canvas = nullptr;
		HeapVector<uint16_t> rgb565_buffer;
		size_t rgb565_capacity = 0;

		~RendererImpl()
		{
			if (canvas)
			{
				delete canvas;
				canvas = nullptr;
			}
		}

		void ensure_capacity(size_t required_pixels)
		{
			if (rgb565_capacity >= required_pixels && rgb565_buffer.size() >= required_pixels)
				return;
			rgb565_buffer.deinitialize();
			rgb565_buffer.initialize(required_pixels);
			rgb565_capacity = required_pixels;
		}
	};

	void Renderer::init(bool texture_only)
	{
		ROBOTICK_WARNING_IF(texture_only, "Renderer - texture_only not yet supported on esp32 platforms");

		if (initialized)
			return;

		if (!impl)
			impl = new RendererImpl();

		M5.Lcd.setRotation(3);
		physical_w = 320;
		physical_h = 240;
		impl->canvas = new M5Canvas(&M5.Lcd);
		impl->canvas->createSprite(physical_w, physical_h);
		initialized = true;
	}

	void Renderer::clear(const Color& color)
	{
		if (!impl || !impl->canvas)
			return;
		impl->canvas->fillScreen(impl->canvas->color565(color.r, color.g, color.b));
	}

	void Renderer::present()
	{
		if (!impl || !impl->canvas)
			return;
		impl->canvas->pushSprite(0, 0);
	}

	bool Renderer::capture_as_png(uint8_t* dst, size_t capacity, size_t& out_size)
	{
		(void)dst;
		(void)capacity;
		out_size = 0;
		ROBOTICK_WARNING("Renderer::capture_as_png() not yet supported on esp32 platforms");
		return false;
	}

	void Renderer::cleanup()
	{
		if (!impl)
			return;

		delete impl;
		impl = nullptr;
		initialized = false;
	}

	void Renderer::draw_ellipse_filled(const Vec2& center, const float rx, const float ry, const Color& color)
	{
		if (!impl || !impl->canvas)
			return;
		impl->canvas->setColor(color.r, color.g, color.b);
		impl->canvas->fillEllipse(to_px_x(center.x), to_px_y(center.y), to_px_w(rx), to_px_h(ry));
	}

	void Renderer::draw_triangle_filled(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Color& color)
	{
		if (!impl || !impl->canvas)
			return;
		int x0 = to_px_x(p0.x);
		int y0 = to_px_y(p0.y);
		int x1 = to_px_x(p1.x);
		int y1 = to_px_y(p1.y);
		int x2 = to_px_x(p2.x);
		int y2 = to_px_y(p2.y);

		uint32_t c = impl->canvas->color565(color.r, color.g, color.b);
		impl->canvas->fillTriangle(x0, y0, x1, y1, x2, y2, c);
	}

	void Renderer::draw_text(const char* text, const Vec2& pos, const float size, const TextAlign align, const Color& color)
	{
		if (!text || !*text || !impl || !impl->canvas)
			return;

		impl->canvas->setTextSize(1);
		impl->canvas->setTextColor(impl->canvas->color565(color.r, color.g, color.b));
		impl->canvas->setTextDatum(align == TextAlign::Center ? middle_center : top_left);
		impl->canvas->drawString(text, to_px_x(pos.x), to_px_y(pos.y));
	}

	// === New: raw RGBA blit, stretched to current viewport ===
	void Renderer::draw_image_rgba8888_fit(const uint8_t* pixels, int w, int h)
	{
		if (!pixels || w <= 0 || h <= 0 || !impl || !impl->canvas)
			return;

		// Convert RGBA8888 -> RGB565 and draw scaled to the viewport region
		// NOTE: For now we simply scale to fill the logical viewport using M5 drawScaledSprite-like path.
		// M5Canvas doesn't provide an RGBA path, so convert then push.
		const size_t pixel_count = static_cast<size_t>(w) * static_cast<size_t>(h);
		impl->ensure_capacity(pixel_count);
		uint16_t* rgb565 = impl->rgb565_buffer.data();
		if (!rgb565)
			return;

		const uint8_t* src = pixels;
		for (int i = 0; i < w * h; ++i)
		{
			const uint8_t r = src[0];
			const uint8_t g = src[1];
			const uint8_t b = src[2];
			// ignore alpha
			src += 4;

			const uint16_t r5 = static_cast<uint16_t>(r >> 3);
			const uint16_t g6 = static_cast<uint16_t>(g >> 2);
			const uint16_t b5 = static_cast<uint16_t>(b >> 3);
			rgb565[static_cast<size_t>(i)] = static_cast<uint16_t>((r5 << 11) | (g6 << 5) | b5);
		}

		// Destination rect is the current logical viewport region in pixels
		const int dst_x = offset_x;
		const int dst_y = offset_y;
		const int dst_w = static_cast<int>(logical_w * scale);
		const int dst_h = static_cast<int>(logical_h * scale);

		// Use pushImage with automatic scaling via startWrite/writePixels if available.
		// As a simple compatible path, draw at 1:1 when sizes match; otherwise use M5's pushImage disable scaling (drawImage does not scale),
		// so we manually scale via drawPixel sampling (nearest). Kept simple for now.
		if (dst_w == w && dst_h == h)
		{
			impl->canvas->pushImage(dst_x, dst_y, w, h, rgb565);
		}
		else
		{
			// Nearest-neighbour manual scale
			for (int y = 0; y < dst_h; ++y)
			{
				const int sy = (y * h) / dst_h;
				const uint16_t* row = &rgb565[static_cast<size_t>(sy) * static_cast<size_t>(w)];
				for (int x = 0; x < dst_w; ++x)
				{
					const int sx = (x * w) / dst_w;
					impl->canvas->drawPixel(dst_x + x, dst_y + y, row[sx]);
				}
			}
		}
	}
} // namespace robotick

#endif // #if defined(ROBOTICK_PLATFORM_ESP32)
