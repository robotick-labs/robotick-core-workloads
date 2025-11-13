#if defined(ROBOTICK_PLATFORM_ESP32)

#include "robotick/systems/Renderer.h"
#include <M5Unified.h>
#include <vector>

namespace robotick
{
	static M5Canvas* canvas = nullptr;

	void Renderer::init(bool texture_only)
	{
		ROBOTICK_WARNING_IF(texture_only, "Renderer - texture_only not yet supported on esp32 platforms");

		M5.Lcd.setRotation(3);
		physical_w = 320;
		physical_h = 240;
		canvas = new M5Canvas(&M5.Lcd);
		canvas->createSprite(physical_w, physical_h);
	}

	void Renderer::clear(const Color& color)
	{
		canvas->fillScreen(canvas->color565(color.r, color.g, color.b));
	}

	void Renderer::present()
	{
		canvas->pushSprite(0, 0);
	}

	std::vector<uint8_t> Renderer::capture_as_png()
	{
		ROBOTICK_WARNING("Renderer::capture_as_png() not yet supported on esp32 platforms");
		return {};
	}

	void Renderer::cleanup()
	{
		if (canvas)
		{
			delete canvas;
			canvas = nullptr;
		}
	}

	void Renderer::draw_ellipse_filled(const Vec2& center, const float rx, const float ry, const Color& color)
	{
		canvas->setColor(color.r, color.g, color.b);
		canvas->fillEllipse(to_px_x(center.x), to_px_y(center.y), to_px_w(rx), to_px_h(ry));
	}

	void Renderer::draw_triangle_filled(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Color& color)
	{
		int x0 = to_px_x(p0.x);
		int y0 = to_px_y(p0.y);
		int x1 = to_px_x(p1.x);
		int y1 = to_px_y(p1.y);
		int x2 = to_px_x(p2.x);
		int y2 = to_px_y(p2.y);

		uint32_t c = canvas->color565(color.r, color.g, color.b);
		canvas->fillTriangle(x0, y0, x1, y1, x2, y2, c);
	}

	void Renderer::draw_text(const char* text, const Vec2& pos, const float size, const TextAlign align, const Color& color)
	{
		if (!text || !*text || !canvas)
			return;

		canvas->setTextSize(1);
		canvas->setTextColor(canvas->color565(color.r, color.g, color.b));
		canvas->setTextDatum(align == TextAlign::Center ? middle_center : top_left);
		canvas->drawString(text, to_px_x(pos.x), to_px_y(pos.y));
	}

	// === New: raw RGBA blit, stretched to current viewport ===
	void Renderer::draw_image_rgba8888_fit(const uint8_t* pixels, int w, int h)
	{
		if (!pixels || w <= 0 || h <= 0 || !canvas)
			return;

		// Convert RGBA8888 -> RGB565 and draw scaled to the viewport region
		// NOTE: For now we simply scale to fill the logical viewport using M5 drawScaledSprite-like path.
		// M5Canvas doesn't provide an RGBA path, so convert then push.
		static std::vector<uint16_t> rgb565; // desktop-only STL is avoided in core; here it's an ESP-side UI path
		rgb565.resize(static_cast<size_t>(w) * static_cast<size_t>(h));

		const uint8_t* src = pixels;
		for (int i = 0, n = w * h; i < n; ++i)
		{
			const uint8_t r = src[0];
			const uint8_t g = src[1];
			const uint8_t b = src[2];
			// ignore alpha
			src += 4;

			const uint16_t r5 = static_cast<uint16_t>(r >> 3);
			const uint16_t g6 = static_cast<uint16_t>(g >> 2);
			const uint16_t b5 = static_cast<uint16_t>(b >> 3);
			rgb565[static_cast<size_t>(i)] = (r5 << 11) | (g6 << 5) | (b5);
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
			canvas->pushImage(dst_x, dst_y, w, h, rgb565.data());
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
					canvas->drawPixel(dst_x + x, dst_y + y, row[sx]);
				}
			}
		}
	}
} // namespace robotick

#endif // #if defined(ROBOTICK_PLATFORM_ESP32)
