# TODO: MuJoCo headless rendering bring-up (EGL, surfaceless/device)

Goal: make `Unit/Systems/MuJoCoRenderContext` reliably render non-black frames in true headless mode (no SDL windows), and reuse the same path in `MuJoCoCameraWorkload`.

Last run: `./build_linux_debug.sh -t MuJoCoRenderContext` passes (blue background renders and PNG output is non-empty).

Status: `Unit/Systems/MuJoCoRenderContext` now passes after forcing `offsamples=0`, forwarding snapshot data, and fixing the camera z-axis in `blue_background.xml`.

## 0) Ground truth: what we learned ✅
- A “real” GL context exists and works: manual GL rendering can produce expected pixels.
- The remaining failure is specific to MuJoCo’s offscreen/MJR path under our current EGL setup.
- Known working headless pattern exists in the wild: `dm_control` uses `EGL_EXT_platform_device` + surfaceless contexts (no pbuffers, no X/Wayland).

## 1) Switch EGL to “known good” headless mode (dm_control style) ✅
- Implement `EGL_EXT_platform_device` path:
  - `eglQueryDevicesEXT` → `eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, device, nullptr)` → `eglInitialize`.
  - Bind desktop GL: `eglBindAPI(EGL_OPENGL_API)` and create context.
  - Make current surfaceless: `eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context)`.
- Support `MUJOCO_EGL_DEVICE_ID` (or similar) to select a GPU index when multiple devices exist.
- Fallback policy (only if device extension missing):
  - `EGL_PLATFORM_SURFACELESS_MESA` via `eglGetPlatformDisplayEXT` if present.
  - Avoid pbuffers unless proven necessary.

## 2) Make MuJoCo’s GL object lifetime unambiguous
- Ensure the EGL context is current before every MuJoCo GL call that can allocate GL resources:
  - `mjr_makeContext`
  - `mjr_resizeOffscreen`
  - `mjr_setBuffer`
  - `mjr_render`
- Ensure we never “lose” current-ness between init and render.

## 3) Keep the minimal GL sanity test ✅
- Keep the “manual GL clear produces blue pixels” section as a guardrail:
- Render into a known FBO and read back.
- If that succeeds while MuJoCo remains black, focus exclusively on MuJoCo’s FBO selection / context init.

## 4) Validate MuJoCo’s own offscreen requirements (model + buffers)
- Ensure model XML sets offscreen framebuffer large enough for tests/workloads:
  - `<visual><global offwidth="..." offheight="..."/></visual>` ✅ (set in `blue_background.xml`)
- Add explicit checks/logs:
  - `mjr_maxViewport(context_)` after `mjr_resizeOffscreen`
  - `glGetIntegerv(GL_FRAMEBUFFER_BINDING, ...)` + `glCheckFramebufferStatus(...)` immediately after `mjr_setBuffer(mjFB_OFFSCREEN, ...)`
  - `glGetError()` after `mjr_render` and after `mjr_readPixels`
  - FBO binding/status logs around render/read ✅
  - Post-render pixel probe (1x1 `glReadPixels`) ✅
  - Per-step glGetError logging around render/read ✅

## 5) Tighten tests once fixed
- Once the surfaceless/device EGL path works:
- Remove temporary logs / instrumentation. ✅
- Make the test strictly assert: ✅
  - offscreen clear/read returns non-zero pixels ✅
  - MuJoCo render returns a blue-dominant image for `blue_background.xml` ✅
  - PNG output is non-empty and blue-dominant ✅

## 6) Roll the same path into workloads
- After `Unit/Systems/MuJoCoRenderContext` is green in headless mode:
- Ensure `MuJoCoCameraWorkload` uses the same `MuJoCoRenderContext` initialization sequence and produces non-black frames in headless runs.
