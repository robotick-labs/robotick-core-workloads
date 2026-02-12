## CanvasWorkload

Procedural 2D renderer that loads a declarative “canvas scene” file, exposes
its controllable properties as a dynamic blackboard, and renders the result to
either a PNG buffer or an on-screen window. The workload contains **no**
expressive logic; higher-level workloads (e.g. `FaceControlWorkload`) decide how
to drive the exposed controls.

### Config

| Field | Type | Description |
| --- | --- | --- |
| `scene_path` | `FixedString256` | Path to a `.canvas.yaml` scene file (required). |
| `render_to_texture` | `bool` | `true` → capture PNG into the output; `false` → present via `Renderer`. |

Size policy: the scene file’s `canvas.logical_size` and `canvas.output_size`
define the logical viewport and physical render target dimensions. The config
currently does not override those values.

### Inputs

- `controls` (`Blackboard`): dynamically generated field schema derived from
  the scene file. Each control entry in the scene maps a node property to a flat
  alias (e.g. `left_eye_translate`), and CanvasWorkload adds matching fields to
  this blackboard. Missing runtime values fall back to the node’s default; unknown
  values are ignored.

### Outputs

- `face_png_data` (`ImagePng128k`): when `render_to_texture=true`, contains the
  rendered PNG bytes; when `false`, the buffer size is zero and the renderer
  presents to the screen directly.

### Scene File Summary (`*.canvas.yaml`)

```yaml
canvas:
  logical_size: { width: 320, height: 240 }   # coordinate space for transforms
  output_size: { width: 800, height: 480 }    # rendered texture resolution
  background: { r: 255, g: 255, b: 255, a: 255 }

scene:
  type: group                                # root node
  id: root
  translate: { x: 0, y: 0 }
  rotate_deg: 0
  children:
    - type: group                            # left eye group
      id: left_eye
      translate: { x: 60, y: 120 }
      rotate_deg: 0
      children:
        - type: ellipse
          id: left_eye_blob
          translate: { x: 0, y: 0 }
          rotate_deg: 0
          style: { fill: { r: 0, g: 0, b: 0, a: 255 } }
          geometry: { rx: 40, ry: 65 }
          children:
            - type: ellipse                  # sparkle inherits transforms
              id: left_eye_sparkle
              translate: { x: 10, y: -22 }
              rotate_deg: 0
              style: { fill: { r: 255, g: 255, b: 255, a: 255 } }
              geometry: { rx: 13, ry: 16 }

controls:
  - target: left_eye.translate               # property path on a node
    alias: left_eye_translate                # blackboard field name
  - target: left_eye.scale
    alias: left_eye_scale
```

Supported node types (v1):

- `group`: transform-only container (no rendering).
- `ellipse`: draws a filled ellipse (`geometry: { rx, ry }`).
- `rect`: draws a filled rectangle (`geometry: { width, height }`).

Common node fields:

| Field | Notes |
| --- | --- |
| `id` | Required, unique within the scene. |
| `translate` | `{ x, y }` in logical coordinates (required). |
| `rotate_deg` | Clockwise degrees (required). |
| `scale` | `{ x, y }`, defaults to `{1,1}` when omitted. |
| `style.fill` | RGBA bytes; default opaque white. |
| `children` | Optional list; transforms inherit parent matrices. |

Controls map node properties to aliases. Supported `target` paths in v1:

- `translate` or `translate.x` / `translate.y`
- `scale` or `scale.x` / `scale.y`
- `rotate_deg`
- `visible`
- `alpha`

Validation performed during load:

- Node ids must be unique and the hierarchy acyclic.
- Geometry parameters (`rx`, `ry`, `width`, `height`) must be positive.
- Each control must reference an existing node/property and use a unique alias.

### Usage Notes

1. Configure the workload with the scene file and render mode.
2. Connect upstream workloads to the generated control blackboard fields.
3. CanvasWorkload applies the control values each tick and renders the updated
   scene; expressive behavior such as blinking or look offsets remains entirely
   upstream, keeping the renderer deterministic and portable.
