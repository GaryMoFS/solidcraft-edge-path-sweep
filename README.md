# Edge Path & Sweep (`edge-path-sweep`)

A WebAssembly (`craft:plugin@0.2.0` / `commands@0.2`) plugin for **SolidCraft (v0.47+)** (use v0.2.0 for SolidCraft 0.44-0.46) that builds an open or closed 3D sketch path from picked body edges and optionally sweeps a picked sketch profile or built-in shape along that path.

## Features

- **Unified dialog (`edge-path-sweep`):** Exposed as **Edge Path & Sweep...** in the menu and **Edge Path** on the **Plugins** ribbon tab, ordered cleanly via `x-solidcraft-order`.
- **Native viewport & Browser picking (`x-solidcraft-selection`):**
  - **Edges (`accept: ["edge"]`, required):** Pick one or more body edges directly in the viewport (open the dialog first or pre-select edges); edges are automatically chained in double-ended order (`open` or `closed` loop), including full-circle and cylindrical-arc midpoint disambiguation.
  - **Sweep Profile (`accept: ["profile"]`, optional):** Pick a sketch profile in the viewport or click a Sketch in the Browser, or specify a fallback sketch name/ID in `profile_sketch`. Leave empty to create the 3D path only.
- **Built-in quick profiles:** Choose `circle` or `rectangle` (`profile_width`, `profile_height`) when no sketch profile is picked.
- **Sweep plane & options:** Optional perpendicular sweep plane at path start (`construct.plane.along_path`), profile X/Y offset (`offset_x`, `offset_y`), profile rotation (`profile_angle_deg`), orientation (`perpendicular` / `parallel`), `twist_deg`, `end_scale`, and boolean operation (`new_body`, `join`, `cut`, `intersect`).
- **Repeated runs:** Checks `document.inspect` timeline names so repeated invocations automatically pick unique feature names instead of colliding.

## Building & Packaging

Requires Rust with the `wasm32-unknown-unknown` target (on Windows with MinGW/GNU toolchain, set `$env:RUSTUP_TOOLCHAIN = "stable-x86_64-pc-windows-gnu"`):

```sh
rustup target add wasm32-unknown-unknown
cd plugin && cargo build --release
cd ../packager && cargo run --release
```

Run the packager integration tests (uses `SolidCraft-r47-trial` crates in `[dev-dependencies]`):

```sh
cd packager && cargo test --release -- --nocapture
```

Packaging outputs `dist/edge-path-sweep.zip` and `dist/edge-path-sweep/`, which can be installed in SolidCraft via **Plugins > Install plugin...**.
