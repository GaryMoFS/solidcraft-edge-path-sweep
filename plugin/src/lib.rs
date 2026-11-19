#![no_std]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

extern crate alloc;

use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::f64::consts::PI;
use serde::Deserialize;
use serde_json::{Value, json};

#[global_allocator]
static ALLOCATOR: dlmalloc::GlobalDlmalloc = dlmalloc::GlobalDlmalloc;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}

wit_bindgen::generate!({
    path: "../wit/plugin.wit",
    world: "command-plugin",
});

use craft::plugin::commands;
use craft::plugin::control;
use craft::plugin::log::{self, Level};
use craft::plugin::types::{FieldFailure, PluginError};
use exports::craft::plugin::entry::Guest;

const EPS: f64 = 1e-6;

struct EdgePathSweepPlugin;

export!(EdgePathSweepPlugin);

impl Guest for EdgePathSweepPlugin {
    fn execute(entry: String, params: String) -> Result<String, PluginError> {
        let value: Value = serde_json::from_str(&params).map_err(|e| invalid("params", format!("invalid JSON: {e}")))?;
        match entry.as_str() {
            "edge-path-sweep" => run_edge_path_sweep(&value),
            _ => Err(invalid("entry", format!("unknown entry `{entry}`"))),
        }
    }
}

fn invalid(field: &str, reason: impl Into<String>) -> PluginError {
    PluginError::InvalidInput(FieldFailure {
        field: field.to_string(),
        reason: reason.into(),
    })
}

fn run_cmd(cmd: &str, args: Value) -> Result<Value, PluginError> {
    if control::cancelled() {
        return Err(PluginError::Cancelled);
    }
    let raw = commands::run(cmd, &args.to_string())?;
    serde_json::from_str(&raw).map_err(|e| PluginError::Trap(format!("invalid JSON from {cmd}: {e}")))
}

#[derive(Deserialize)]
#[serde(default)]
struct EdgePathSweepParams {
    edges: Vec<Value>,
    path_mode: String,
    create_sweep_plane: bool,
    finish_sketch: bool,
    sketch_name: String,
    profile: Vec<Value>,
    profile_shape: String,
    profile_width: f64,
    profile_height: f64,
    profile_sketch: String,
    offset_x: f64,
    offset_y: f64,
    profile_angle_deg: f64,
    orientation: String,
    twist_deg: f64,
    end_scale: f64,
    sweep_operation: String,
}

impl Default for EdgePathSweepParams {
    fn default() -> Self {
        Self {
            edges: Vec::new(),
            path_mode: "open".to_string(),
            create_sweep_plane: true,
            finish_sketch: true,
            sketch_name: "Edge Path".to_string(),
            profile: Vec::new(),
            profile_shape: "none".to_string(),
            profile_width: 5.0,
            profile_height: 5.0,
            profile_sketch: String::new(),
            offset_x: 0.0,
            offset_y: 0.0,
            profile_angle_deg: 0.0,
            orientation: "perpendicular".to_string(),
            twist_deg: 0.0,
            end_scale: 1.0,
            sweep_operation: "new_body".to_string(),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PathMode {
    Open,
    Closed,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ProfileShape {
    None,
    Circle,
    Rectangle,
}

enum SweepProfile {
    PickedProfiles { sketch: u64, indices: Vec<usize> },
    SketchName(String),
    BuiltInCircle { diameter: f64 },
    BuiltInRect { width: f64, height: f64 },
}

fn parse_path_mode(s: &str) -> Result<PathMode, PluginError> {
    match s.trim() {
        "open" | "" => Ok(PathMode::Open),
        "closed" => Ok(PathMode::Closed),
        _ => Err(invalid("path_mode", "must be \"open\" or \"closed\"")),
    }
}

fn parse_profile_shape(s: &str) -> Result<ProfileShape, PluginError> {
    match s.trim() {
        "none" | "" => Ok(ProfileShape::None),
        "circle" => Ok(ProfileShape::Circle),
        "rectangle" => Ok(ProfileShape::Rectangle),
        _ => Err(invalid("profile_shape", "must be \"none\", \"circle\", or \"rectangle\"")),
    }
}

fn parse_orientation(s: &str) -> Result<&'static str, PluginError> {
    match s.trim() {
        "perpendicular" | "" => Ok("perpendicular"),
        "parallel" => Ok("parallel"),
        _ => Err(invalid("orientation", "must be \"perpendicular\" or \"parallel\"")),
    }
}

fn parse_sweep_operation(s: &str) -> Result<&'static str, PluginError> {
    match s.trim() {
        "new_body" | "" => Ok("new_body"),
        "join" => Ok("join"),
        "cut" => Ok("cut"),
        "intersect" => Ok("intersect"),
        _ => Err(invalid("sweep_operation", "must be \"new_body\", \"join\", \"cut\", or \"intersect\"")),
    }
}

fn check_finite(field: &str, v: f64) -> Result<f64, PluginError> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(invalid(field, "must be a finite number"))
    }
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = sub3(a, b);
    libm::sqrt(dot3(d, d))
}

fn norm3(a: [f64; 3]) -> Option<[f64; 3]> {
    let len = libm::sqrt(dot3(a, a));
    if len.is_finite() && len > 1e-9 {
        Some([a[0] / len, a[1] / len, a[2] / len])
    } else {
        None
    }
}

/// Rotates vector `v` around unit `axis` by `angle` radians (Rodrigues' formula).
fn rotate_around(v: [f64; 3], axis: [f64; 3], angle: f64) -> [f64; 3] {
    let c = libm::cos(angle);
    let s = libm::sin(angle);
    let k_cross_v = cross3(axis, v);
    let k_dot_v = dot3(axis, v);
    add3(
        add3(scale3(v, c), scale3(k_cross_v, s)),
        scale3(axis, k_dot_v * (1.0 - c)),
    )
}

fn read_vec3(v: Option<&Value>) -> Option<[f64; 3]> {
    let arr = v?.as_array()?;
    if arr.len() != 3 {
        return None;
    }
    let x = arr[0].as_f64()?;
    let y = arr[1].as_f64()?;
    let z = arr[2].as_f64()?;
    if x.is_finite() && y.is_finite() && z.is_finite() {
        Some([x, y, z])
    } else {
        None
    }
}

#[derive(Clone)]
struct SelectedEdge {
    orig_index: usize,
    body: Option<String>,
    point: Option<[f64; 3]>,
    curve: String,
    start: Option<[f64; 3]>,
    end: Option<[f64; 3]>,
    center: Option<[f64; 3]>,
    axis: Option<[f64; 3]>,
    radius: Option<f64>,
    length_mm: Option<f64>,
    reversed: bool,
}

impl SelectedEdge {
    fn eff_start(&self) -> Option<[f64; 3]> {
        if self.reversed { self.end } else { self.start }
    }
    fn eff_end(&self) -> Option<[f64; 3]> {
        if self.reversed { self.start } else { self.end }
    }
}

fn raw_selection_items() -> Result<Vec<Value>, PluginError> {
    let raw = run_cmd("selection.get", json!({ "detail": false }))?;
    let mut out = Vec::new();
    if let Some(items) = raw.get("items").and_then(Value::as_array) {
        for item in items {
            if let Some(obj) = item.as_object() {
                let mut clean = obj.clone();
                clean.remove("kind");
                out.push(Value::Object(clean));
            }
        }
    }
    Ok(out)
}

fn extract_edges(sel: &Value) -> Vec<SelectedEdge> {
    let mut edges = Vec::new();
    if let Some(items) = sel.get("items").and_then(Value::as_array) {
        for item in items {
            let kind = item
                .get("kind")
                .or_else(|| item.get("type"))
                .and_then(Value::as_str)
                .unwrap_or("");
            if kind != "edge" {
                continue;
            }
            let body = item.get("body").and_then(Value::as_str).map(str::to_string);
            let point = read_vec3(item.get("point"));
            let curve = item
                .get("curve")
                .and_then(Value::as_str)
                .unwrap_or("other")
                .to_string();
            let start = read_vec3(item.get("start"));
            let end = read_vec3(item.get("end"));
            let center = read_vec3(item.get("center"));
            let axis = read_vec3(item.get("axis")).and_then(norm3);
            let radius = item.get("radius").and_then(Value::as_f64).filter(|r| r.is_finite() && *r > 0.0);
            let length_mm = item.get("length_mm").and_then(Value::as_f64).filter(|l| l.is_finite() && *l > 0.0);
            edges.push(SelectedEdge {
                orig_index: edges.len(),
                body,
                point,
                curve,
                start,
                end,
                center,
                axis,
                radius,
                length_mm,
                reversed: false,
            });
        }
    }
    edges
}

fn resolve_edges(edge_refs: &[Value]) -> Result<Vec<SelectedEdge>, PluginError> {
    let sel = if !edge_refs.is_empty() {
        let restore_items = raw_selection_items()?;
        let set_res = run_cmd("select.set", json!({ "items": edge_refs }));
        if let Err(e) = set_res {
            let _ = run_cmd("select.set", json!({ "items": restore_items }));
            return Err(e);
        }
        let detail = run_cmd("selection.get", json!({ "detail": true }));
        let _ = run_cmd("select.set", json!({ "items": restore_items }));
        detail?
    } else {
        run_cmd("selection.get", json!({ "detail": true }))?
    };

    let edges = extract_edges(&sel);
    if edges.is_empty() {
        return Err(invalid("edges", "pick one or more body edges before running"));
    }
    Ok(edges)
}

#[derive(Clone, Copy)]
enum AttachSide {
    AppendFwd,
    AppendRev,
    PrependFwd,
    PrependRev,
}

/// Orders edges into a head-to-tail chain, growing from both ends so out-of-order picks
/// and reversed first edges chain seamlessly without jumping across gaps.
fn order_edges(mut edges: Vec<SelectedEdge>) -> Vec<SelectedEdge> {
    if edges.len() <= 1 {
        return edges;
    }
    let mut chain: VecDeque<SelectedEdge> = VecDeque::with_capacity(edges.len());
    chain.push_back(edges.remove(0));

    while !edges.is_empty() {
        let head_pt = chain.front().and_then(SelectedEdge::eff_start);
        let tail_pt = chain.back().and_then(SelectedEdge::eff_end);
        if head_pt.is_none() && tail_pt.is_none() {
            chain.extend(edges);
            break;
        }

        let mut best_idx = 0usize;
        let mut best_side = AttachSide::AppendFwd;
        let mut best_dist = f64::MAX;

        for (i, e) in edges.iter().enumerate() {
            if let Some(tail) = tail_pt {
                if let Some(s) = e.start {
                    let d = dist3(tail, s);
                    if d < best_dist {
                        best_dist = d;
                        best_idx = i;
                        best_side = AttachSide::AppendFwd;
                    }
                }
                if let Some(en) = e.end {
                    let d = dist3(tail, en);
                    if d < best_dist {
                        best_dist = d;
                        best_idx = i;
                        best_side = AttachSide::AppendRev;
                    }
                }
            }
            if let Some(head) = head_pt {
                if let Some(en) = e.end {
                    let d = dist3(head, en);
                    if d < best_dist {
                        best_dist = d;
                        best_idx = i;
                        best_side = AttachSide::PrependFwd;
                    }
                }
                if let Some(s) = e.start {
                    let d = dist3(head, s);
                    if d < best_dist {
                        best_dist = d;
                        best_idx = i;
                        best_side = AttachSide::PrependRev;
                    }
                }
            }
        }

        let mut next = edges.remove(best_idx);
        match best_side {
            AttachSide::AppendFwd => {
                next.reversed = false;
                chain.push_back(next);
            }
            AttachSide::AppendRev => {
                next.reversed = true;
                chain.push_back(next);
            }
            AttachSide::PrependFwd => {
                next.reversed = false;
                chain.push_front(next);
            }
            AttachSide::PrependRev => {
                next.reversed = true;
                chain.push_front(next);
            }
        }
    }

    let mut ordered: Vec<SelectedEdge> = chain.into_iter().collect();
    // If edge 0 ended up at the tail of the chain, flip the chain so it starts from the
    // first-picked edge's outer end while staying head-to-tail contiguous.
    if ordered.last().is_some_and(|e| e.orig_index == 0) {
        ordered.reverse();
        for e in &mut ordered {
            e.reversed = !e.reversed;
        }
    }
    ordered
}

fn arc_through_point(edge: &SelectedEdge) -> Option<[f64; 3]> {
    let center = edge.center?;
    let axis = edge.axis?;
    let s = edge.eff_start()?;
    let e = edge.eff_end()?;
    let vs = sub3(s, center);
    let ve = sub3(e, center);
    let r = libm::sqrt(dot3(vs, vs));
    if r <= 1e-9 {
        return None;
    }

    let cos_a = (dot3(vs, ve) / (r * r)).clamp(-1.0, 1.0);
    let sin_a = dot3(axis, cross3(vs, ve)) / (r * r);
    let mut a1 = libm::atan2(sin_a, cos_a);
    if a1 <= 0.0 {
        a1 += 2.0 * PI;
    }
    let a2 = 2.0 * PI - a1;

    let mid1 = add3(center, rotate_around(vs, axis, 0.5 * a1));
    let mid2 = add3(center, rotate_around(vs, scale3(axis, -1.0), 0.5 * a2));

    // 1. Prefer the edge's own pick point when it is away from the endpoints.
    if let Some(pt) = edge.point
        && dist3(pt, s) > 1e-4
        && dist3(pt, e) > 1e-4
    {
        let vp = sub3(pt, center);
        let vp_plane = sub3(vp, scale3(axis, dot3(vp, axis)));
        if dot3(vp_plane, vp_plane) > 1e-12 {
            let mut ap = libm::atan2(dot3(axis, cross3(vs, vp_plane)), dot3(vs, vp_plane));
            if ap <= 0.0 {
                ap += 2.0 * PI;
            }
            return Some(if ap < a1 { mid1 } else { mid2 });
        }
    }

    // 2. Otherwise compare arc length r * angle against reported edge length_mm.
    if let Some(len) = edge.length_mm
        && libm::fabs(a1 - a2) > 1e-3
    {
        let err1 = libm::fabs(r * a1 - len);
        let err2 = libm::fabs(r * a2 - len);
        return Some(if err1 <= err2 { mid1 } else { mid2 });
    }

    // 3. Final fallback based on edge reversal.
    Some(if edge.reversed { mid2 } else { mid1 })
}

fn emit_3d_path(
    edges: &[SelectedEdge],
    mode: PathMode,
    sketch_name: &str,
    finish_sketch: bool,
) -> Result<(Value, Vec<String>), PluginError> {
    let name = if sketch_name.trim().is_empty() { "Edge Path" } else { sketch_name.trim() };
    let created = run_cmd("sketch.create", json!({ "plane": "XY", "name": name }))?;
    let sketch_id = created
        .get("sketch")
        .cloned()
        .or_else(|| created.get("id").cloned())
        .ok_or_else(|| PluginError::Trap("sketch.create returned no sketch id".to_string()))?;

    run_cmd("sketch.options", json!({ "sketch_3d": true }))?;

    let mut curve_ids: Vec<String> = Vec::new();
    let mut first_start: Option<[f64; 3]> = None;
    let mut last_end: Option<[f64; 3]> = None;

    for edge in edges {
        let Some(s) = edge.eff_start() else {
            return Err(invalid("edges", "selected edge has no start point"));
        };
        let Some(e) = edge.eff_end() else {
            return Err(invalid("edges", "selected edge has no end point"));
        };
        if first_start.is_none() {
            first_start = Some(s);
        }
        last_end = Some(e);

        match edge.curve.as_str() {
            "line" => {
                if dist3(s, e) > EPS {
                    let res = run_cmd("sketch.line3d", json!({ "points": [s, e] }))?;
                    if let Some(arr) = res.get("curves").and_then(Value::as_array) {
                        for c in arr {
                            if let Some(id) = c.as_str() {
                                curve_ids.push(id.to_string());
                            }
                        }
                    }
                }
            }
            "circle" | "arc" => {
                // Full circle only when start and end coincide (split cylinder rims are 180 deg arcs).
                let is_full_circle = dist3(s, e) < 1e-4;
                if is_full_circle
                    && let (Some(center), Some(axis), Some(radius)) = (edge.center, edge.axis, edge.radius)
                {
                    let radial = match norm3(sub3(s, center)) {
                        Some(dir) => scale3(dir, radius),
                        None => {
                            let ref_v = if libm::fabs(axis[0]) < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
                            scale3(norm3(cross3(ref_v, axis)).unwrap_or([1.0, 0.0, 0.0]), radius)
                        }
                    };
                    let p0 = add3(center, radial);
                    let p90 = add3(center, rotate_around(radial, axis, 0.5 * PI));
                    let p180 = add3(center, rotate_around(radial, axis, PI));
                    let p270 = add3(center, rotate_around(radial, axis, 1.5 * PI));

                    for (a, b, c) in [(p0, p90, p180), (p180, p270, p0)] {
                        let res = run_cmd("sketch.arc3d", json!({ "start": a, "through": b, "end": c }))?;
                        if let Some(id) = res.get("curves").and_then(Value::as_array).and_then(|v| v.first()).and_then(Value::as_str) {
                            curve_ids.push(id.to_string());
                        }
                    }
                } else if let Some(through) = arc_through_point(edge) {
                    let res = run_cmd("sketch.arc3d", json!({ "start": s, "through": through, "end": e }))?;
                    if let Some(id) = res.get("curves").and_then(Value::as_array).and_then(|v| v.first()).and_then(Value::as_str) {
                        curve_ids.push(id.to_string());
                    }
                } else if dist3(s, e) > EPS {
                    let res = run_cmd("sketch.line3d", json!({ "points": [s, e] }))?;
                    if let Some(id) = res.get("curves").and_then(Value::as_array).and_then(|v| v.first()).and_then(Value::as_str) {
                        curve_ids.push(id.to_string());
                    }
                }
            }
            _ => {
                return Err(invalid(
                    "edges",
                    "arbitrary spline/intersection edges cannot be converted to a 3D sketch (select a sweep profile to sweep directly along body edges)",
                ));
            }
        }
    }

    if mode == PathMode::Closed
        && let (Some(fs), Some(le)) = (first_start, last_end)
        && dist3(le, fs) > 1e-4
    {
        let res = run_cmd("sketch.line3d", json!({ "points": [le, fs] }))?;
        if let Some(id) = res.get("curves").and_then(Value::as_array).and_then(|v| v.first()).and_then(Value::as_str) {
            curve_ids.push(id.to_string());
        }
    }

    if curve_ids.is_empty() {
        return Err(invalid("edges", "no usable 3D path curves were generated from the selected edges"));
    }

    if finish_sketch {
        run_cmd("sketch.finish", json!({}))?;
    }

    Ok((sketch_id, curve_ids))
}

fn resolve_profile_source(p: &EdgePathSweepParams, shape: ProfileShape) -> Result<Option<SweepProfile>, PluginError> {
    if !p.profile.is_empty() {
        let mut sketch_id: Option<u64> = None;
        let mut indices = Vec::new();
        for item in &p.profile {
            let kind = item
                .get("kind")
                .or_else(|| item.get("type"))
                .and_then(Value::as_str)
                .unwrap_or("");
            if kind != "profile" {
                return Err(invalid("profile", "picked item must be a sketch profile"));
            }
            let sk = item
                .get("sketch")
                .and_then(Value::as_u64)
                .ok_or_else(|| invalid("profile", "picked profile has no sketch id"))?;
            let idx = item
                .get("index")
                .and_then(Value::as_u64)
                .ok_or_else(|| invalid("profile", "picked profile has no index"))? as usize;
            if let Some(prev) = sketch_id {
                if prev != sk {
                    return Err(invalid("profile", "picked profiles must belong to one sketch"));
                }
            } else {
                sketch_id = Some(sk);
            }
            if !indices.contains(&idx) {
                indices.push(idx);
            }
        }
        if let Some(sketch) = sketch_id {
            return Ok(Some(SweepProfile::PickedProfiles { sketch, indices }));
        }
    }

    if !p.profile_sketch.trim().is_empty() {
        return Ok(Some(SweepProfile::SketchName(p.profile_sketch.trim().to_string())));
    }

    Ok(match shape {
        ProfileShape::None => None,
        ProfileShape::Circle => Some(SweepProfile::BuiltInCircle { diameter: p.profile_width }),
        ProfileShape::Rectangle => Some(SweepProfile::BuiltInRect {
            width: p.profile_width,
            height: p.profile_height,
        }),
    })
}

fn push_unique(out: &mut Vec<String>, id: &str) {
    if !out.iter().any(|existing| existing == id) {
        out.push(id.to_string());
    }
}

fn collect_sketch_curves(inspected: &Value, profile_indices: Option<&[usize]>) -> Vec<String> {
    let mut out = Vec::new();

    if let Some(indices) = profile_indices
        && let Some(profiles) = inspected.get("profiles").and_then(Value::as_array)
    {
        for &idx in indices {
            let profile = profiles
                .iter()
                .find(|p| p.get("index").and_then(Value::as_u64) == Some(idx as u64))
                .or_else(|| profiles.get(idx));
            if let Some(prof) = profile {
                if let Some(curves) = prof.get("curves").and_then(Value::as_array) {
                    for c in curves {
                        if let Some(id) = c.as_str() {
                            push_unique(&mut out, id);
                        }
                    }
                }
                if let Some(holes) = prof.get("holes").and_then(Value::as_array) {
                    for hole in holes {
                        if let Some(hcurves) = hole.as_array() {
                            for c in hcurves {
                                if let Some(id) = c.as_str() {
                                    push_unique(&mut out, id);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if out.is_empty()
        && let Some(curves) = inspected.get("curves").and_then(Value::as_array)
    {
        for c in curves {
            if c.get("construction").and_then(Value::as_bool) == Some(true) {
                continue;
            }
            if let Some(id) = c.get("id").and_then(Value::as_str) {
                push_unique(&mut out, id);
            }
        }
        if out.is_empty() {
            for c in curves {
                if let Some(id) = c.get("id").and_then(Value::as_str) {
                    push_unique(&mut out, id);
                }
            }
        }
    }

    out
}

fn copy_profile_onto_plane(
    src_sketch: Value,
    profile_indices: Option<&[usize]>,
    plane_ref: &Value,
    name: &str,
    offset_x: f64,
    offset_y: f64,
    profile_angle_deg: f64,
) -> Result<Value, PluginError> {
    run_cmd("sketch.edit", json!({ "sketch": src_sketch }))?;
    let inspected = match run_cmd("sketch.inspect", json!({})) {
        Ok(v) => v,
        Err(e) => {
            let _ = run_cmd("sketch.finish", json!({}));
            return Err(e);
        }
    };
    let entities = collect_sketch_curves(&inspected, profile_indices);
    if entities.is_empty() {
        let _ = run_cmd("sketch.finish", json!({}));
        return Err(invalid("profile", "profile sketch has no curves"));
    }
    if let Err(e) = run_cmd("sketch.copy", json!({ "entities": entities })) {
        let _ = run_cmd("sketch.finish", json!({}));
        return Err(e);
    }
    run_cmd("sketch.finish", json!({}))?;

    let prof_sketch = run_cmd("sketch.create", json!({ "plane": plane_ref, "name": format!("{name} Profile") }))?;
    let prof_id = prof_sketch
        .get("sketch")
        .cloned()
        .or_else(|| prof_sketch.get("id").cloned())
        .ok_or_else(|| PluginError::Trap("profile sketch creation returned no id".to_string()))?;

    run_cmd("sketch.paste", json!({ "at": [offset_x, offset_y] }))?;

    if libm::fabs(profile_angle_deg) > 1e-9 {
        let pasted_info = run_cmd("sketch.inspect", json!({}))?;
        let pasted_ids = collect_sketch_curves(&pasted_info, None);
        if !pasted_ids.is_empty() {
            run_cmd(
                "sketch.transform",
                json!({
                    "entities": pasted_ids,
                    "angle": profile_angle_deg,
                    "center": [offset_x, offset_y]
                }),
            )?;
        }
    }
    run_cmd("sketch.finish", json!({}))?;
    Ok(prof_id)
}

fn create_builtin_profile_on_plane(
    source: &SweepProfile,
    plane_ref: &Value,
    name: &str,
    offset_x: f64,
    offset_y: f64,
    profile_angle_deg: f64,
) -> Result<Value, PluginError> {
    let prof_sketch = run_cmd("sketch.create", json!({ "plane": plane_ref, "name": format!("{name} Profile") }))?;
    let prof_id = prof_sketch
        .get("sketch")
        .cloned()
        .or_else(|| prof_sketch.get("id").cloned())
        .ok_or_else(|| PluginError::Trap("profile sketch creation returned no id".to_string()))?;

    match *source {
        SweepProfile::BuiltInCircle { diameter } => {
            run_cmd(
                "sketch.circle.center",
                json!({
                    "center": [offset_x, offset_y],
                    "radius": diameter * 0.5
                }),
            )?;
        }
        SweepProfile::BuiltInRect { width, height } => {
            run_cmd(
                "sketch.rectangle.center",
                json!({
                    "center": [offset_x, offset_y],
                    "corner": [offset_x + width * 0.5, offset_y + height * 0.5]
                }),
            )?;
            if libm::fabs(profile_angle_deg) > 1e-9 {
                let info = run_cmd("sketch.inspect", json!({}))?;
                let rect_ids = collect_sketch_curves(&info, None);
                if !rect_ids.is_empty() {
                    run_cmd(
                        "sketch.transform",
                        json!({
                            "entities": rect_ids,
                            "angle": profile_angle_deg,
                            "center": [offset_x, offset_y]
                        }),
                    )?;
                }
            }
        }
        SweepProfile::PickedProfiles { .. } | SweepProfile::SketchName(_) => {}
    }

    run_cmd("sketch.finish", json!({}))?;
    Ok(prof_id)
}

fn plane_ref_from_result(sweep_plane: &Value) -> Result<Value, PluginError> {
    sweep_plane
        .get("feature")
        .cloned()
        .or_else(|| sweep_plane.get("id").cloned())
        .or_else(|| sweep_plane.get("name").cloned())
        .ok_or_else(|| PluginError::Trap("sweep plane creation returned no feature id or name".to_string()))
}

fn unique_base_name(base: &str) -> Result<String, PluginError> {
    let doc = run_cmd(
        "document.inspect",
        json!({ "measure": false, "timeline": "summary", "bodies": false }),
    )?;
    let existing: Vec<String> = doc
        .get("timeline")
        .and_then(Value::as_array)
        .map(|arr| {
            arr.iter()
                .filter_map(|row| row.get("name").and_then(Value::as_str).map(str::to_string))
                .collect()
        })
        .unwrap_or_default();

    let collides = |candidate: &str| {
        let plane = format!("{candidate} Sweep Plane");
        let prof = format!("{candidate} Profile");
        let sweep = format!("{candidate} Sweep");
        existing.iter().any(|name| {
            name.eq_ignore_ascii_case(candidate)
                || name.eq_ignore_ascii_case(&plane)
                || name.eq_ignore_ascii_case(&prof)
                || name.eq_ignore_ascii_case(&sweep)
        })
    };

    if !collides(base) {
        return Ok(base.to_string());
    }
    for n in 2..=10_000 {
        let candidate = format!("{base} ({n})");
        if !collides(&candidate) {
            return Ok(candidate);
        }
    }
    Ok(base.to_string())
}

fn run_edge_path_sweep(value: &Value) -> Result<String, PluginError> {
    let p: EdgePathSweepParams = serde_json::from_value(value.clone()).map_err(|e| invalid("params", e.to_string()))?;
    let mode = parse_path_mode(&p.path_mode)?;
    let shape = parse_profile_shape(&p.profile_shape)?;
    let orientation = parse_orientation(&p.orientation)?;
    let op = parse_sweep_operation(&p.sweep_operation)?;

    check_finite("profile_width", p.profile_width)?;
    check_finite("profile_height", p.profile_height)?;
    check_finite("offset_x", p.offset_x)?;
    check_finite("offset_y", p.offset_y)?;
    check_finite("profile_angle_deg", p.profile_angle_deg)?;
    check_finite("twist_deg", p.twist_deg)?;
    check_finite("end_scale", p.end_scale)?;

    if p.profile_width <= 0.0 {
        return Err(invalid("profile_width", "must be greater than 0"));
    }
    if p.profile_height <= 0.0 {
        return Err(invalid("profile_height", "must be greater than 0"));
    }
    if !(-1440.0..=1440.0).contains(&p.twist_deg) {
        return Err(invalid("twist_deg", "must be between -1440 and 1440"));
    }
    if !(0.01..=100.0).contains(&p.end_scale) {
        return Err(invalid("end_scale", "must be between 0.01 and 100"));
    }

    let sweep_profile = resolve_profile_source(&p, shape)?;
    let raw_edges = resolve_edges(&p.edges)?;
    let edges = order_edges(raw_edges);
    let raw_name = if p.sketch_name.trim().is_empty() { "Edge Path" } else { p.sketch_name.trim() };
    let name = unique_base_name(raw_name)?;
    let name = name.as_str();

    let Some(sweep_profile) = sweep_profile else {
        let need_finish = p.finish_sketch || p.create_sweep_plane;
        control::progress(0.25, "Creating 3D sketch path from edges");
        let (sketch_id, curve_ids) = emit_3d_path(&edges, mode, name, need_finish)?;

        let mut sweep_plane = Value::Null;
        if p.create_sweep_plane
            && let Some(first_curve) = curve_ids.first()
        {
            control::progress(0.8, "Creating sweep plane at path start");
            sweep_plane = run_cmd(
                "construct.plane.along_path",
                json!({
                    "path": { "sketch": sketch_id, "curve": first_curve },
                    "t": 0.0,
                    "name": format!("{name} Sweep Plane")
                }),
            )?;
        }

        control::progress(1.0, "Done");
        return Ok(json!({
            "mode": "path_only",
            "sketch": sketch_id,
            "curves": curve_ids,
            "closed": mode == PathMode::Closed,
            "sweep_plane": sweep_plane
        })
        .to_string());
    };

    // Sweep Mode
    let has_spline_edge = edges.iter().any(|e| e.curve == "other");
    let single_body = edges
        .first()
        .and_then(|e| e.body.as_ref())
        .filter(|b| edges.iter().all(|e| e.body.as_ref() == Some(b)))
        .cloned();

    let (path_sketch_id, path_curves, sweep_plane, use_body_edges) = if has_spline_edge && mode == PathMode::Open {
        let Some(body) = single_body else {
            return Err(invalid("edges", "direct spline-edge sweep requires all edges on one body"));
        };
        let edge_pts: Vec<[f64; 3]> = edges.iter().filter_map(|e| e.point).collect();
        if edge_pts.len() != edges.len() {
            return Err(invalid("edges", "missing pick point on selected edge"));
        }
        let first_pt = edge_pts[0];
        control::progress(0.3, "Creating sweep plane along body edge");
        let plane = run_cmd(
            "construct.plane.along_path",
            json!({
                "path": { "edge": first_pt, "body": body },
                "t": 0.0,
                "name": format!("{name} Sweep Plane")
            }),
        )?;
        (Value::Null, Vec::new(), plane, Some((body, edge_pts)))
    } else {
        control::progress(0.25, "Creating 3D sketch path from edges");
        let (sk_id, curves) = emit_3d_path(&edges, mode, name, true)?;
        let first_curve = curves.first().cloned().unwrap_or_default();
        control::progress(0.45, "Creating sweep plane at path start");
        let plane = run_cmd(
            "construct.plane.along_path",
            json!({
                "path": { "sketch": sk_id, "curve": first_curve },
                "t": 0.0,
                "name": format!("{name} Sweep Plane")
            }),
        )?;
        (sk_id, curves, plane, None)
    };

    let plane_ref = plane_ref_from_result(&sweep_plane)?;

    control::progress(0.65, "Aligning, offsetting, and rotating profile");
    let prof_id = match &sweep_profile {
        SweepProfile::PickedProfiles { sketch, indices } => copy_profile_onto_plane(
            json!(sketch),
            Some(indices),
            &plane_ref,
            name,
            p.offset_x,
            p.offset_y,
            p.profile_angle_deg,
        )?,
        SweepProfile::SketchName(sk) => copy_profile_onto_plane(
            json!(sk),
            None,
            &plane_ref,
            name,
            p.offset_x,
            p.offset_y,
            p.profile_angle_deg,
        )?,
        SweepProfile::BuiltInCircle { .. } | SweepProfile::BuiltInRect { .. } => {
            create_builtin_profile_on_plane(&sweep_profile, &plane_ref, name, p.offset_x, p.offset_y, p.profile_angle_deg)?
        }
    };

    control::progress(0.85, "Sweeping profile along path");
    let mut sweep_args = json!({
        "sketch": prof_id,
        "orientation": orientation,
        "operation": op,
        "name": format!("{name} Sweep")
    });
    if let Some((body, edge_pts)) = use_body_edges {
        sweep_args["path_body"] = json!(body);
        sweep_args["path_edges"] = json!(edge_pts);
    } else {
        sweep_args["path_sketch"] = path_sketch_id.clone();
        sweep_args["path"] = json!(path_curves);
    }
    if libm::fabs(p.twist_deg) > 1e-9 {
        sweep_args["twist"] = json!(format!("{} deg", p.twist_deg));
    }
    if libm::fabs(p.end_scale - 1.0) > 1e-9 {
        sweep_args["scale"] = json!(p.end_scale);
    }

    log::log(Level::Info, &format!("Sweeping profile `{}` along edge path", name));
    let sweep_result = run_cmd("solid.sweep", sweep_args)?;
    control::progress(1.0, "Done");

    Ok(json!({
        "mode": "sweep",
        "path_sketch": path_sketch_id,
        "path_curves": path_curves,
        "profile_sketch": prof_id,
        "sweep_plane": sweep_plane,
        "sweep": sweep_result
    })
    .to_string())
}
