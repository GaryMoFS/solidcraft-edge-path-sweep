use std::fs::{self, File};
use std::io::Write;
use std::path::PathBuf;

use anyhow::{Context, Result};
use wit_component::{ComponentEncoder, StringEncoding};
use wit_parser::Resolve;
use zip::write::SimpleFileOptions;
use zip::ZipWriter;

fn main() -> Result<()> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().context("missing parent")?.to_path_buf();
    let core_wasm = root.join("plugin/target/wasm32-unknown-unknown/release/edge_path_sweep_plugin.wasm");
    let wit_path = root.join("wit/plugin.wit");
    let manifest_path = root.join("plugin.json");

    let mut wasm_bytes = fs::read(&core_wasm).with_context(|| format!("read {}", core_wasm.display()))?;
    let mut resolve = Resolve::default();
    let (pkg_id, _) = resolve.push_path(&wit_path).with_context(|| format!("parse {}", wit_path.display()))?;
    let world_id = resolve.select_world(&[pkg_id], Some("command-plugin"))?;

    wit_component::embed_component_metadata(&mut wasm_bytes, &resolve, world_id, StringEncoding::UTF8)?;
    let component = ComponentEncoder::default().module(&wasm_bytes)?.validate(true).encode()?;

    let dist_dir = root.join("dist/edge-path-sweep");
    fs::create_dir_all(&dist_dir)?;
    let out_wasm = dist_dir.join("edge-path-sweep.wasm");
    let out_manifest = dist_dir.join("plugin.json");
    let manifest_bytes = fs::read(&manifest_path)?;

    fs::write(&out_wasm, &component)?;
    fs::write(&out_manifest, &manifest_bytes)?;

    let zip_path = root.join("dist/edge-path-sweep.zip");
    let file = File::create(&zip_path)?;
    let mut zip = ZipWriter::new(file);
    let opts = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    zip.start_file("edge-path-sweep/plugin.json", opts)?;
    zip.write_all(&manifest_bytes)?;
    zip.start_file("edge-path-sweep/edge-path-sweep.wasm", opts)?;
    zip.write_all(&component)?;
    zip.finish()?;

    println!("Built component: {} ({} bytes)", out_wasm.display(), component.len());
    println!("Packaged zip:    {}", zip_path.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use serde_json::{json, Value};
    use solidcraft_engine::Session;

    fn dist_and_approval(session: &mut Session) -> (PathBuf, String) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().unwrap().to_path_buf();
        let pkg_dir = root.join("dist/edge-path-sweep");
        let zip_path = root.join("dist/edge-path-sweep.zip");
        assert!(pkg_dir.join("plugin.json").exists(), "run packager first");
        assert!(zip_path.exists(), "run packager first");

        let inspect = session
            .execute("plugin.inspect", &json!({ "path": pkg_dir.to_string_lossy() }))
            .expect("plugin.inspect should succeed");
        assert_eq!(inspect["within_host_limits"], json!(true));
        let approval = inspect["approval"].as_str().expect("approval hash").to_string();
        println!("PLUGIN_APPROVAL={approval}");
        (pkg_dir, approval)
    }

    fn dist3(a: &[f64], b: &[f64]) -> f64 {
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        let dz = a[2] - b[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    fn edge_ref_near(session: &mut Session, body: &str, target: [f64; 3]) -> Value {
        let listed = session.execute("model.edges", &json!({ "body": body })).expect("model.edges");
        let edges = listed["edges"].as_array().expect("edges array");
        let best = edges
            .iter()
            .min_by(|a, b| {
                let am = a["mid"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect::<Vec<_>>();
                let bm = b["mid"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect::<Vec<_>>();
                dist3(&am, &target).total_cmp(&dist3(&bm, &target))
            })
            .expect("matching edge");
        json!({
            "type": "edge",
            "body": body,
            "index": best["index"],
            "point": best["mid"]
        })
    }

    #[test]
    fn real_solidcraft_r47_path_only_open_closed_and_out_of_order_chaining() {
        let mut session = Session::default();
        let (pkg_dir, approval) = dist_and_approval(&mut session);

        // Box spanning x in [0, 40], y in [0, 30], z in [0, 20].
        session
            .execute("solid.box", &json!({ "length": 40.0, "width": 30.0, "height": 20.0, "body_name": "GuideBox" }))
            .expect("box");

        // Keep an unrelated body selected to verify explicit `edges` restores session selection.
        session.execute("select.set", &json!({ "items": [{ "type": "body", "name": "GuideBox" }] })).expect("select body");

        // 1. Out-of-order and reversed-first-edge chain:
        // Pick 3 connected edges around the top perimeter in non-sequential order:
        // top-left (x=0, y=15, z=20), top-right (x=40, y=15, z=20), top-back (x=20, y=30, z=20).
        let e_left = edge_ref_near(&mut session, "GuideBox", [0.0, 15.0, 20.0]);
        let e_right = edge_ref_near(&mut session, "GuideBox", [40.0, 15.0, 20.0]);
        let e_back = edge_ref_near(&mut session, "GuideBox", [20.0, 30.0, 20.0]);

        let res_open = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "edge-path-sweep",
                    "params": {
                        "edges": [e_left.clone(), e_right.clone(), e_back.clone()],
                        "path_mode": "open",
                        "sketch_name": "OpenChain"
                    }
                }),
            )
            .expect("open out-of-order path should succeed");
        assert_eq!(res_open["result"]["mode"], json!("path_only"));
        assert_eq!(res_open["result"]["closed"], json!(false));
        assert_eq!(res_open["result"]["curves"].as_array().map(Vec::len), Some(3));
        assert!(!res_open["result"]["sweep_plane"].is_null());

        // Verify the 3 generated wires form one contiguous head-to-tail chain with zero jumps.
        let inspected = session.execute("sketch.inspect", &json!({ "sketch": "OpenChain" })).expect("inspect open chain");
        let wires = inspected["wires"].as_array().expect("wires");
        assert_eq!(wires.len(), 3);
        for pair in wires.windows(2) {
            let end: Vec<f64> = pair[0]["end"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
            let start: Vec<f64> = pair[1]["start"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
            assert!(dist3(&end, &start) < 1e-4, "wires must chain head-to-tail: {end:?} vs {start:?}");
        }

        // Verify original selection was preserved.
        let sel = session.execute("selection.get", &json!({ "detail": false })).expect("selection.get");
        assert_eq!(sel["items"].as_array().map(Vec::len), Some(1));
        assert_eq!(sel["items"][0]["type"], json!("body"));

        // 2. Closed path from 2 edges adds a 3rd closing line.
        let e_front = edge_ref_near(&mut session, "GuideBox", [20.0, 0.0, 20.0]);
        let res_closed_2 = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "edge-path-sweep",
                    "params": {
                        "edges": [e_front.clone(), e_right.clone()],
                        "path_mode": "closed",
                        "sketch_name": "ClosedFromTwo"
                    }
                }),
            )
            .expect("closed from 2 edges");
        assert_eq!(res_closed_2["result"]["closed"], json!(true));
        assert_eq!(res_closed_2["result"]["curves"].as_array().map(Vec::len), Some(3));

        // 3. Closed path from all 4 perimeter edges does NOT add a 5th duplicate closing line.
        let res_closed_4 = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "edge-path-sweep",
                    "params": {
                        "edges": [e_front, e_right, e_back, e_left],
                        "path_mode": "closed",
                        "sketch_name": "ClosedFromFour"
                    }
                }),
            )
            .expect("closed from 4 edges");
        assert_eq!(res_closed_4["result"]["curves"].as_array().map(Vec::len), Some(4));
    }

    #[test]
    fn real_solidcraft_r47_cylinder_rim_arc_orientation() {
        let mut session = Session::default();
        let (pkg_dir, approval) = dist_and_approval(&mut session);

        session
            .execute("solid.cylinder", &json!({ "radius": 10.0, "height": 15.0, "body_name": "Cyl" }))
            .expect("cylinder");

        let listed = session.execute("model.edges", &json!({ "body": "Cyl" })).expect("model.edges");
        let edges = listed["edges"].as_array().expect("edges");
        // Find top rim arc edges (z ~= 15.0, not vertical seam).
        let top_arcs: Vec<Value> = edges
            .iter()
            .filter(|e| e["mid"][2].as_f64().unwrap() > 14.9)
            .map(|e| {
                json!({
                    "type": "edge",
                    "body": "Cyl",
                    "index": e["index"],
                    "point": e["mid"]
                })
            })
            .collect();
        assert!(!top_arcs.is_empty(), "expected top rim edge(s) on cylinder");

        // Pick ONLY ONE top rim edge (180 deg split rim or full circle depending on kernel representation).
        let first_rim = top_arcs[0].clone();
        let expected_mid: Vec<f64> = first_rim["point"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();

        let res = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "edge-path-sweep",
                    "params": {
                        "edges": [first_rim],
                        "path_mode": "open",
                        "sketch_name": "RimArc"
                    }
                }),
            )
            .expect("single rim arc path should succeed");
        let curve_count = res["result"]["curves"].as_array().map(Vec::len).unwrap();
        let expected_count = if top_arcs.len() == 1 { 2 } else { 1 };
        assert_eq!(curve_count, expected_count, "split rim must produce 1 arc, full circle 2 arcs");

        if expected_count == 1 {
            let inspected = session.execute("sketch.inspect", &json!({ "sketch": "RimArc" })).expect("inspect rim arc");
            let wire = &inspected["wires"][0];
            assert!(wire["arc"].is_object(), "must be an exact 3D arc: {wire}");
            let sweep_deg = wire["arc"]["sweep_deg"].as_f64().unwrap();
            assert!((sweep_deg - 180.0).abs() < 1.0, "expected ~180 deg arc, got {sweep_deg}");
            // Verify wire passes near the picked edge midpoint (not the opposite 180 deg side).
            let mut min_d = f64::MAX;
            let state = session.world_state().unwrap();
            let ss = state.sketches.iter().find(|s| s.name == "RimArc").unwrap();
            for p in &ss.sketch.wires[0].pts {
                min_d = min_d.min(dist3(&[p.x, p.y, p.z], &expected_mid));
            }
            assert!(min_d < 0.2, "arc must match picked rim side; min dist to midpoint = {min_d}");
        }
    }

    #[test]
    fn real_solidcraft_r47_unified_sweep_picked_profile_builtin_and_repeated_runs() {
        let mut session = Session::default();
        let (pkg_dir, approval) = dist_and_approval(&mut session);

        session
            .execute("solid.box", &json!({ "length": 40.0, "width": 30.0, "height": 20.0, "body_name": "GuideBox" }))
            .expect("box");

        // Create a sketch with TWO separate profiles + a construction line.
        let prof_sk = session.execute("sketch.create", &json!({ "plane": "XY", "name": "MultiProf" })).expect("prof sketch");
        let prof_sk_id = prof_sk["sketch"].as_u64().expect("sketch id");
        session
            .execute("sketch.rectangle.center", &json!({ "center": [0.0, 0.0], "corner": [2.0, 1.0] }))
            .expect("rect 1 (4x2)");
        session
            .execute("sketch.rectangle.center", &json!({ "center": [50.0, 0.0], "corner": [60.0, 10.0] }))
            .expect("rect 2 (20x20)");
        session
            .execute("sketch.line", &json!({ "points": [[-10.0, -10.0], [10.0, -10.0]], "construction": true }))
            .expect("construction line");
        session.execute("sketch.finish", &json!({})).expect("finish prof");

        let e_front = edge_ref_near(&mut session, "GuideBox", [20.0, 0.0, 20.0]);
        let e_right = edge_ref_near(&mut session, "GuideBox", [40.0, 15.0, 20.0]);

        // 1. Sweep ONLY profile 0 (the 4x2 rectangle) with offset & rotation, leaving default sketch_name.
        let res_1: Value = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "edge-path-sweep",
                    "params": {
                        "edges": [e_front.clone(), e_right.clone()],
                        "profile": [{ "type": "profile", "sketch": prof_sk_id, "index": 0 }],
                        "offset_x": 0.5,
                        "offset_y": -0.25,
                        "profile_angle_deg": 30.0,
                        "sweep_operation": "new_body"
                    }
                }),
            )
            .expect("sweep 1 should succeed");
        assert_eq!(res_1["result"]["mode"], json!("sweep"));
        let bodies_1 = res_1["result"]["sweep"]["created_bodies"].as_array().expect("created_bodies");
        assert_eq!(bodies_1.len(), 1, "only profile 0 must be swept, not profile 1: {bodies_1:?}");

        // 2. Second sweep with the SAME default sketch_name ("Edge Path") and built-in rectangle:
        // verifies plane_ref resolves by feature ID (not colliding with the first sweep plane's name).
        let e_back = edge_ref_near(&mut session, "GuideBox", [20.0, 30.0, 20.0]);
        let res_2: Value = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "edge-path-sweep",
                    "params": {
                        "edges": [e_back],
                        "profile_shape": "rectangle",
                        "profile_width": 4.0,
                        "profile_height": 2.0,
                        "profile_angle_deg": 45.0,
                        "sweep_operation": "new_body"
                    }
                }),
            )
            .expect("second default-named sweep with built-in rectangle should succeed");
        assert_eq!(res_2["result"]["mode"], json!("sweep"));
        assert_eq!(res_2["result"]["sweep"]["created_bodies"].as_array().map(Vec::len), Some(1));

        // 3. Built-in circle sweep.
        let e_left = edge_ref_near(&mut session, "GuideBox", [0.0, 15.0, 20.0]);
        let res_3: Value = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "edge-path-sweep",
                    "params": {
                        "edges": [e_left],
                        "profile_shape": "circle",
                        "profile_width": 3.0,
                        "sweep_operation": "new_body",
                        "sketch_name": "CircleEdgePath"
                    }
                }),
            )
            .expect("built-in circle sweep should succeed");
        assert_eq!(res_3["result"]["mode"], json!("sweep"));
        assert_eq!(res_3["result"]["sweep"]["created_bodies"].as_array().map(Vec::len), Some(1));
    }
}
