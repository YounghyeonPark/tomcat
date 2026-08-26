// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Younghyeon Park
//! The winding temperature — what M18 could only put a warning on.
//! `cargo run --release --example winding`

use tomcat_thermal::from_power_py::*;
use tomcat_thermal::winding::*;
use tomcat_thermal::*;

const BATTERY_WH: f64 = 42.0;
// M40: was a third stale copy of 83.5607. Use the guarded constant.
use tomcat_thermal::from_power_py::TOTAL_W;

fn main() {
    let w = 6.0 * TROT_W;

    println!("=== settled (continuous / tethered), {w:.1} W copper loss ===");
    for (fin, e) in [("polished", EMIS_POLISHED), ("anodised", EMIS_ANODISED)] {
        let g = settled(e, STILL_AIR_H, w, Joints::default());
        println!(
            "  {fin:9} winding {:6.1}  stator {:6.1}  housing {:6.1}  skin {:6.1} C  (+{:.1} K)",
            g.winding_c, g.stator_c, g.housing_c, g.skin_c, g.winding_above_skin()
        );
    }

    println!("\n=== ONE BATTERY (the operating case), coupled + audited ===");
    for (fin, e) in [("polished", EMIS_POLISHED), ("anodised", EMIS_ANODISED)] {
        match discharge(e, STILL_AIR_H, Joints::default(), BATTERY_WH, TOTAL_W, w) {
            Ok((g, m)) => println!(
                "  {fin:9} flat at {m:5.2} min: winding {:5.1} C, skin {:5.1} C  (+{:.1} K)",
                g.winding_c, g.skin_c, g.winding_above_skin()
            ),
            Err(v) => println!("  {fin:9} AUDIT FAILED: {v:?}"),
        }
    }

    println!("\n=== gradient vs joint conductance (anodised) — the [assumed] axis ===");
    println!("  {:>8} {:>12} {:>10} {:>16}", "joints", "winding C", "skin C", "winding-skin K");
    for (k, g) in gradient_sweep(EMIS_ANODISED, w) {
        println!(
            "  {:>7.2}x {:>11.1} {:>10.1} {:>15.1}",
            k, g.winding_c, g.skin_c, g.winding_above_skin()
        );
    }
    println!("\n  The skin finish moves the whole stack; the joints set the spread.");
    println!("  They are independent levers, and only the second is uncertain.");
}
