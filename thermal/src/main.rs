// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Younghyeon Park
//! Prints the R5 thermal report. `cargo run --release` from `thermal/`.

use tomcat_thermal::from_power_py::*;
use tomcat_thermal::*;

fn c(rise: f64) -> f64 {
    AMBIENT_C + rise
}

fn main() {
    let motor = Part::motor();
    let girdle = Part::girdle();

    println!("=== as modelled ===");
    println!(
        "  motor   {:5.1} cm2 skin, Biot {:.4}, tau {:4.1} min",
        motor.area_m2 * 1e4,
        motor.biot(STILL_AIR_H),
        motor.time_constant_min(EMIS_POLISHED, STILL_AIR_H)
    );
    println!(
        "  girdle  {:5.1} cm2 skin, Biot {:.4}, tau {:4.1} min",
        girdle.area_m2 * 1e4,
        girdle.biot(STILL_AIR_H),
        girdle.time_constant_min(EMIS_POLISHED, STILL_AIR_H)
    );
    println!(
        "  centralising 6 motors costs {:.0} % of the rejection area ({:.0} -> {:.0} cm2)",
        100.0 * (1.0 - girdle.area_m2 / (6.0 * motor.area_m2)),
        6.0 * motor.area_m2 * 1e4,
        girdle.area_m2 * 1e4
    );
    println!("  runtime: trot {TROT_RUNTIME_MIN:.1} min, stand {STAND_RUNTIME_MIN:.1} min");

    println!("\n=== front girdle, 6 motors — the boundary that decides it ===");
    println!("  {:22} {:>12} {:>16}", "", "continuous", "one battery");
    for (name, w, mins) in [
        ("trot (21.0 W)", 6.0 * TROT_W, TROT_RUNTIME_MIN),
        ("stand, no brake", 6.0 * STAND_W, STAND_RUNTIME_MIN),
    ] {
        for (fin, e) in [("polished", EMIS_POLISHED), ("anodised", EMIS_ANODISED)] {
            println!(
                "  {name:15} {fin:9} {:8.1} C {:12.1} C",
                c(girdle.equilibrium(e, STILL_AIR_H, w)),
                c(girdle.rise_after(e, STILL_AIR_H, w, mins))
            );
        }
    }

    println!("\n=== airflow sensitivity (girdle, anodised, trot) ===");
    for h in [5.0, 7.0, 15.0, 25.0] {
        println!(
            "  h={h:5.1} W/m2K -> continuous {:5.1} C",
            c(girdle.equilibrium(EMIS_ANODISED, h, 6.0 * TROT_W))
        );
    }

    // The version to trust: pack and girdle as two coupled domains, energy audited
    // by the kernel, runtime emergent rather than taken from elsewhere.
    println!("\n=== coupled discharge, under the kernel's conservation audit ===");
    const BATTERY_WH: f64 = 42.0;
    const TOTAL_W: f64 = 83.5607;
    for (fin, e) in [("polished", EMIS_POLISHED), ("anodised", EMIS_ANODISED)] {
        match girdle.discharge(e, STILL_AIR_H, BATTERY_WH, TOTAL_W, 6.0 * TROT_W) {
            Ok(d) => println!(
                "  {fin:9} pack flat at {:5.2} min -> {:5.1} C  \
                 ({:.0} % of the settled rise)",
                d.minutes,
                c(d.rise_k),
                100.0 * d.fraction_of_equilibrium
            ),
            Err(v) => println!("  {fin:9} AUDIT FAILED: {v:?}"),
        }
    }
    println!(
        "  runtime is emergent ({BATTERY_WH} Wh at {TOTAL_W:.1} W), not assumed \
         — power.py says {TROT_RUNTIME_MIN:.2} min"
    );

    println!("\n⚠️  assembly-SKIN temperatures. Windings run hotter, and copper loss");
    println!("    is the only source modelled (ADR-0021), so reality is worse.");
}
