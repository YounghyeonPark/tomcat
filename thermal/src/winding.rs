// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Younghyeon Park
//! The winding temperature — the question M18 could only put a warning on.
//!
//! Every figure in [`crate`] is an **assembly-skin** temperature, because a
//! [`LumpedMass`](dualis_thermal::LumpedMass) has one temperature and there was no
//! way to join two of them by a conductance. The caveat said so:
//!
//! > ⚠️ A lumped mass has ONE temperature. The real winding is hotter than the skin
//! > these numbers describe, and the winding is what fails.
//!
//! A warning is not an answer. `dualis-thermal` 0.3 added `ThermalNetwork`
//! ([upstream #2](https://github.com/YounghyeonPark/dualis/issues/2)), so the chain
//! **winding → stator → housing → girdle → air** is now expressible and the gradient
//! is a number.
//!
//! # What is `[assumed]` here, and why the sweep is the result
//!
//! The masses split a 131.7 g motor into copper, laminations and housing, and the
//! **joint conductances are guesses** — slot insulation, an interference fit, a
//! bolted mount. They are the least known thing in this file and the gradient is
//! roughly `P/UA` in each of them, so a single quoted rise would be false precision.
//! [`gradient_sweep`] is the deliverable; one number is not.
//!
//! ⚠️ Still copper loss only, inherited from ADR-0021. Iron loss appears in the
//! *stator*, not the winding, so including it would redistribute this gradient as
//! well as raise it.

use dualis_core::{Domain, Exchange, Substance};
use dualis_thermal::{Environment, Node, ThermalNetwork, HEAT};
use dualis_units::{Area, Conductance, Length, Temperature, Time, Volume};

use crate::{AMBIENT_C, STILL_AIR_H};

/// Mass split of one GIM3505-9, kg. `[assumed]` — sums to the 131.7 g of R1.
pub const WINDING_KG: f64 = 0.025;
/// Stator laminations.
pub const STATOR_KG: f64 = 0.045;
/// Housing, bearings, gearbox casing.
pub const HOUSING_KG: f64 = 0.060;
/// Girdle structure left over once six motors are removed from `front_girdle_mass`.
pub const STRUCTURE_KG: f64 = 1.122 - 6.0 * (WINDING_KG + STATOR_KG + HOUSING_KG);

/// Six motors per girdle, so every mass and conductance scales together.
pub const MOTORS: f64 = 6.0;

/// Joint conductances for ONE motor, W/K. `[assumed]`, and swept for that reason.
#[derive(Clone, Copy, Debug)]
pub struct Joints {
    /// Winding to stator, through the slot insulation — the tightest and worst-known.
    pub winding_stator: f64,
    /// Stator to housing, an interference fit over a large area.
    pub stator_housing: f64,
    /// Housing to girdle structure, a bolted mount.
    pub housing_structure: f64,
}

impl Default for Joints {
    fn default() -> Joints {
        Joints {
            winding_stator: 0.9,
            stator_housing: 2.4,
            housing_structure: 1.5,
        }
    }
}

impl Joints {
    /// Scale every joint by the same factor — the sweep axis.
    pub fn scaled(self, k: f64) -> Joints {
        Joints {
            winding_stator: self.winding_stator * k,
            stator_housing: self.stator_housing * k,
            housing_structure: self.housing_structure * k,
        }
    }
}

/// The four temperatures, once settled.
#[derive(Clone, Copy, Debug)]
pub struct Gradient {
    /// Hottest — and the only one a motor's rating actually refers to.
    pub winding_c: f64,
    /// Where iron loss would land, if it were modelled.
    pub stator_c: f64,
    /// Motor case.
    pub housing_c: f64,
    /// The girdle skin — what every M18 figure reports.
    pub skin_c: f64,
}

impl Gradient {
    /// How much hotter the winding is than the number M18 published.
    pub fn winding_above_skin(&self) -> f64 {
        self.winding_c - self.skin_c
    }
}

fn vol(kg: f64, s: &Substance) -> Volume {
    Volume::from_si(kg / s.density.to_si())
}

/// Build winding → stator → housing → girdle → air for a whole girdle.
///
/// The six motors are lumped per stage rather than modelled separately: they are
/// identical and symmetric, so six parallel chains reduce exactly to one chain with
/// six times the mass and six times the conductance.
pub fn girdle_network(emissivity: f64, h: f64, joints: Joints) -> (ThermalNetwork, [Node; 4]) {
    let amb = Temperature::celsius(AMBIENT_C);
    let cu = Substance::copper();
    let fe = Substance::electrical_steel();
    let al = Substance::aluminium_6061().with_emissivity(emissivity);

    let mut n = ThermalNetwork::new("girdle");
    // Interior nodes: no environment, so they can only conduct outward. Before 0.3
    // this was the blocking problem — a nested LumpedMass would also convect and
    // radiate to a room it cannot see, and shed heat twice.
    let winding = n.node(
        "winding",
        cu.clone(),
        vol(MOTORS * WINDING_KG, &cu),
        Length::mm(2.0),
        amb,
    );
    let stator = n.node(
        "stator",
        fe.clone(),
        vol(MOTORS * STATOR_KG, &fe),
        Length::mm(8.0),
        amb,
    );
    let housing = n.node(
        "housing",
        al.clone(),
        vol(MOTORS * HOUSING_KG, &al),
        Length::mm(4.0),
        amb,
    );
    // Only the structure sees air, through the same 302 cm2 skin M18 used — so this
    // network and the single lump reject heat through identical geometry.
    let structure = n.node_losing_to(
        "structure",
        al.clone(),
        vol(STRUCTURE_KG, &al),
        Length::from_si(crate::Part::girdle().volume_m3 / crate::Part::girdle().area_m2),
        amb,
        Environment {
            ambient: amb,
            convection_w_per_m2_k: h,
            area: Area::from_si(crate::Part::girdle().area_m2),
        },
    );

    let j = joints.scaled(MOTORS);
    n.link(winding, stator, Conductance::w_per_k(j.winding_stator))
        .expect("link");
    n.link(stator, housing, Conductance::w_per_k(j.stator_housing))
        .expect("link");
    n.link(housing, structure, Conductance::w_per_k(j.housing_structure))
        .expect("link");
    n.absorbing(winding).expect("copper loss lands in the copper");

    (n, [winding, stator, housing, structure])
}

/// Settle the network under a constant copper loss and read every node.
pub fn settled(emissivity: f64, h: f64, watts: f64, joints: Joints) -> Gradient {
    let (mut n, [w, s, ho, st]) = girdle_network(emissivity, h, joints);
    let mut bus = Exchange::new();
    let dt = 1.0;
    for k in 0..900_000 {
        bus.publish(HEAT, watts * dt);
        n.step(Time::s(k as f64 * dt), Time::s(dt), &mut bus)
            .expect("network step");
    }
    Gradient {
        winding_c: n.temperature(w).in_celsius(),
        stator_c: n.temperature(s).in_celsius(),
        housing_c: n.temperature(ho).in_celsius(),
        skin_c: n.temperature(st).in_celsius(),
    }
}

/// Run the network off a real pack until it goes flat, under the conservation audit.
///
/// The operating case, as opposed to the settled one: the girdle never gets near
/// equilibrium on 42 Wh (M18), so this is the temperature the winding actually sees.
pub fn discharge(
    emissivity: f64,
    h: f64,
    joints: Joints,
    battery_wh: f64,
    total_w: f64,
    heat_w: f64,
) -> Result<(Gradient, f64), dualis_core::Violation> {
    use dualis_core::{Schedule, Simulation};
    let (net, [w, st, ho, sk]) = girdle_network(emissivity, h, joints);
    let mut sim = Simulation::new(Schedule::Staggered)
        .with(crate::Battery::new(battery_wh, total_w, heat_w))
        .with(net);
    let mut minutes = 0.0;
    for k in 0..200_000 {
        sim.advance(Time::s(1.0))?;
        minutes = (k + 1) as f64 / 60.0;
        if sim
            .domain_as::<crate::Battery>("battery")
            .is_some_and(crate::Battery::is_flat)
        {
            break;
        }
    }
    let n = sim.domain_as::<ThermalNetwork>("girdle").expect("network");
    Ok((
        Gradient {
            winding_c: n.temperature(w).in_celsius(),
            stator_c: n.temperature(st).in_celsius(),
            housing_c: n.temperature(ho).in_celsius(),
            skin_c: n.temperature(sk).in_celsius(),
        },
        minutes,
    ))
}

/// The actual deliverable: how the winding gradient moves with the joints, which are
/// the least-known inputs in the model.
pub fn gradient_sweep(emissivity: f64, watts: f64) -> Vec<(f64, Gradient)> {
    [0.25, 0.5, 1.0, 2.0, 4.0]
        .iter()
        .map(|&k| (k, settled(emissivity, STILL_AIR_H, watts, Joints::default().scaled(k))))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::from_power_py::TROT_W;
    use crate::{EMIS_ANODISED, EMIS_POLISHED};

    const W: f64 = 6.0 * TROT_W;

    #[test]
    fn the_mass_split_still_adds_up_to_the_girdle() {
        let total = MOTORS * (WINDING_KG + STATOR_KG + HOUSING_KG) + STRUCTURE_KG;
        assert!((total - 1.122).abs() < 1e-9, "split sums to {total}");
        assert!(STRUCTURE_KG > 0.0, "six motors do not fit in the girdle");
    }

    #[test]
    fn the_network_skin_agrees_with_the_single_lump() {
        // The cross-check that makes the gradient trustworthy: same mass, same skin
        // area, same emissivity, so the OUTSIDE must behave as M18 already reported.
        // If this drifts, the network is not modelling the same girdle.
        for e in [EMIS_POLISHED, EMIS_ANODISED] {
            let lump = AMBIENT_C + crate::Part::girdle().equilibrium(e, STILL_AIR_H, W);
            let net = settled(e, STILL_AIR_H, W, Joints::default()).skin_c;
            assert!(
                (net - lump).abs() < 2.0,
                "network skin {net:.1} C vs single lump {lump:.1} C"
            );
        }
    }

    #[test]
    fn the_winding_is_hotter_than_the_skin_and_the_caveat_was_right() {
        let g = settled(EMIS_ANODISED, STILL_AIR_H, W, Joints::default());
        assert!(g.winding_c > g.stator_c);
        assert!(g.stator_c > g.housing_c);
        assert!(g.housing_c > g.skin_c);
    }

    #[test]
    fn the_gradient_is_set_by_the_joints_not_by_the_skin() {
        // Why the sweep IS the result: the joints are `[assumed]`, and the gradient
        // is roughly P/UA in each, so it moves with them almost proportionally.
        let sweep = gradient_sweep(EMIS_ANODISED, W);
        let tight = sweep.last().unwrap().1.winding_above_skin();
        let loose = sweep.first().unwrap().1.winding_above_skin();
        assert!(loose > tight * 4.0, "loose {loose:.1} K vs tight {tight:.1} K");
    }

    #[test]
    fn interior_nodes_have_no_biot_number_and_that_is_the_honest_answer() {
        // An interior node has no film coefficient to compare against. 0.3 returns
        // None rather than a reassuring number — which is the trap M18 walked into
        // when a 5e-4 Biot for a whole assembly looked like permission to lump it.
        let (n, [w, _, _, st]) = girdle_network(EMIS_ANODISED, STILL_AIR_H, Joints::default());
        assert!(n.biot_number(w).is_none());
        assert!(n.biot_number(st).is_some());
    }
}
