// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Younghyeon Park
//! Thermal duty — closing [OPEN_RISKS R5](../../docs/OPEN_RISKS.md).
//!
//! ADR-0021 checked the motor **electrically** — 2.79 A peak against a 4.19 A
//! rating, 0.89 A RMS against 1.60 A — and called it comfortable. That is not the
//! thermal question. The thermal question is whether the heat can *leave*, and R5
//! has sat as "gated on hardware" ever since.
//!
//! It is not gated on hardware. A lumped-capacitance model answers it, and the
//! answer changes a design decision.
//!
//! # The finding
//!
//! **The battery is the thermal protection, by coincidence rather than design.**
//! A bare girdle's effective time constant (~47 min) is longer than the runtime it
//! can be fed for (~30 min), so the robot runs out of energy before it overheats.
//! Tether it, or hot-swap the pack, and that protection vanishes: continuous
//! trotting with a polished girdle settles near **114 °C**.
//!
//! ⚠️ Anodised, that mechanism does **not** apply — its effective time constant is
//! 25.6 min, *shorter* than the runtime. It is safe on its own merits, because its
//! equilibrium is ~75 °C. See [`Part::effective_time_constant_min`]; the figure
//! first published for M18 came from the convection-only `LumpedMass::time_constant`.
//!
//! # ⚠️ What this model does not see
//!
//! - **Winding hotspots.** A lumped mass has ONE temperature. The real winding is
//!   hotter than the skin these numbers describe, and the winding is what fails.
//!   Treat every figure here as an assembly-skin temperature, i.e. optimistic for
//!   the component that actually matters.
//! - **Copper loss only.** Inherited from ADR-0021 — no iron, switching or gearbox
//!   losses, so the real dissipation is higher than the watts fed in here.
//! - **Internal conduction is assumed perfect** at girdle level. The Biot number
//!   says the lumped approximation is safe for a *solid* block; a girdle is a
//!   structure with motors and air gaps inside, so it is weaker than the number
//!   suggests.
//! - The girdle envelope, `h`, emissivities and motor geometry are all `[assumed]`,
//!   which is why every result is swept rather than stated once.

pub mod winding;

use dualis_core::conserved::quantity;
use dualis_core::substance::ThermalProps;
use dualis_core::{Domain, Exchange, Kind, Ledger, Schedule, Simulation, Substance, Violation};
use dualis_thermal::{Environment, LumpedMass, HEAT};
use dualis_units::{
    Area, Density, Length, Power, SpecificHeat, Temperature, ThermalConductivity,
    ThermalExpansion, Time, Volume,
};

/// The pack, draining in real time, publishing only its copper share as heat.
///
/// Added on dualis 0.2. The first pass compared two numbers by hand — "the girdle's
/// 53 min time constant beats the 30 min runtime" — which is a *claim about* the
/// coupling, not the coupling itself. Running the battery as a real domain makes the
/// runtime **emergent** (it falls out of 42 Wh at 83.6 W) and puts the whole thing
/// under [`Simulation`]'s conservation audit, so the bookkeeping is checked by the
/// kernel rather than by me.
///
/// Only the copper loss lands in the girdle. Mechanical work and electronics leave
/// the pack too, so they are carried in `spent_elsewhere` — without them the ledger
/// would show the pack losing energy that never arrived anywhere and the audit would
/// (correctly) refuse it.
pub struct Battery {
    name: String,
    /// Joules left in the pack.
    remaining_j: f64,
    /// Joules that left the pack as work or electronics, not as girdle heat.
    spent_elsewhere_j: f64,
    /// Total draw, W — what actually empties the pack.
    total_w: f64,
    /// The share of that arriving in this girdle as copper loss, W.
    heat_w: f64,
}

impl Battery {
    /// A pack of `wh` watt-hours, drawn at `total_w`, of which `heat_w` is girdle heat.
    pub fn new(wh: f64, total_w: f64, heat_w: f64) -> Battery {
        assert!(heat_w <= total_w, "heat share cannot exceed the total draw");
        Battery {
            name: "battery".to_string(),
            remaining_j: wh * 3600.0,
            spent_elsewhere_j: 0.0,
            total_w,
            heat_w,
        }
    }

    /// Joules left.
    pub fn remaining_j(&self) -> f64 {
        self.remaining_j
    }

    /// Whether the pack is flat — the thing that ends the run.
    pub fn is_flat(&self) -> bool {
        self.remaining_j <= 0.0
    }
}

impl Domain for Battery {
    fn name(&self) -> &str {
        &self.name
    }

    fn kind(&self) -> Kind {
        Kind::Evolving
    }

    fn max_stable_dt(&self, _now: Time) -> Time {
        // Draining is linear, so nothing here is unstable; cap the step at the life
        // left so the pack cannot overshoot into negative energy inside one step.
        if self.total_w <= 0.0 || self.remaining_j <= 0.0 {
            return Time::s(f64::INFINITY);
        }
        Time::s(self.remaining_j / self.total_w)
    }

    fn step(&mut self, _t: Time, dt: Time, bus: &mut Exchange) -> Result<(), Violation> {
        let want = self.total_w * dt.to_si();
        let drawn = want.min(self.remaining_j);
        // Scale the heat share with what was actually drawn, so a partial last step
        // splits the same way a full one does.
        let heat = if want > 0.0 {
            self.heat_w * dt.to_si() * (drawn / want)
        } else {
            0.0
        };
        self.remaining_j -= drawn;
        self.spent_elsewhere_j += drawn - heat;
        bus.publish(HEAT, heat);
        Ok(())
    }

    /// Everything the pack still accounts for: what is left in it, plus what it has
    /// already spent on work and electronics. NOT the heat — that is the girdle's
    /// entry now, and counting it here too would double it.
    fn ledger(&self) -> Ledger {
        Ledger::new().with(quantity::ENERGY, self.remaining_j + self.spent_elsewhere_j)
    }

    /// Needed so `discharge` can read the pack back out of the simulation.
    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }
}

/// Result of running a girdle off a real pack until it goes flat.
#[derive(Clone, Copy, Debug)]
pub struct Discharge {
    /// Minutes until the pack was empty — emergent, not assumed.
    pub minutes: f64,
    /// Girdle rise above ambient when it went flat, K.
    pub rise_k: f64,
    /// Fraction of the settled rise actually reached. Below 1 means the pack, not
    /// the design, is what kept the temperature down.
    pub fraction_of_equilibrium: f64,
}

/// Per-motor dissipation and runtimes, generated by `tomcat_kin.power`.
///
/// ⚠️ These are copied from the Python model, so they can go stale. The pytest
/// `tests/test_thermal_constants.py` fails if `power.py` moves away from them —
/// which is the only thing keeping this crate honest.
pub mod from_power_py {
    /// Copper loss per leg motor at the 50 cm/s trot, W. (`copper_w / 12`)
    pub const TROT_W: f64 = 3.5029;
    /// Per-motor draw HOLDING a stance, W. Higher than trotting — a cable can only
    /// pull, so posture costs current. (`legs_w / 12`)
    pub const STAND_W: f64 = 4.3514;
    /// Minutes of trotting on the 300 g pack.
    pub const TROT_RUNTIME_MIN: f64 = 30.16;
    /// Minutes standing on the pack, brake OFF.
    pub const STAND_RUNTIME_MIN: f64 = 37.49;
}

/// Room air. Everything is quoted as a rise, so this only sets the absolute scale.
pub const AMBIENT_C: f64 = 25.0;
/// Still-air convective coefficient, W·m⁻²·K⁻¹. `[assumed]`
pub const STILL_AIR_H: f64 = 7.0;
/// Bare/polished aluminium.
pub const EMIS_POLISHED: f64 = 0.09;
/// Anodised — and the single cheapest thermal lever available here.
pub const EMIS_ANODISED: f64 = 0.90;

/// A body that heats up: geometry, mass and surface finish.
#[derive(Clone, Copy, Debug)]
pub struct Part {
    /// Outer surface available to lose heat through, m².
    pub area_m2: f64,
    /// Enclosed volume, m³.
    pub volume_m3: f64,
    /// Mass, kg.
    pub mass_kg: f64,
    /// Specific heat, J·kg⁻¹·K⁻¹.
    pub cp: f64,
    /// Bulk conductivity, W·m⁻¹·K⁻¹ — only used for the Biot check.
    pub k: f64,
}

impl Part {
    /// One SteadyWin GIM3505-9, free-standing.
    ///
    /// "3505" is a 35 mm stator; with the housing and the 9:1 planetary the external
    /// envelope is about ⌀46 × 33 mm `[assumed]`. `cp = 450` rather than aluminium's
    /// 896 because a motor is copper, electrical steel, magnets and air — and that
    /// choice **halves the time constant**, which is the whole answer here.
    pub fn motor() -> Part {
        let (d, l) = (0.046, 0.033);
        Part {
            area_m2: 2.0 * std::f64::consts::PI * d * d / 4.0 + std::f64::consts::PI * d * l,
            volume_m3: std::f64::consts::PI * d * d / 4.0 * l,
            mass_kg: 0.1317,
            cp: 450.0,
            k: 50.0,
        }
    }

    /// The front girdle: six motors and the structure holding them.
    ///
    /// **This is the honest boundary.** P1 centralises the motors, so their
    /// individual surfaces are not free to room air — the assembly skin is what
    /// rejects the heat, and it cannot reject more than its own area allows however
    /// good the internal conduction is.
    pub fn girdle() -> Part {
        let (w, h, d) = (0.096, 0.060, 0.060); // spans the ±48 mm track `[assumed]`
        Part {
            area_m2: 2.0 * (w * h + w * d + h * d),
            volume_m3: w * h * d,
            mass_kg: 1.122, // params.front_girdle_mass — motors included
            cp: 600.0,
            k: 150.0,
        }
    }

    fn substance(&self, emissivity: f64) -> Substance {
        Substance {
            name: "part".to_string(),
            density: Density::kg_per_m3(self.mass_kg / self.volume_m3),
            thermal: Some(ThermalProps {
                conductivity: ThermalConductivity::w_per_m_k(self.k),
                specific_heat: SpecificHeat::j_per_kg_k(self.cp),
                expansion: ThermalExpansion::ppm_per_k(23.0),
                emissivity,
            }),
            mechanical: None,
            acoustic: None,
        }
    }

    /// Build the lumped domain for this part.
    pub fn lumped(&self, emissivity: f64, h: f64) -> LumpedMass {
        LumpedMass::new(
            "part",
            self.substance(emissivity),
            Volume::from_si(self.volume_m3),
            Length::from_si(self.volume_m3 / self.area_m2),
            Temperature::celsius(AMBIENT_C),
            Environment {
                ambient: Temperature::celsius(AMBIENT_C),
                convection_w_per_m2_k: h,
                area: Area::from_si(self.area_m2),
            },
        )
    }

    /// Rise above ambient (K) after `minutes` absorbing `watts`.
    ///
    /// Stepped rather than solved, so the radiative `εσA(T⁴-Tₐ⁴)` term is carried
    /// exactly instead of linearised — it is the same order as still-air convection
    /// at these temperatures, so linearising it would hide the anodising result.
    pub fn rise_after(&self, emissivity: f64, h: f64, watts: f64, minutes: f64) -> f64 {
        let mut m = self.lumped(emissivity, h);
        let mut bus = Exchange::new();
        let dt = 1.0;
        for k in 0..(minutes * 60.0 / dt) as usize {
            bus.publish(HEAT, watts * dt);
            m.step(Time::s(k as f64 * dt), Time::s(dt), &mut bus)
                .expect("lumped step");
        }
        m.rise().to_si()
    }

    /// Settled rise (K) — 10 time constants is plenty.
    pub fn equilibrium(&self, emissivity: f64, h: f64, watts: f64) -> f64 {
        let tau = self.time_constant_min(emissivity, h);
        self.rise_after(emissivity, h, watts, tau * 10.0)
    }

    /// `C/(hA + linearised radiative)` in minutes, as dualis 0.3 reports it.
    ///
    /// ⚠️ Took an emissivity argument only from 0.3 on. Before that
    /// `LumpedMass::time_constant` was convection-only, so this helper hardcoded
    /// `EMIS_POLISHED` and it made no difference. On 0.3 it makes a large one
    /// (49.2 min polished vs 29.9 anodised), and leaving the hardcode in would have
    /// been a silent regression introduced by an upgrade that fixed a bug.
    pub fn time_constant_min(&self, emissivity: f64, h: f64) -> f64 {
        self.lumped(emissivity, h).time_constant().to_si() / 60.0
    }

    /// Time to reach 63.2 % of the settled rise, **measured from the transient**.
    ///
    /// ⚠️ Use this, not [`Part::time_constant_min`]. `LumpedMass::time_constant` is
    /// `C/(hA)` — convection only — so it reports the same number whatever the
    /// emissivity, and radiation is the same order as still-air convection here.
    /// For the anodised girdle the quoted 53.0 min is really **25.6 min**.
    ///
    /// M18's first pass took the quoted figure at face value and built the
    /// "the pack dies before the girdle heats" mechanism on it. That is this
    /// project's oldest recurring error — a nominal figure standing in for a
    /// measured one — arriving via a dependency this time.
    pub fn effective_time_constant_min(&self, emissivity: f64, h: f64, watts: f64) -> f64 {
        let settled = self.equilibrium(emissivity, h, watts);
        let target = 0.632 * settled;
        let mut m = self.lumped(emissivity, h);
        let mut bus = Exchange::new();
        for k in 0..2_000_000 {
            bus.publish(HEAT, watts);
            m.step(Time::s(k as f64), Time::s(1.0), &mut bus).expect("step");
            if m.rise().to_si() >= target {
                return (k + 1) as f64 / 60.0;
            }
        }
        f64::INFINITY
    }

    /// `hL/k` — whether one temperature was an honest description.
    pub fn biot(&self, h: f64) -> f64 {
        self.lumped(EMIS_POLISHED, h).biot_number()
    }

    /// Steady rise from `LumpedMass::equilibrium_rise`.
    ///
    /// ⚠️ Renamed from `equilibrium_no_radiation` on the dualis 0.3 upgrade. That
    /// name described 0.2's behaviour and became false when upstream
    /// [#1](https://github.com/YounghyeonPark/dualis/issues/1) landed — it now solves
    /// `P = hA·ΔT + εσA((Tₐ+ΔT)⁴ − Tₐ⁴)` and agrees with the stepped transient to
    /// three figures. A function whose name outlives its behaviour is worse than no
    /// helper, so the emissivity is explicit here too.
    pub fn equilibrium_quoted(&self, emissivity: f64, h: f64, watts: f64) -> f64 {
        self.lumped(emissivity, h)
            .equilibrium_rise(Power::w(watts))
            .to_si()
    }

    /// Run this part off a real pack until it goes flat, under a conservation audit.
    ///
    /// ⚠️ **This is the version to trust.** `rise_after` hand-steps a lumped mass for
    /// a duration taken from elsewhere; this couples the pack and the girdle as two
    /// domains on one bus, so the runtime is emergent and every joule is accounted.
    /// [`Simulation::advance`] returns a [`Violation`] if the books do not balance —
    /// which is the check M18's first pass did not have.
    pub fn discharge(
        &self,
        emissivity: f64,
        h: f64,
        battery_wh: f64,
        total_w: f64,
        heat_w: f64,
    ) -> Result<Discharge, Violation> {
        let dt = Time::s(1.0);
        let mut sim = Simulation::new(Schedule::Staggered)
            .with(Battery::new(battery_wh, total_w, heat_w))
            .with(self.lumped(emissivity, h));

        let mut minutes = 0.0;
        for k in 0..200_000 {
            sim.advance(dt)?; // audits energy; propagates a Violation if it fails
            minutes = (k + 1) as f64 * dt.to_si() / 60.0;
            if sim
                .domain_as::<Battery>("battery")
                .is_some_and(Battery::is_flat)
            {
                break;
            }
        }
        let rise_k = sim
            .domain_as::<LumpedMass>("part")
            .map(|m| m.rise().to_si())
            .unwrap_or(f64::NAN);
        let eq = self.equilibrium(emissivity, h, heat_w);
        Ok(Discharge {
            minutes,
            rise_k,
            fraction_of_equilibrium: rise_k / eq,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::from_power_py::*;
    use super::*;

    #[test]
    fn lumped_approximation_is_honest_for_both_parts() {
        // Under ~0.1 the body has no internal gradient this model cannot see.
        assert!(Part::motor().biot(STILL_AIR_H) < 0.1);
        assert!(Part::girdle().biot(STILL_AIR_H) < 0.1);
    }

    #[test]
    fn centralising_the_motors_costs_rejection_area() {
        // P1's first measured THERMAL cost: the girdle skin is smaller than the six
        // motor surfaces it swallows. Everywhere else P1 has paid off.
        let six_motors = 6.0 * Part::motor().area_m2;
        let girdle = Part::girdle().area_m2;
        assert!(girdle < six_motors);
        let lost = 1.0 - girdle / six_motors;
        assert!((0.30..0.45).contains(&lost), "area lost was {lost}");
    }

    #[test]
    fn the_quoted_time_constant_now_tracks_emissivity_and_stays_conservative() {
        // Rewritten on the dualis 0.3 upgrade, and the inversion is the point.
        //
        // It used to assert that `time_constant` IGNORED emissivity and overstated
        // the real figure by ~2x — upstream #1, which this project reported after
        // being caught by it (see the M18 correction). 0.3 carries a linearised
        // radiative term, so the assertion flips shape: tau must now MOVE with
        // emissivity, and stay slightly conservative because the linearisation is
        // taken at ambient rather than at the operating point.
        let g = Part::girdle();
        let w = 6.0 * TROT_W;
        let q_pol = g.time_constant_min(EMIS_POLISHED, STILL_AIR_H);
        let q_ano = g.time_constant_min(EMIS_ANODISED, STILL_AIR_H);
        assert!(q_ano < q_pol * 0.75, "tau must track emissivity: {q_pol} vs {q_ano}");

        for (e, quoted) in [(EMIS_POLISHED, q_pol), (EMIS_ANODISED, q_ano)] {
            let measured = g.effective_time_constant_min(e, STILL_AIR_H, w);
            assert!(measured < quoted, "measured {measured} >= quoted {quoted}");
            assert!(quoted < measured * 1.25, "quoted {quoted} too far over {measured}");
        }
        // And the M18 correction still stands, now on upstream's own numbers:
        // a bare girdle outlasts the pack, an anodised one does not.
        assert!(g.effective_time_constant_min(EMIS_POLISHED, STILL_AIR_H, w) > TROT_RUNTIME_MIN);
        assert!(g.effective_time_constant_min(EMIS_ANODISED, STILL_AIR_H, w) < TROT_RUNTIME_MIN);
    }

    #[test]
    fn anodised_is_safe_on_its_own_merits_not_on_the_battery_running_out() {
        // Which is the point: once the quoted tau is corrected, the polished case is
        // the one leaning on the coincidence, and anodising removes that lean.
        let g = Part::girdle();
        assert!(AMBIENT_C + g.equilibrium(EMIS_ANODISED, STILL_AIR_H, 6.0 * TROT_W) < 80.0);
    }

    #[test]
    fn continuous_trot_on_a_polished_girdle_is_too_hot() {
        // The result that stops R5 being "comfortable": remove the battery limit
        // and a bare girdle settles past 100 C.
        let rise = Part::girdle().equilibrium(EMIS_POLISHED, STILL_AIR_H, 6.0 * TROT_W);
        assert!(AMBIENT_C + rise > 100.0, "settled at {} C", AMBIENT_C + rise);
    }

    #[test]
    fn anodising_is_worth_more_than_thirty_kelvin() {
        // Radiation is the same order as still-air convection here, so emissivity
        // 0.09 -> 0.90 is the cheapest lever available.
        let g = Part::girdle();
        let polished = g.equilibrium(EMIS_POLISHED, STILL_AIR_H, 6.0 * TROT_W);
        let anodised = g.equilibrium(EMIS_ANODISED, STILL_AIR_H, 6.0 * TROT_W);
        assert!(polished - anodised > 30.0, "gain was {}", polished - anodised);
        assert!(AMBIENT_C + anodised < 80.0);
    }

    #[test]
    fn standing_without_the_brake_is_the_worst_thermal_case() {
        // Reinforces ADR-0021 from a direction it never checked: a cable can only
        // pull, so holding a stance draws MORE per motor than trotting does.
        assert!(STAND_W > TROT_W);
        let g = Part::girdle();
        assert!(
            g.equilibrium(EMIS_POLISHED, STILL_AIR_H, 6.0 * STAND_W)
                > g.equilibrium(EMIS_POLISHED, STILL_AIR_H, 6.0 * TROT_W)
        );
    }

    #[test]
    fn motor_mass_uncertainty_moves_tau_but_not_equilibrium() {
        // OPEN_RISKS R1 is a heat-CAPACITY uncertainty, not a heat-REJECTION one:
        // it changes how fast you get there, never where you end up.
        let light = Part::motor();
        let heavy = Part { mass_kg: 0.200, ..light };
        assert!(
            heavy.time_constant_min(EMIS_POLISHED, STILL_AIR_H)
                > light.time_constant_min(EMIS_POLISHED, STILL_AIR_H)
        );
        assert!(
            (heavy.equilibrium_quoted(EMIS_POLISHED, STILL_AIR_H, TROT_W)
                - light.equilibrium_quoted(EMIS_POLISHED, STILL_AIR_H, TROT_W))
            .abs()
                < 1e-9
        );
    }
}

#[cfg(test)]
mod audited {
    use super::from_power_py::*;
    use super::*;

    const BATTERY_WH: f64 = 42.0; // power.py: 0.300 kg * 175 Wh/kg * 0.80
    const TOTAL_W: f64 = 83.5607; // whole-robot trot draw
    const GIRDLE_HEAT_W: f64 = 6.0 * TROT_W;

    #[test]
    fn the_energy_books_balance_under_the_kernel_audit() {
        // The check M18's first pass did not have. `advance` refuses if the summed
        // ledgers move, so a Violation here means the model leaks or invents energy.
        let d = Part::girdle()
            .discharge(EMIS_ANODISED, STILL_AIR_H, BATTERY_WH, TOTAL_W, GIRDLE_HEAT_W)
            .expect("conservation audit must pass");
        assert!(d.minutes > 0.0 && d.rise_k > 0.0);
    }

    #[test]
    fn runtime_is_emergent_and_agrees_with_power_py() {
        // Nothing tells the simulation how long to run: it stops when 42 Wh at
        // 83.6 W is gone. Landing on power.py's number is a real cross-check.
        let d = Part::girdle()
            .discharge(EMIS_ANODISED, STILL_AIR_H, BATTERY_WH, TOTAL_W, GIRDLE_HEAT_W)
            .unwrap();
        assert!(
            (d.minutes - TROT_RUNTIME_MIN).abs() < 0.2,
            "emergent {} min vs power.py {TROT_RUNTIME_MIN}",
            d.minutes
        );
    }

    #[test]
    fn the_pack_not_the_design_is_what_limits_the_temperature() {
        // The M18 headline, now a property of a coupled run rather than two numbers
        // compared by hand: the girdle is still climbing when the pack dies.
        for e in [EMIS_POLISHED, EMIS_ANODISED] {
            let d = Part::girdle()
                .discharge(e, STILL_AIR_H, BATTERY_WH, TOTAL_W, GIRDLE_HEAT_W)
                .unwrap();
            assert!(
                d.fraction_of_equilibrium < 0.85,
                "reached {:.0}% of equilibrium — the pack is no longer the limit",
                100.0 * d.fraction_of_equilibrium
            );
        }
    }
}

#[cfg(test)]
mod the_audit_has_teeth {
    use super::*;
    use dualis_units::Time;

    /// A pack that publishes heat but forgets to write it off its own books.
    ///
    /// Exists to prove the audit is load-bearing. An audit that cannot fail is
    /// decoration, and this project has been burned by exactly that shape of
    /// reassurance before — a check that returned a comforting number because it
    /// was silently skipping the case it was meant to catch.
    struct LeakyBattery {
        name: String,
        emitted_j: f64,
    }

    impl Domain for LeakyBattery {
        fn name(&self) -> &str {
            &self.name
        }
        fn kind(&self) -> Kind {
            Kind::Evolving
        }
        fn max_stable_dt(&self, _now: Time) -> Time {
            Time::s(f64::INFINITY)
        }
        fn step(&mut self, _t: Time, dt: Time, bus: &mut Exchange) -> Result<(), Violation> {
            let j = 21.0 * dt.to_si();
            self.emitted_j += j;
            bus.publish(HEAT, j); // ... and never debits itself for it
            Ok(())
        }
        fn ledger(&self) -> Ledger {
            Ledger::new().with(quantity::ENERGY, 0.0) // the lie
        }
    }

    #[test]
    fn inventing_energy_is_refused() {
        let mut sim = Simulation::new(Schedule::Staggered)
            .with(LeakyBattery {
                name: "leaky".to_string(),
                emitted_j: 0.0,
            })
            .with(Part::girdle().lumped(EMIS_ANODISED, STILL_AIR_H));

        let mut failed = false;
        for _ in 0..100 {
            if sim.advance(Time::s(1.0)).is_err() {
                failed = true;
                break;
            }
        }
        assert!(
            failed,
            "the conservation audit accepted energy from nowhere — it is decoration"
        );
    }
}
