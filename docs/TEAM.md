# T.O.M.C.A.T. — Sub-Agent Team

Project-scoped Claude Code sub-agents live in [`.claude/agents/`](../.claude/agents).
Each specialist owns one subsystem; one generalist lead coordinates them.

## Roster

| Agent | Role | Owns | Key tools |
|-------|------|------|-----------|
| **tomcat-lead** | Generalist **manager** — plans, decomposes, delegates, keeps docs/ADRs coherent, integrates & reviews | whole system, `docs/` coherence | delegation (Task), all core tools |
| **tomcat-kinematics** | Kinematics / dynamics / tendon map / gait | `kinematics/` (`tomcat_kin`) | Python, pytest, Bash |
| **tomcat-firmware** | Real-time motor/tension control, safety, comms | `firmware/` | Bash, build/flash |
| **tomcat-electronics** | KiCad schematics/PCB, drivers, power, sensing | `electronics/` | `kicad-cli` skill, Bash |
| **tomcat-mechanical** | Leg/joint/spine geometry, tendon routing, BOM | `mechanical/` | docs, CAD notes |
| **tomcat-research** | Literature, verified numbers, citations | `docs/REFERENCES.md`, `docs/LITERATURE_REVIEW.md` | `deep-research` skill, web |

## How to use them

- **Let the lead drive multi-part work:** ask `tomcat-lead` for anything that
  spans subsystems; it decomposes the work and dispatches specialists (in
  parallel when tasks are independent), then integrates and reports.
- **Call a specialist directly** for focused work in its area (e.g. "have
  tomcat-kinematics add the spine model").
- Invoke via the Agent tool's `subagent_type`, or just name the agent in a
  request (e.g. *"use tomcat-firmware to draft the tension control loop"*).

## Delegation map (who hands off what)

```
                         ┌───────────────┐
                         │  tomcat-lead  │  (plans, integrates, owns ADRs)
                         └───┬───┬───┬───┘
        ┌────────────────────┘   │   └────────────────────┐
        ▼                        ▼                        ▼
 tomcat-research ──numbers──▶ tomcat-kinematics ──setpoints──▶ tomcat-firmware
        │                        │                             ▲
        │                        ▼ geometry/moment arms        │ elec limits
        └──biomech──▶ tomcat-mechanical ──placement──▶ tomcat-electronics
```

## Conventions all agents follow
- Ground every decision in `docs/PRINCIPLES.md` (P1 tendon-drive, P2 whole-body
  curvature) and the ADR log; propose an ADR update rather than diverging.
- Use **verified** numbers from `docs/LITERATURE_REVIEW.md`; label assumptions
  and placeholders as such.
- Stay in your lane — raise cross-subsystem needs to the lead.
- Don't commit or push unless the user asks.

> Models are inherited from the session by default. To specialize cost/latency,
> add a `model:` field (e.g. `sonnet`) to an agent's frontmatter.
