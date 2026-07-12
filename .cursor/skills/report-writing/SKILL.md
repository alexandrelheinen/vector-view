---
name: report-writing
description: >-
  Write proportional proof reports when creating or updating pull requests,
  completing cloud-agent tasks, or when the maintainer asks whether work
  actually works. Load on every new PR creation, PR update, release fix, or
  long-running bug investigation. Depth scales with PR size and problem
  difficulty — one sentence for trivial tuning, engineer-grade evidence
  (metrics, graphs, renders) for large refactors and chronic failures.
metadata:
  author: fret
  version: "1.1.0"
  policy: AGENTS.md#proof-reports-mandatory
---

# Report writing — proof that work works

Agents must **prove** outcomes, not assert them. The report is the deliverable
that lets the maintainer merge or reject **without re-running your investigation**.

## When to use (mandatory triggers)

Load this skill when **any** of these occur:

- Creating or updating a **pull request**
- Finishing a **cloud-agent** task that implies a PR
- User asks: “does it work?”, “convince me”, “show evidence”, “report”
- **Release / CI / showcase** pipeline failure
- User says a prior fix **created another problem** or names a known failure mode
  (e.g. “time-compression”, “flake”, “psychological manipulation”)
- **Stochastic** behavior (RNG, timing, CI runner variance) or **visual/export**
  output (video, MuJoCo, plots) where logs can mislead

Repo policy: [AGENTS.md](../../../AGENTS.md#proof-reports-mandatory),
[CONTRIBUTING.md](../../../CONTRIBUTING.md) (Rules for AI agents → Communication).

---

## Step 0 — Pick tier (before writing)

Estimate tier from **four inputs**:

| Input | Ask |
| --- | --- |
| **Diff size** | Files changed, LOC, modules touched |
| **Blast radius** | CI, release, sim, public API, data |
| **Problem age** | First attempt vs chronic / multi-PR arc |
| **Observability** | Deterministic pass/fail vs stochastic / visual |

### Tier rubric

| Tier | When | Deliverable |
| --- | --- | --- |
| **T0** | Typo, comment, format-only, obvious one-liner | **One sentence** + gates passed |
| **T1** | ≤3 files, ≤~80 LOC, deterministic | **2–4 bullets** (problem, fix, command, result) |
| **T2** | Multi-file fix/feature, CI repair, new tests | **Problem / Fix / Verification** + gate output |
| **T3** | Release blocker, flake, sim/render pipeline, cross-module | **Evidence report** + metric table + chart or before/after |
| **T4** | Refactor, architecture, chronic bug, tag release, maintainer distrust | **Engineer-grade** — full layer stack + artifacts + recurrence |

### Escalate +1 tier when

- A previous agent PR claimed success but CI/user still red
- Behavior is **stochastic** (planner RNG, race, CI speed)
- **Export/resampling** can make failure look like success
- Maintainer already knows the trick — prove **mechanism**, not symptoms

**Never** ship T0 for a T3+ problem.

Templates per tier: [references/report-template.md](references/report-template.md)

---

## Step 1 — Separate timeline from analysis (T2+)

SRE best practice ([Google postmortem culture](https://sre.google/sre-book/postmortem-culture/)):

1. **Timeline** — sourced facts only (log lines, timestamps, command exit codes)
2. **Analysis** — interpretation, clearly labeled
3. **Recurrence** — have we seen this before? what happened to the last fix?

Do **not** mix “the database was slow” (vague) with “P99 latency 2000 ms at 23:47:12” (evidence). See [references/sre-principles.md](references/sre-principles.md).

---

## Step 2 — Evidence layers (T3–T4)

Build in order. Each layer answers one skeptic question.

| Layer | Content | Question answered |
| --- | --- | --- |
| **1. Reproduce verbatim** | CI URL, quoted exception, matching local repro | “Did you hit *my* failure?” |
| **2. Control variable** | Change one knob (seed, timeout, flag); same env as CI | “Environment or code?” |
| **3. State / time domain** | Trajectories, bar charts, success rates over N trials; **viewer clock vs sim clock** if resampling | “What actually happened?” |
| **4. Pixel proof** | Screenshots / MuJoCo frames via **same script as CI** | “Would I see this in the product?” |
| **5. Fix chain** | `root cause → symptom → wrong fix → correct fix → verification` | “What should I merge?” |

### Honest failure vs deceptive success

When export pipelines **resample** long failed runs into short clips, the viewer
clock diverges from physics clock — apparent speedup without real success.
**Rejecting** such exports is correct; **seeding/tuning** so physics actually
finishes is the complementary fix. Never swap one deception for another (e.g.
skip export checks to green CI).

---

## Step 3 — Exemplar (T4): Dubins physics release

Maintainer feedback: *“You fixed one problem, you created another”* — honest
rejection of fake clips exposed unseeded planner flake.

| Layer | What we showed |
| --- | --- |
| 1 | CI: `race_duration_s=300.0`, `max_cross_track_error_m=67.80` |
| 2 | Same physics controller; only `planner_rng_seed` changes |
| 3 | 8×8 trial success rate; XY trajectories; viewer vs sim time (8.6×) |
| 4 | MuJoCo overview: goal x≈74 m only on seeded success |
| 5 | `unseeded flake → time-compress export → reject clips → seed RNG` |

Repro script (when on branch): `scripts/evidence_dubins_physics_seed.py`  
Artifacts: `/opt/cursor/artifacts/dubins_physics_evidence/*.png`

---

## Step 4 — Where to publish

| Audience | Location |
| --- | --- |
| Maintainer (chat) | Final turn — **verdict first**, then layers; embed images |
| PR reviewers | PR body **## Verification** (+ **## Problem** at T2+) |
| Chronic issues | `docs/postmortems/<topic>.md` only if user asks |

### PR body skeleton (T1+)

```markdown
## Problem
[One paragraph or verbatim CI quote]

## Fix
[What changed and why — mechanism, not symptom only]

## Verification
- Commands: `...`
- Results: exit codes, counts, timings
- [T3+] Charts/screenshots or artifact paths

## Recurrence
[Prior attempts / what this fix adds beyond last PR]
```

---

## Step 5 — Follow-ups (T3+)

Each action item needs five fields (incident postmortem norm):

**Owner** · **Verb** (add/remove/update/test/deploy) · **Measurable outcome** · **Tracker** · **Due**

Avoid vague verbs: “investigate”, “explore”, “monitor” without a metric.

---

## Anti-patterns

| Do not | Do instead |
| --- | --- |
| “Should work” / “CI will pass” | Paste command + exit code |
| Fix without showing failure mode | Reproduce first |
| Single lucky run for flake | N trials + success rate |
| Log wall | One chart or table |
| Symptom-only patch on chronic bug | Recurrence + mechanism |
| Time-compress failed sim into showcase | Reject + fix root cause |

---

## Commands (FRET)

```bash
bash scripts/check/formatting.sh
bash scripts/check/types.sh
bash scripts/check/pre_push.sh --skip-ros
python3 -m pytest tests/ --ignore=tests/integration -p no:launch_testing -p no:launch_ros
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl   # headless MuJoCo / release renders
```

---

## Progressive references

Load only when needed:

- [references/report-template.md](references/report-template.md) — copy-paste T0–T4 bodies
- [references/sre-principles.md](references/sre-principles.md) — evidence, blameless, Five Whys, recurrence

---

## Maintainer: PR-open automation (optional)

Skills are discovered from `.cursor/skills/`; GitHub does **not** auto-run them.

To enforce on every PR:

1. [Cursor Automations](https://cursor.com/docs/cloud-agent/automations) → trigger **PR opened**
2. Prompt: “Follow `report-writing` skill; assess tier from diff + issue history; post proportional **Verification** comment.”

Or invoke manually: `/report-writing` or `@report-writing` in Agent chat.

---

## Maintainer: create / edit this skill

| Action | How |
| --- | --- |
| **Edit** | Change `.cursor/skills/report-writing/SKILL.md`; keep `name` = folder name |
| **Scaffold new skill** | Agent chat → `/create-skill` |
| **Manual layout** | `.cursor/skills/<name>/SKILL.md` + optional `scripts/`, `references/`, `assets/` |
| **Also loaded from** | `.agents/skills/` (same layout), `~/.cursor/skills/` (global) |
| **Force manual only** | `disable-model-invocation: true` in frontmatter |

Docs: [cursor.com/docs/skills](https://cursor.com/docs/skills)
