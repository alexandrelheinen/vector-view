# SRE / incident-report principles (enrichment)

Condensed from [Google SRE postmortem culture](https://sre.google/sre-book/postmortem-culture/),
incident postmortem practice guides, and blameless review norms. Apply at **T3+**.

## Evidence over narrative

Every significant claim needs a **citation**:

| Weak | Strong |
| --- | --- |
| “Physics was flaky” | “8/8 unseeded runs: 3 timed out at 300 s (`race_duration_s=300.0`)" |
| “CI failed” | Link + quoted log line with exit code |
| “Much faster now” | “55.6 s vs 300 s timeout; p50 race duration 56 s over 8 trials” |

Write the **timeline from sources first** (logs, metrics, commands). Write
**analysis second**. Never mix sourced facts and interpretation in one sentence
without labeling.

## Blameless, systemic framing

Ask what **conditions** allowed the failure — not who broke it.

- Good: “Export path resampled timeout logs without checking `both_reached_goal`”
- Bad: “Agent shipped a bad fix”

If root cause lands on a person, keep asking **why the system had no guardrail**
(honest CI gate, deterministic seed, export validation).

## Recurrence field (mandatory at T3+)

Answer explicitly:

1. Have we seen this failure mode before?
2. What happened to the last fix?
3. Did the new fix address **mechanism** or only **symptom**?

Example (Dubins): symptom fix = reject fake clips; mechanism fix = seed planner RNG.

## Action items — five closure fields

Each follow-up needs:

| Field | Example |
| --- | --- |
| **Owner** | agent / maintainer / team |
| **Verb** | add, remove, update, test, deploy — not “investigate” |
| **Measurable outcome** | “render-dubins job exit 0 on tag push” |
| **Tracker** | PR #, issue #, Asana task |
| **Due** | date or release event |

## Root cause depth

Prefer **Five Whys** until you hit a guardrail or design decision:

1. Why did CI fail? → physics race timeout
2. Why timeout? → untrackable plan
3. Why untrackable? → unseeded ARC sample tree
4. Why unseeded in export? → seed only in tests, not render path
5. Why no guardrail? → export did not require `both_reached_goal` until `8b2cf32`

## When to write a full report

Triggers (adapted from SRE postmortem policy):

- Release / showcase pipeline failure
- Recurring CI flake after a “fix”
- Maintainer distrust (“you fixed one problem, created another”)
- Stochastic or visual correctness (sim, render, ML)
- Cross-module or multi-PR arc

**Not** required: typos, format-only, single-line deterministic fixes (T0–T1).

## Pre-mortem (optional, T4 refactors)

Before large changes, briefly document:

- What could fail?
- What metric would prove success?
- What would a misleading “success” look like? (e.g. time-compressed fake clip)
