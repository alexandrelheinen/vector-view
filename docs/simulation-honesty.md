# Simulation honesty guideline

This project visualizes **real** iCub contact forces in Gazebo. Demos, docs images, and comparison screenshots must reflect what the simulation actually does.

## Rule

**Do not add fake assets to the simulation or to captured output.**

When asked to **solve** a problem:

- **Resolve it** in the sim, plugin, world, scripts, or build, **or**
- **Report that you cannot solve it yet**, with the specific blocker.

Pretending something works when it does not is unacceptable in this repository — regardless of intent, the effect is the same: misleading demos and docs.

## What counts as fake

| Not allowed | Why |
|-------------|-----|
| Post-processing arrows onto camera frames | Forces were not rendered by Vector View |
| Editing comparison PNGs to add contact visuals | Hides broken capture or physics |
| Decorative geometry not tied to contact data | Misrepresents force direction/magnitude |
| Empty plots presented as success | Hides missing topics or timing bugs |
| "Fixing" screenshots instead of the sim | Users trust images as ground truth |

## What is allowed

- Force arrows from Vector View (markers in GUI, geometry on links in headless when needed)
- Plots from real `/vectorview/...` contact topics
- Capture scripts that only **record** sim output
- Incomplete work **labeled** as incomplete, with known limitations documented

## Headless vs GUI

Gazebo Harmonic headless cameras may not draw marker-service arrows even when markers exist in topic lists. The honest responses are:

1. Implement a sim-side visual that headless rendering actually draws (e.g. link geometry updated from contact forces), **or**
2. Document that automated capture cannot yet match the interactive GUI, **or**
3. Use interactive GUI capture only when that path is verified — not painted overlays

## For agents and contributors

Cursor agents: see `.cursor/rules/simulation-honesty.mdc` (always applied).

Humans: follow this document when changing `assets/`, `src/plugin/`, `scripts/`, or `docs/images/`.
