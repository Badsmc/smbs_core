# smbs_core

**Headless engine for automatic bend sequence planning in sheet metal fabrication.**

`smbs_core` takes a STEP model of a folded part, extracts real bends and
panel topology, finds a physically feasible bend sequence with given
tooling and machine, validates every step against real B-Rep geometry,
and outputs JSON with an honest status.

The engine runs **without FreeCAD**. FreeCAD is used only as a frontend
for visualization and simulation at the final stage.

---

## Table of Contents

- [Why this exists](#why-this-exists)
- [Architectural principles](#architectural-principles)
- [Current status](#current-status)
- [Installation](#installation)
- [Quick start](#quick-start)
- [Data formats](#data-formats)
- [Project structure](#project-structure)
- [Development](#development)
- [Testing](#testing)
- [Limitations](#limitations)
- [Roadmap](#roadmap)
- [License](#license)

---

## Why this exists

Bend sequence planning is non-trivial. For a part with 8–10 bends, the
number of possible orders exceeds millions. Not every order is physically
feasible: the part may collide with the punch, die, machine frame,
previously bent panels, or the backgauge.

`smbs_core` solves this automatically:

1. Reads a folded part from STEP.
2. Detects real bends, panels, and topology.
3. Loads tooling (punch, die, machine frame) from `.FCStd` + sidecar.
4. Positions tooling in a unified machine coordinate system.
5. Searches for a sequence, validating each step physically.
6. Returns JSON with an honest status for each step and the whole sequence.

---

## Architectural principles

### Headless-first

The engine does not depend on FreeCAD and does not import `FreeCAD`,
`Part`, or `FreeCADGui`. All geometry goes through the `GeometryBackend`
abstraction, implemented on top of `pythonocc-core`.

Benefits:

- **Testability** — unit tests without GUI and without a CAD application.
- **Speed** — the engine runs much faster without rendering.
- **Portability** — works on a server, in Docker, in CI.
- **Clean architecture** — FreeCAD becomes just a viewer.

### UNKNOWN ≠ VALID

This is a hard rule. If a check did not happen — missing geometry, OCC
failure, undefined topology — the result is `UNKNOWN`, not `VALID`.
`UNKNOWN` is never turned into `VALID` by a flag, a setting, or a
"fast mode".

The program must be better at saying **"I cannot confirm safety"**
than at showing a beautiful sequence that will collide with tooling.

### No hidden defaults

If data is missing — an error or `PARTIAL`/`UNKNOWN`. Not a "reasonable"
angle, thickness, or length. No fallback stubs in the working path.

### Swappable tooling

The user can replace the punch, die, or machine via a config JSON.
The engine recomputes the sequence and validation from scratch. The cache
is not reused between different tooling configurations.

### Parts of arbitrary complexity

The engine does not assume:

- the panel graph is a tree;
- all bends are along X/Y/Z axes;
- symmetry;
- absence of cutouts;
- a limit on the number of bends.

If a part is unsupported, the program reports specific diagnostics
instead of producing a "beautiful" but wrong solution.

---

## Current status

**Status: architectural prototype. Not ready for production use.**

The project is under active development. The architectural skeleton is
correct and stable, but the implementation is not complete.

| Subsystem | Status |
| :--- | :--- |
| `GeometryBackend` (ABC) | ✅ Defined |
| `PythonOCCBackend` | 🟡 Partially implemented |
| `MockBackend` | 🟡 For unit tests |
| STEP loading | 🟡 Basic |
| `.FCStd` + sidecar loading | 🟡 Basic |
| Bend extraction | 🔴 Needs work (angle sign, defaults) |
| Panel topology | 🔴 Needs implementation |
| Panel kinematics | 🔴 Needs work |
| Tooling positioning | 🔴 Simplified |
| Collision checking | 🔴 False `VALID` when data missing |
| Sequence search | 🟡 A*, DFS, Greedy present |
| Backgauge | 🟡 Basic calculation |
| CLI | 🟡 Basic |
| JSON schema v1.0 | 🟡 Needs validation |
| Tests | 🔴 Missing |
| Documentation | 🟡 In progress |

**What to expect now:**

- The pipeline can run on simple parts.
- Individual components can be used for experiments.
- You **cannot** trust an automatically produced sequence without
  independent verification.

**Estimated timeline:**

- Honest validator + basic tests: 2–3 weeks.
- Working headless engine on STEP: 4–6 weeks.
- Realistic tooling geometry + collisions: 6–10 weeks.
- Release candidate: 2–4 months.

---

## Installation

### Requirements

- Python 3.11 or higher.
- `pythonocc-core` (recommended via conda).
- Optional: FreeCAD 1.1.x — for generating `.FCStd` and visualization.

### Install via conda (recommended)

```bash
conda create -n smbs python=3.11
conda activate smbs
conda install -c conda-forge pythonocc-core
git clone https://github.com/Badsmc/smbs_core.git
cd smbs_core
pip install -e .
