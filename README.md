# FreeCAD Chrono Physics Workbench

A rigid-body physics simulation workbench for FreeCAD powered by [Project Chrono](https://github.com/projectchrono/chrono) through PyChrono. It keeps the commands, document model, panels, and workflow of FreeCAD-BulletPhysics while using Chrono for rigid-body dynamics and contacts.

---

## Screenshots

Video tutorial in screenshot/tutorial.mp4

![Simulation overview — boxes and a sphere colliding mid-fall, Physics World properties visible in the Data tab](screenshot/overview.png)

![Collision boundaries — green wireframes show each body's collision envelope](screenshot/collision%20boundary.png)

![Rigid Body Summary table — editable per-body properties with bulk-apply controls](screenshot/summary%20table.png)

![Physics World Settings panel — gravity, damping, time step, solver, and mesh resolution](screenshot/world%20settings.png)

![Emitter object — emit rigid-body object](screenshot/emitter.png)

![Interestingly many research use Chrono Physics for packed bed simulation](screenshot/packed_bed.png)

---

## Features

- **Active & Passive rigid bodies** — active bodies are driven by physics; passive bodies act as static colliders (floors, walls, ramps)
- **Velocity Launcher** — Give bodies a velocity to throw at other active bodies
- **Original geometry is never modified** — the workbench drives `App::Link` clones, leaving your original solids untouched and fully editable at all times
- **Automatic collision shape detection** — spheres, cylinders, and boxes get exact analytic collision shapes; any other solid is tessellated into a convex-hull or concave mesh automatically
- **Animation timeline** — scrubber, transport controls (first / step back / play-pause / step forward / last), adjustable playback speed (0.1× – 8×), and loop toggle
- **Simulation cache** — results are saved to disk alongside your `.FCStd` file so the timeline is available immediately on reopening the panel without re-running the simulation
- **Rigid Body Summary table** — editable overview of all bodies (type, density, friction, mesh type, mesh resolution) with bulk-apply inputs for changing multiple bodies at once
- **Bake frame as new origin** — commit any timeline frame back to the original objects as the new rest position; fully undoable with Ctrl+Z
- **Physics World settings** — gravity magnitude and direction, simulation end time, frame time step, sub-steps (accuracy), solver iterations, and global mesh resolution, all stored in the document
- **Recursive downgrade** — Break down array of solids into individual components, useful when setting up a simulation with many solids as rigid body using array and need to separate them.

- **Emitter** — Emit multiple rigid-body objects in certain orientation or direction without manually duplicating rigid-body objects.

- **Destroy Rigid Body** — When touched, will be deleted/disabled. This can reduce unnecessary compute resources where rigid-body fell off the platform.
- **Observers and conditional events** — non-physical regions report body entry/exit, body type, speed, AND/OR conditions, delays, emitter start/stop events, and simulation stop conditions.
- **Multicore execution** — the CPU-parallel option selects Chrono's multicore collision system and assigns all available threads to Chrono and Eigen.
- **Analytics and settling controls** — stop when all active bodies settle, inspect speed data, and export the same cached animation/analytics data used by the original workflow.
---

## Requirements

| Requirement | Version |
|---|---|
| FreeCAD | 1.0 or later; tested with 26.3.0 weekly 2026-08-20 |
| Project Chrono / PyChrono | 10.0.0; tested with the openSUSE `python3-pychrono` package |
| Helper Python | Python matching the PyChrono package (Python 3.13 on current Tumbleweed) |

---

## Installation

### 1 — Install PyChrono

On openSUSE Tumbleweed, install the PyChrono RPM built with Project Chrono:

```bash
sudo zypper install python3-pychrono
```

The supplied FreeCAD AppImage embeds Python 3.11 while Tumbleweed PyChrono uses Python 3.13. The workbench detects this automatically and keeps `/usr/bin/python3.13` running as a local Chrono worker. To select another compatible interpreter:

```bash
export FREECAD_CHRONO_PYTHON=/path/to/python-with-pychrono
```

When FreeCAD and PyChrono use the same Python ABI, the workbench uses PyChrono directly without the worker.

### 2 — Install the workbench

**Option A — FreeCAD Addon Manager (recommended once listed)**

Open FreeCAD → Tools → Addon Manager → search for **Chrono Physics** → Install.

**Option B — Manual**

Clone this repository into your FreeCAD `Mod` folder:

```bash
# Linux / macOS
cd ~/.local/share/FreeCAD/Mod        # FreeCAD 1.x path
# or ~/.FreeCAD/Mod for older versions
git clone https://github.com/kevinsmia1939/FreeCAD-Chrono

# Windows
cd %APPDATA%\FreeCAD\Mod
git clone https://github.com/kevinsmia1939/FreeCAD-Chrono
```

Restart FreeCAD and select **Chrono Physics** from the workbench dropdown. For the FreeCAD build used during development, this repository is installed at `~/.local/share/FreeCAD/v26-3/Mod/FreeCAD-Chrono`.

---

## Quick Start

1. Open or create a FreeCAD document containing at least one solid.
2. Switch to the **Chrono Physics** workbench.
3. Click **Create Physics Container** — this adds a *Chrono Physics* folder to the model tree containing a *Physics World* settings object and a *Rigid Body Summary* table.
4. Select a solid in the 3D view, then click **Add Active Body** (moves with physics) or **Add Passive Body** (static collider).
5. Repeat step 4 for each solid you want to simulate.
6. Adjust physics properties in the **Physics World** object (gravity, end time, time step, etc.) and per-body properties (density, friction, restitution) via the model tree or the Rigid Body Summary table.
7. Click **Run Simulation** in the toolbar to open the simulation panel.
8. Click **Simulate** — a progress bar tracks the run. When it finishes the timeline slider is enabled.
9. Drag the slider or press **Play** to watch the animation.

The `example` directory contains migrated Chrono versions of both original `.FCStd` examples.

## Current differences from the Bullet backend

These are limitations of the current PyChrono integration, rather than general limitations of Chrono's C++ engine:

- PyChrono 10 does not expose the V-HACD helper bundled with PyBullet. A dynamic body explicitly set to a concave mesh therefore falls back to a single convex hull. Static concave triangle meshes remain exact.
- The installed bindings do not expose `ChSystemMulticoreNSC`. CPU parallelization still uses Chrono's `MULTICORE` collision system and threaded Chrono/Eigen core, but the NSC dynamics system itself is not replaced by the separate multicore-system class.
- PyBullet's per-body CCD controls do not have a direct equivalent in the bound Chrono API used here. Sub-steps remain the primary protection against very fast bodies tunnelling through thin geometry.
- Query-only observer boundary checks use exact FreeCAD point-in-solid containment plus a conservative bounding-radius proximity test. PyBullet can provide an exact `getClosestPoints` query for two disabled-response collision objects.
- Chrono requires an inertia tensor. Analytic boxes, spheres, and cylinders use their exact diagonal inertia; arbitrary meshes and compounds use a stable spherical estimate, which can produce somewhat different rotation from Bullet's inferred mesh inertia.
- With the supplied AppImage, pipe-based bridging adds some overhead compared with an in-process PyChrono module. It does not change simulation results and disappears when both programs use the same Python ABI.

## License

GPL-3.0-or-later — see [LICENSE](LICENSE).
