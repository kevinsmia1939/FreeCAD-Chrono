"""Exercise a migrated emitter and custom convex-mesh body with Chrono."""

from pathlib import Path
import sys

import FreeCAD


ADDON = Path(__file__).resolve().parents[1]
if str(ADDON) not in sys.path:
    sys.path.insert(0, str(ADDON))

from freecad.Chrono.objects.ChronoContainer import find_container
from freecad.Chrono.objects.ChronoMesh import make_mesh_settings
from freecad.Chrono.simulation.ChronoSimulation import run_simulation


document = FreeCAD.openDocument(str(ADDON / "example" / "packed_bed.FCStd"))
try:
    container = find_container(document)
    assert container is not None
    settings = getattr(container, "MeshSettings", None)
    if settings is None:
        settings = make_mesh_settings(container)
    settings.MeshSize = 2.0
    emitter = container.Emitters[0]
    emitter.UseParticlesPerSecond = False
    emitter.Count = 3
    emitter.StartTime = 0.0
    emitter.EndTime = 0.04
    world = container.World
    world.EndTime = 0.08
    world.TimeStep = 0.02
    world.SubSteps = 2
    result = run_simulation(cpu_parallel=True)
    assert result is not None
    frames, _time_step, _speeds = result
    emitted = [name for name in frames[-1] if name.startswith("_ChEmit_")]
    assert len(emitted) == 3, emitted
    print("Chrono custom mesh and emitter simulation: PASS")
finally:
    FreeCAD.closeDocument(document.Name)
