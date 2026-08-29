"""Run a short migrated example through the complete simulation pipeline."""

from pathlib import Path
import sys

import FreeCAD


ADDON = Path(__file__).resolve().parents[1]
if str(ADDON) not in sys.path:
    sys.path.insert(0, str(ADDON))

from freecad.Chrono.objects.ChronoContainer import find_container
from freecad.Chrono.objects.ChronoMesh import make_mesh_settings
from freecad.Chrono.simulation.ChronoSimulation import run_simulation


document = FreeCAD.openDocument(str(ADDON / "example" / "example.FCStd"))
try:
    container = find_container(document)
    assert container is not None, "Chrono container was not restored"
    settings = getattr(container, "MeshSettings", None)
    if settings is None:
        settings = make_mesh_settings(container)
    settings.Mesher = "Standard"
    settings.MeshSize = 2.0
    world = container.World
    world.EndTime = 0.1
    world.TimeStep = 0.02
    world.SubSteps = 2
    result = run_simulation(cpu_parallel=True)
    assert result is not None, "Full Chrono simulation returned no result"
    frames, time_step, speed_frames = result
    assert len(frames) == 6, len(frames)
    assert len(speed_frames) == len(frames)
    assert abs(time_step - 0.02) < 1e-12
    print("Migrated Chrono example simulation: PASS")
finally:
    FreeCAD.closeDocument(document.Name)
