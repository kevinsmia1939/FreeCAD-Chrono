"""Exercise a migrated emitter and custom convex-mesh body with Chrono."""

from pathlib import Path
import sys

import FreeCAD


ADDON = Path(__file__).resolve().parents[1]
if str(ADDON) not in sys.path:
    sys.path.insert(0, str(ADDON))

from freecad.Chrono.objects.ChronoContainer import find_container
from freecad.Chrono.objects.ChronoMesh import make_mesh_settings
from freecad.Chrono.simulation.ChronoSimulation import (
    delete_simulation_cache,
    load_simulation_cache,
    run_simulation,
    save_simulation_cache,
)


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
    emitter.EndConditionType = "Time"
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
    assert len(emitter.GeneratedLinks) == 3

    save_simulation_cache(frames, _time_step, _speeds, document)
    assert load_simulation_cache(document) is not None
    assert delete_simulation_cache(document)
    assert load_simulation_cache(document) is None
    assert not emitter.GeneratedLinks
    assert not [obj for obj in document.Objects
                if obj.Name.startswith("_ChEmit_")]
    for group in (emitter.DeactivatedGroup, emitter.DynamicGroup,
                  emitter.FixedGroup):
        assert not group.Group

    # A link left behind before GeneratedLinks was updated (for example after
    # an interrupted simulation) is also part of the emitter cache state.
    orphan = document.addObject("App::Link", "_ChEmit_interrupted_1")
    orphan.setLink(emitter.TemplateObjects[0])
    orphan_name = orphan.Name
    assert delete_simulation_cache(document)
    assert document.getObject(orphan_name) is None

    # Clearing an already empty cache remains harmless.
    assert not delete_simulation_cache(document)
    print("Chrono emitter simulation and cache reset: PASS")
finally:
    FreeCAD.closeDocument(document.Name)
