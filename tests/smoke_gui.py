"""GUI registration smoke test for toolbar/menu feature parity."""

from pathlib import Path
import sys

import FreeCAD
import FreeCADGui
from PySide import QtCore, QtWidgets


ADDON = Path(__file__).resolve().parents[1]
if str(ADDON) not in sys.path:
    sys.path.insert(0, str(ADDON))

workbenches = FreeCADGui.listWorkbenches()
matches = [name for name, workbench in workbenches.items()
           if getattr(workbench, "MenuText", "") == "Chrono Physics"]
if not matches:
    import freecad.Chrono.init_gui  # noqa: F401
    workbenches = FreeCADGui.listWorkbenches()
    matches = [name for name, workbench in workbenches.items()
               if getattr(workbench, "MenuText", "") == "Chrono Physics"]
assert matches, "Chrono Physics workbench was not registered"

FreeCADGui.activateWorkbench(matches[0])
commands = set(FreeCADGui.listCommands())
expected = {
    "Chrono_CreateContainer",
    "Chrono_AddActiveBody",
    "Chrono_AddPassiveBody",
    "Chrono_AddMesh",
    "Chrono_Downgrade",
    "Chrono_AddLauncher",
    "Chrono_AddEmitter",
    "Chrono_AddDestroyRigidBody",
    "Chrono_AddObserver",
    "Chrono_RunSimulation",
}
missing = expected - commands
assert not missing, f"Missing Chrono GUI commands: {sorted(missing)}"

document = FreeCAD.openDocument(str(ADDON / "example" / "packed_bed.FCStd"))
try:
    container = document.getObject("Chrono")
    assert container is not None
    assert type(container.ViewObject.Proxy).__module__.startswith("freecad.Chrono")
    from freecad.Chrono.commands.CmdRunSimulation import SimulationPanel
    panel = SimulationPanel()
    threading_text = panel.threading_status_label.text()
    assert "CPU threading:" in threading_text, threading_text
    assert "Bullet collision fallback" in threading_text, threading_text
    assert "convex_hull" in threading_text, threading_text
    panel.reject()

    # The emitter count editor must not impose the former 10,000-particle cap.
    from freecad.Chrono.objects.ChronoEmitter import EmitterPanel
    emitter = container.Emitters[0]
    emitter_panel = EmitterPanel(emitter)
    assert emitter_panel.count_spin.maximum() == 2147483647
    emitter_panel.count_spin.setValue(1234567)
    emitter_panel._save()
    assert emitter.Count == 1234567
finally:
    FreeCAD.closeDocument(document.Name)
print("Chrono GUI workbench and all parity commands: PASS")

application = QtWidgets.QApplication.instance()
QtCore.QTimer.singleShot(100, application.quit)
