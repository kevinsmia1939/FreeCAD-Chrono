import FreeCAD
import FreeCADGui

from PySide.QtWidgets import QMessageBox


def _mod_path():
    from .. import ChronoUtils
    return ChronoUtils.MOD_PATH


def _icons_path():
    from .. import ChronoUtils
    return ChronoUtils.ICONS_PATH


def _selected_active_rigid_bodies():
    """Return selected enabled RigidBody objects whose BodyType is Dynamic."""
    result = []
    for obj in FreeCADGui.Selection.getSelection():
        if (hasattr(obj, "Proxy")
                and type(obj.Proxy).__name__ == "RigidBodyFeature"
                and getattr(obj, "BodyType", None) in ("Dynamic", "Active")
                and getattr(obj, "Enabled", True)):
            result.append(obj)
    return result


class AddLauncherCommand:
    def GetResources(self):
        import os
        return {
            "Pixmap": os.path.join(_icons_path(), "ChronoLauncher.svg"),
            "MenuText": "Add Velocity Launcher",
            "ToolTip": (
                "Add a Velocity Launcher to the selected dynamic rigid body.\n"
                "The body stays frozen until LaunchTime, then receives an\n"
                "instantaneous velocity impulse and behaves normally afterwards.\n\n"
                "Select a Dynamic Rigid Body in the tree first."
            ),
        }

    def IsActive(self):
        return (FreeCAD.ActiveDocument is not None
                and len(_selected_active_rigid_bodies()) > 0)

    def Activated(self):
        from ..objects.ChronoContainer import find_container
        from ..objects.ChronoLauncher import make_launcher

        rbs = _selected_active_rigid_bodies()
        if not rbs:
            QMessageBox.warning(
                None,
                "No Dynamic Rigid Body Selected",
                "Select one or more Dynamic Rigid Body objects in the tree view,\n"
                "then run Add Velocity Launcher.",
            )
            return

        container = find_container()
        if container is None:
            QMessageBox.warning(
                None,
                "No Physics Container",
                "Create a Physics Container first.",
            )
            return

        FreeCAD.ActiveDocument.openTransaction("Add Velocity Launcher")
        for rb in rbs:
            make_launcher(rb, container=container)
        FreeCAD.ActiveDocument.commitTransaction()


FreeCADGui.addCommand("Chrono_AddLauncher", AddLauncherCommand())
