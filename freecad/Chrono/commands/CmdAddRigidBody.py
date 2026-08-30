import FreeCAD
import FreeCADGui

from PySide.QtWidgets import QMessageBox


def _mod_path():
    from .. import ChronoUtils
    return ChronoUtils.MOD_PATH


def _icons_path():
    from .. import ChronoUtils
    return ChronoUtils.ICONS_PATH


def _has_shape(obj):
    return hasattr(obj, "Shape") and obj.Shape is not None


def _selection_has_shapes():
    return any(_has_shape(o) for o in FreeCADGui.Selection.getSelection())


def _require_container():
    """Return the container or show an error and return None."""
    from ..objects.ChronoContainer import find_container
    c = find_container()
    if c is None:
        QMessageBox.warning(
            None,
            "No Physics Container",
            "Create a Physics Container first.\n\n"
            "Use: Chrono Physics → Create Physics Container",
        )
    return c


class AddDynamicBodyCommand:
    def GetResources(self):
        import os
        return {
            "Pixmap": os.path.join(_icons_path(), "AddActiveBody.svg"),
            "MenuText": "Add Dynamic Rigid Body",
            "ToolTip": (
                "Create a dynamic Chrono ChBody from the selected solid.\n"
                "An App::Link clone is created inside the container;\n"
                "the original solid is not modified."
            ),
        }

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None and _selection_has_shapes()

    def Activated(self):
        from ..objects.RigidBody import make_rigid_body
        container = _require_container()
        if container is None:
            return
        sel = [o for o in FreeCADGui.Selection.getSelection() if _has_shape(o)]
        if not sel:
            return
        FreeCAD.ActiveDocument.openTransaction("Add Dynamic Rigid Body")
        for obj in sel:
            make_rigid_body(obj, "Dynamic", container=container)
        FreeCAD.ActiveDocument.commitTransaction()


class AddFixedBodyCommand:
    def GetResources(self):
        import os
        return {
            "Pixmap": os.path.join(_icons_path(), "AddPassiveBody.svg"),
            "MenuText": "Add Fixed Rigid Body",
            "ToolTip": (
                "Create a fixed Chrono ChBody from the selected solid.\n"
                "An App::Link clone is created inside the container;\n"
                "the original solid is not modified."
            ),
        }

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None and _selection_has_shapes()

    def Activated(self):
        from ..objects.RigidBody import make_rigid_body
        container = _require_container()
        if container is None:
            return
        sel = [o for o in FreeCADGui.Selection.getSelection() if _has_shape(o)]
        if not sel:
            return
        FreeCAD.ActiveDocument.openTransaction("Add Fixed Rigid Body")
        for obj in sel:
            make_rigid_body(obj, "Fixed", container=container)
        FreeCAD.ActiveDocument.commitTransaction()


FreeCADGui.addCommand("Chrono_AddActiveBody", AddDynamicBodyCommand())
FreeCADGui.addCommand("Chrono_AddPassiveBody", AddFixedBodyCommand())
