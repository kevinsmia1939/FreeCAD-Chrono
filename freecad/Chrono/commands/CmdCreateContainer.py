import FreeCAD
import FreeCADGui


def _mod_path():
    from .. import ChronoUtils
    return ChronoUtils.MOD_PATH


def _icons_path():
    from .. import ChronoUtils
    return ChronoUtils.ICONS_PATH


class CreateContainerCommand:
    def GetResources(self):
        import os
        return {
            "Pixmap": os.path.join(_icons_path(), "ChronoContainer.svg"),
            "MenuText": "Create Physics Container",
            "ToolTip": (
                "Create a Chrono Physics container with a Chrono System object.\n"
                "All rigid bodies and settings are organized inside this container."
            ),
        }

    def IsActive(self):
        if FreeCAD.ActiveDocument is None:
            return False
        # Disable if a container already exists
        from ..objects.ChronoContainer import find_container
        return find_container() is None

    def Activated(self):
        from ..objects.ChronoContainer import make_container
        FreeCAD.ActiveDocument.openTransaction("Create Physics Container")
        make_container()
        FreeCAD.ActiveDocument.commitTransaction()


FreeCADGui.addCommand("Chrono_CreateContainer", CreateContainerCommand())
