import FreeCADGui


class ChronoWorkbench(FreeCADGui.Workbench):
    MenuText = "Chrono Physics"
    ToolTip = "Chrono Physics rigid body simulation"

    def __init__(self):
        import os
        from . import ChronoUtils
        self.__class__.Icon = os.path.join(ChronoUtils.ICONS_PATH, "Chrono.svg")
        FreeCADGui.addIconPath(ChronoUtils.ICONS_PATH)
        from .preferences.ChronoPreferences import ChronoPreferencesPage
        FreeCADGui.addPreferencePage(ChronoPreferencesPage, "Chrono Physics")

    def Initialize(self):
        from .commands import CmdCreateContainer
        from .commands import CmdAddRigidBody
        from .commands import CmdAddLauncher
        from .commands import CmdAddEmitter
        from .commands import CmdAddDestroyRigidBody
        from .commands import CmdAddObserver
        from .commands import CmdAddMesh
        from .commands import CmdDowngrade
        from .commands import CmdRunSimulation
        from .commands import CmdRefreshEmitterGroups

        tool_list = [
            "Chrono_CreateContainer",
            "Separator",
            "Chrono_AddActiveBody",
            "Chrono_AddPassiveBody",
            "Separator",
            "Chrono_AddMesh",
            "Separator",
            "Chrono_Downgrade",
            "Separator",
            "Chrono_AddLauncher",
            "Separator",
            "Chrono_AddEmitter",
            "Separator",
            "Chrono_AddDestroyRigidBody",
            "Separator",
            "Chrono_AddObserver",
            "Separator",
            "Chrono_RunSimulation",
            "Chrono_RefreshEmitterGroups",
        ]
        self.appendToolbar("Chrono Physics", tool_list)
        self.appendMenu("&Chrono Physics", tool_list)

    def Activated(self):
        pass

    def Deactivated(self):
        pass

    def GetClassName(self):
        return "Gui::PythonWorkbench"


FreeCADGui.addWorkbench(ChronoWorkbench())
