"""Refresh completed-simulation emitter result folders."""

import os
import FreeCAD
import FreeCADGui


class RefreshEmitterGroupsCommand:
    def GetResources(self):
        from .. import ChronoUtils
        return {
            "Pixmap": os.path.join(ChronoUtils.ICONS_PATH, "RunSimulation.svg"),
            "MenuText": "Refresh Emitted-Body Groups",
            "ToolTip": "Rebuild each emitter's Deactivated, Dynamic, and Fixed folders from the saved simulation cache.",
        }

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None

    def Activated(self):
        from ..simulation.ChronoSimulation import (
            load_simulation_cache, refresh_emitted_link_groups)
        cached = load_simulation_cache()
        if cached is None or not cached[0]:
            FreeCAD.Console.PrintWarning(
                "Chrono: no completed simulation cache is available to refresh emitted-body groups.\n")
            return
        count = refresh_emitted_link_groups(final_frame=cached[0][-1])
        FreeCAD.Console.PrintMessage(
            f"Chrono: refreshed emitted-body result folders ({count} links).\n")


FreeCADGui.addCommand("Chrono_RefreshEmitterGroups", RefreshEmitterGroupsCommand())
