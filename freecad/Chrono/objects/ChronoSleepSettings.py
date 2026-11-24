import os

import FreeCAD

from .ChronoWorld import ChronoWorldViewProvider


SETTINGS_LABEL = "Sleep and Settle Settings"


class SleepSettingsPanel:
    def __init__(self, container):
        from PySide import QtWidgets

        world = container.World
        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle(SETTINGS_LABEL)
        layout = QtWidgets.QVBoxLayout(self.form)
        sleep = QtWidgets.QGroupBox("Sleep")
        sleep.setCheckable(True)
        sleep.setChecked(world.EnableSleeping)
        sleep.setToolTip("Allow settled dynamic bodies to sleep. Contact can wake them.")
        layout.addWidget(sleep)
        sleep_form = QtWidgets.QFormLayout(sleep)

        def number(owner, property_name, unit, maximum, decimals):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(decimals)
            spin.setRange(0.0, maximum)
            spin.setSuffix(" " + unit)
            spin.setValue(getattr(owner, property_name))
            spin.valueChanged.connect(lambda value: setattr(owner, property_name, value))
            return spin

        sleep_form.addRow("Time below both thresholds",
                          number(world, "SleepTime", "s", 3600, 4))
        sleep_form.addRow("Linear speed threshold",
                          number(world, "SleepLinearVelocity", "m/s", 100000, 5))
        sleep_form.addRow("Angular speed threshold",
                          number(world, "SleepAngularVelocity", "rad/s", 100000, 5))
        sleep.toggled.connect(lambda enabled: setattr(world, "EnableSleeping", enabled))

        settle = QtWidgets.QGroupBox("Stop when settled")
        settle.setCheckable(True)
        settle.setChecked(world.StopWhenSettled)
        settle.setToolTip("Stop after all dynamic bodies stay below both speed thresholds. "
                          "Pending launches and emissions keep the simulation running.")
        layout.addWidget(settle)
        settle_form = QtWidgets.QFormLayout(settle)
        settle_form.addRow("Settle duration", number(world, "SettleDuration", "s", 3600, 3))
        settle_form.addRow("Linear speed threshold",
                           number(world, "SettleLinearVelocity", "m/s", 1000, 5))
        settle_form.addRow("Angular speed threshold",
                           number(world, "SettleAngularVelocity", "rad/s", 1000, 5))
        settle.toggled.connect(lambda enabled: setattr(world, "StopWhenSettled", enabled))

        passive = QtWidgets.QGroupBox("Become passive after emission")
        settings = container.SleepSettings
        passive.setCheckable(True)
        passive.setChecked(settings.EnablePassivation)
        passive.toggled.connect(lambda enabled: setattr(settings, "EnablePassivation", enabled))
        layout.addWidget(passive)
        passive_form = QtWidgets.QFormLayout(passive)
        description = QtWidgets.QLabel(
            "Each particle becomes fixed at the selected age and still collides. "
            "Set the delay to zero to keep particles dynamic.")
        description.setWordWrap(True)
        passive_form.addRow(description)
        emitters = [emitter for emitter in container.Emitters if emitter is not None]
        if not emitters:
            passive_form.addRow(QtWidgets.QLabel("Add an emitter to set its delay."))
        for emitter in emitters:
            delay = number(emitter, "BecomePassiveAfter", "s", 100000, 4)
            delay.setEnabled(emitter.BodyType in ("Dynamic", "Active"))
            delay.setToolTip("Delay for each dynamic particle, measured from its emission.")
            passive_form.addRow(emitter.Label, delay)
        layout.addStretch()

    def getStandardButtons(self):
        from PySide import QtWidgets
        button = QtWidgets.QDialogButtonBox.Close
        return getattr(button, "value", button)

    def reject(self):
        import FreeCADGui
        FreeCADGui.Control.closeDialog()
        return True


class SleepSettingsViewProvider(ChronoWorldViewProvider):
    def getIcon(self):
        from .. import ChronoUtils
        return os.path.join(ChronoUtils.ICONS_PATH, "SleepAndSettle.svg")

    def setEdit(self, vobj, mode):
        import FreeCADGui
        container = next(obj for obj in vobj.Object.InList
                         if getattr(obj, "SleepSettings", None) == vobj.Object)
        FreeCADGui.Control.showDialog(SleepSettingsPanel(container))
        return True


def ensure_sleep_settings(container):
    if "SleepSettings" not in container.PropertiesList:
        container.addProperty("App::PropertyLink", "SleepSettings", "Container",
                              "Sleep, settle, and emitted-body passivation settings")
    if container.SleepSettings is None:
        settings = container.Document.addObject("App::FeaturePython", "ChronoSleepSettings")
        settings.Label = SETTINGS_LABEL
        if FreeCAD.GuiUp:
            SleepSettingsViewProvider(settings.ViewObject)
        container.SleepSettings = settings
    settings = container.SleepSettings
    if settings.Label == "Sleep and Become Passive":
        settings.Label = SETTINGS_LABEL
    if "EnablePassivation" not in settings.PropertiesList:
        settings.addProperty("App::PropertyBool", "EnablePassivation", "Emission",
                             "Allow dynamic particles to become fixed after their emitter delay")
        # Preserve the behavior of documents that already used a positive delay.
        settings.EnablePassivation = any(
            getattr(emitter, "BecomePassiveAfter", 0.0) > 0.0 for emitter in getattr(container, "Emitters", []))
    return settings
