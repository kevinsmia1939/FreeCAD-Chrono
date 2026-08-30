import FreeCAD


class ChronoObserverFeature:
    """Non-physical volume that reports rigid bodies within its shape."""

    def __init__(self, obj):
        obj.addProperty("App::PropertyLink", "SourceObject", "Observer",
                        "Volume or surface used to observe rigid bodies")
        obj.addProperty("App::PropertyBool", "Enabled", "Observer",
                        "Include this observer in Chrono Physics simulations")
        obj.Enabled = True
        self._ensure_properties(obj)
        obj.addProperty("App::PropertyStringList", "CurrentBodies", "Observation",
                        "Bodies currently inside or touching the observer")
        obj.addProperty("App::PropertyString", "LastEvent", "Observation",
                        "Most recent observer event")
        obj.Proxy = self

    @staticmethod
    def _ensure_properties(obj):
        definitions = (
            ("App::PropertyEnumeration", "NameCondition", "Trigger Condition",
             "Optional rigid-body name comparison", ["Any", "Exact", "Regular expression"]),
            ("App::PropertyString", "NamePattern", "Trigger Condition",
             "Rigid-body Name or Label to match; regular expressions search any part of it", ""),
            ("App::PropertyEnumeration", "BodyTypeCondition", "Trigger Condition",
             "Rigid-body type required by this observer", ["Any", "Dynamic", "Fixed"]),
            ("App::PropertyEnumeration", "SpeedCondition", "Trigger Condition",
             "Optional speed comparison in m/s", ["Any", "At least", "At most"]),
            ("App::PropertyFloat", "Speed", "Trigger Condition",
             "Speed threshold in m/s", 0.0),
            ("App::PropertyEnumeration", "LifetimeCondition", "Trigger Condition",
             "Optional particle-age comparison; emitted bodies age from emission",
             ["Any", "Greater than", "Less than", "Between"]),
            ("App::PropertyFloat", "Lifetime", "Trigger Condition",
             "Particle lifetime threshold in seconds", 0.0),
            ("App::PropertyFloat", "LifetimeMaximum", "Trigger Condition",
             "Upper particle lifetime threshold in seconds for Between", 0.0),
            ("App::PropertyEnumeration", "ConditionLogic", "Trigger Condition",
             "How enabled name, type, speed, and lifetime conditions are combined", ["AND", "OR"]),
            ("App::PropertyFloat", "TriggerDelay", "Trigger Condition",
             "Seconds a matching body must remain observed before triggering", 0.0),
            ("App::PropertyBool", "Triggered", "Trigger State",
             "True after this observer has emitted its signal during the simulation", False),
            ("App::PropertyString", "TriggerBody", "Trigger State",
             "Body that caused the observer signal", ""),
            ("App::PropertyFloat", "TriggerTime", "Trigger State",
             "Simulation time when the observer signal was emitted", 0.0),
        )
        for property_type, name, group, description, value in definitions:
            if hasattr(obj, name):
                continue
            obj.addProperty(property_type, name, group, description)
            setattr(obj, name, value)
        selected = getattr(obj, "BodyTypeCondition", "Any")
        obj.BodyTypeCondition = ["Any", "Dynamic", "Fixed"]
        obj.BodyTypeCondition = (
            "Any" if selected == "Any"
            else "Fixed" if selected in ("Fixed", "Passive") else "Dynamic")

    def execute(self, obj):
        pass

    def onDocumentRestored(self, obj):
        if not hasattr(obj, "Enabled"):
            obj.addProperty("App::PropertyBool", "Enabled", "Observer",
                            "Include this observer in Chrono Physics simulations")
            obj.Enabled = True
        if not hasattr(obj, "CurrentBodies"):
            obj.addProperty("App::PropertyStringList", "CurrentBodies", "Observation",
                            "Bodies currently inside or touching the observer")
        if not hasattr(obj, "LastEvent"):
            obj.addProperty("App::PropertyString", "LastEvent", "Observation",
                            "Most recent observer event")
        self._ensure_properties(obj)
        if FreeCAD.GuiUp:
            obj.ViewObject.Visibility = True

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class ChronoObserverViewProvider:
    def __init__(self, vobj):
        vobj.Proxy = self

    def attach(self, vobj):
        self.Object = vobj.Object

    def setEdit(self, vobj, mode):
        import FreeCADGui
        FreeCADGui.Control.showDialog(ObserverPanel(vobj.Object))
        return True

    def unsetEdit(self, vobj, mode):
        import FreeCADGui
        FreeCADGui.Control.closeDialog()
        return True

    def doubleClicked(self, vobj):
        return self.setEdit(vobj, 0)

    def getIcon(self):
        import os
        from .. import ChronoUtils
        return os.path.join(ChronoUtils.ICONS_PATH, "AddObserver.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class ObserverPanel:
    """Small editor for the non-physical observer configuration."""

    def __init__(self, observer):
        from PySide import QtWidgets

        self.Object = observer
        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle("Chrono Physics Observer")
        layout = QtWidgets.QVBoxLayout(self.form)
        layout.addWidget(QtWidgets.QLabel(
            "The observer does not collide with rigid bodies. It reports bodies "
            "inside or touching the selected shape."))
        self.enabled = QtWidgets.QCheckBox("Enable observer")
        self.enabled.setChecked(observer.Enabled)
        layout.addWidget(self.enabled)
        condition_group = QtWidgets.QGroupBox("Trigger Condition")
        condition_form = QtWidgets.QFormLayout(condition_group)
        self.body_type = QtWidgets.QComboBox()
        self.body_type.addItems(["Any", "Dynamic", "Fixed"])
        self.body_type.setCurrentText(
            "Fixed" if observer.BodyTypeCondition in ("Fixed", "Passive")
            else "Dynamic" if observer.BodyTypeCondition == "Active"
            else observer.BodyTypeCondition)
        self.name_condition = QtWidgets.QComboBox()
        self.name_condition.addItems(["Any", "Exact", "Regular expression"])
        self.name_condition.setCurrentText(getattr(observer, "NameCondition", "Any"))
        self.name_pattern = QtWidgets.QLineEdit(
            getattr(observer, "NamePattern", ""))
        self.name_pattern.setPlaceholderText("e.g. RigidBody_Ball or ^particle_")
        self.name_pattern.setToolTip(
            "Exact compares the complete rigid-body Name or Label. Regular expression "
            "searches any part of the Name or Label (for example: ^particle_).")
        self.speed_condition = QtWidgets.QComboBox()
        self.speed_condition.addItems(["Any", "At least", "At most"])
        self.speed_condition.setCurrentText(observer.SpeedCondition)
        self.speed = QtWidgets.QDoubleSpinBox()
        self.speed.setRange(0.0, 100000.0)
        self.speed.setDecimals(6)
        self.speed.setSuffix(" m/s")
        self.speed.setValue(observer.Speed)
        self.lifetime_condition = QtWidgets.QComboBox()
        self.lifetime_condition.addItems(["Any", "Greater than", "Less than", "Between"])
        self.lifetime_condition.setCurrentText(
            getattr(observer, "LifetimeCondition", "Any"))
        self.lifetime = QtWidgets.QDoubleSpinBox()
        self.lifetime.setRange(0.0, 100000.0)
        self.lifetime.setDecimals(4)
        self.lifetime.setSuffix(" s")
        self.lifetime.setValue(max(0.0, getattr(observer, "Lifetime", 0.0)))
        self.lifetime_maximum = QtWidgets.QDoubleSpinBox()
        self.lifetime_maximum.setRange(0.0, 100000.0)
        self.lifetime_maximum.setDecimals(4)
        self.lifetime_maximum.setSuffix(" s")
        self.lifetime_maximum.setValue(max(0.0, getattr(observer, "LifetimeMaximum", 0.0)))
        self.logic = QtWidgets.QComboBox()
        self.logic.addItems(["AND", "OR"])
        self.logic.setCurrentText(observer.ConditionLogic)
        self.delay = QtWidgets.QDoubleSpinBox()
        self.delay.setRange(0.0, 100000.0)
        self.delay.setDecimals(4)
        self.delay.setSuffix(" s")
        self.delay.setValue(observer.TriggerDelay)
        condition_form.addRow("Rigid-body name:", self.name_condition)
        condition_form.addRow("Name / expression:", self.name_pattern)
        condition_form.addRow("Body type:", self.body_type)
        condition_form.addRow("Speed:", self.speed_condition)
        condition_form.addRow("Speed threshold:", self.speed)
        condition_form.addRow("Particle lifetime:", self.lifetime_condition)
        self.lifetime_label = QtWidgets.QLabel("Lifetime threshold:")
        condition_form.addRow(self.lifetime_label, self.lifetime)
        self.lifetime_maximum_label = QtWidgets.QLabel("Lifetime maximum:")
        condition_form.addRow(self.lifetime_maximum_label, self.lifetime_maximum)
        condition_form.addRow("Combine rules:", self.logic)
        condition_form.addRow("Trigger delay:", self.delay)
        layout.addWidget(condition_group)
        self.signal = QtWidgets.QLabel()
        layout.addWidget(self.signal)
        self.current = QtWidgets.QPlainTextEdit()
        self.current.setReadOnly(True)
        self.current.setPlaceholderText("No observed bodies yet.")
        layout.addWidget(self.current)
        self.speed_condition.currentIndexChanged.connect(self._update_condition_controls)
        self.lifetime_condition.currentIndexChanged.connect(self._update_condition_controls)
        self.name_condition.currentIndexChanged.connect(self._update_condition_controls)
        self._update_condition_controls()
        self._refresh()

    def _update_condition_controls(self):
        name_enabled = self.name_condition.currentText() != "Any"
        self.name_pattern.setEnabled(name_enabled)
        self.speed.setEnabled(self.speed_condition.currentText() != "Any")
        mode = self.lifetime_condition.currentText()
        self.lifetime.setEnabled(mode != "Any")
        self.lifetime_label.setVisible(mode != "Any")
        self.lifetime.setVisible(mode != "Any")
        between = mode == "Between"
        self.lifetime_maximum_label.setVisible(between)
        self.lifetime_maximum.setVisible(between)

    def _refresh(self):
        self.current.setPlainText("\n".join(self.Object.CurrentBodies))
        if self.Object.Triggered:
            self.signal.setText("Triggered by {} at {:.6f} s".format(
                self.Object.TriggerBody, self.Object.TriggerTime))
        else:
            self.signal.setText("Not triggered in the current simulation.")

    def accept(self):
        self.Object.Enabled = self.enabled.isChecked()
        self.Object.NameCondition = self.name_condition.currentText()
        self.Object.NamePattern = self.name_pattern.text().strip()
        self.Object.BodyTypeCondition = self.body_type.currentText()
        self.Object.SpeedCondition = self.speed_condition.currentText()
        self.Object.Speed = self.speed.value()
        self.Object.LifetimeCondition = self.lifetime_condition.currentText()
        self.Object.Lifetime = self.lifetime.value()
        self.Object.LifetimeMaximum = self.lifetime_maximum.value()
        self.Object.ConditionLogic = self.logic.currentText()
        self.Object.TriggerDelay = self.delay.value()
        self.Object.Document.recompute()
        return True

    def reject(self):
        return True


def make_observer(source_obj, container=None):
    """Create a non-physical observer for *source_obj*."""
    doc = FreeCAD.ActiveDocument
    # Part::FeaturePython keeps this no-shape task object active in the tree.
    obj = doc.addObject("Part::FeaturePython", f"Observer_{source_obj.Name}")
    ChronoObserverFeature(obj)
    obj.SourceObject = source_obj
    obj.Label = f"Observer: {source_obj.Label}"

    if FreeCAD.GuiUp:
        ChronoObserverViewProvider(obj.ViewObject)
        obj.ViewObject.Visibility = True

    if container is not None:
        if not hasattr(container, "Observers"):
            container.addProperty("App::PropertyLinkList", "Observers", "Container",
                                  "Non-physical rigid-body observer objects")
        container.Observers = list(getattr(container, "Observers", [])) + [obj]

    doc.recompute()
    return obj
