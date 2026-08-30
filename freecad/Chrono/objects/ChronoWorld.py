import FreeCAD
from ..GuiUtils import collapsible_section


class ChronoWorldFeature:
    def __init__(self, obj):
        obj.addProperty("App::PropertyFloat", "Gravity", "Physics",
                        "Gravitational acceleration magnitude (m/s²)")
        obj.Gravity = 9.81

        obj.addProperty("App::PropertyVector", "GravityDirection", "Physics",
                        "Gravity direction unit vector (default: −Z = downward)")
        obj.GravityDirection = FreeCAD.Vector(0, 0, -1)

        obj.addProperty("App::PropertyFloat", "EndTime", "Simulation",
                        "Total simulation duration in seconds. "
                        "The number of recorded frames = EndTime / TimeStep.")
        obj.EndTime = 10.0

        obj.addProperty("App::PropertyFloat", "TimeStep", "Simulation",
                        "Duration of each recorded frame in seconds (e.g. 1/60 ≈ 0.01667). "
                        "The Chrono tick = TimeStep / SubSteps, so increasing SubSteps "
                        "improves accuracy without changing playback speed or total duration.")
        obj.TimeStep = 1.0 / 60.0

        obj.addProperty("App::PropertyInteger", "SolverIterations", "Simulation",
                        "Maximum iterations for Chrono's iterative constraint solver per integration step")
        obj.SolverIterations = 10

        obj.addProperty("App::PropertyEnumeration", "ContactModel", "Contact",
                        "Chrono contact formulation: NSC is non-smooth; SMC is penalty based")
        obj.ContactModel = ["NSC", "SMC"]

        obj.addProperty("App::PropertyBool", "EnableCPUParallel", "Simulation",
                        "Use Chrono's multicore collision system and all available core/Eigen threads")
        obj.EnableCPUParallel = True

        obj.addProperty("App::PropertyInteger", "SubSteps", "Simulation",
                        "Integration substeps per recorded frame. "
                        "Higher values prevent objects passing through each other "
                        "at the cost of simulation time (default 4)")
        obj.SubSteps = 4

        obj.addProperty("App::PropertyFloat", "CollisionMargin", "Simulation",
                        "Chrono collision envelope in metres applied to every collision shape. "
                        "Smaller values let objects appear to touch. "
                        "Chrono's built-in default is 0.04 m.")
        obj.CollisionMargin = 0.001

        obj.addProperty("App::PropertyBool", "StopWhenSettled", "Simulation",
                        "End the simulation after all dynamic bodies remain below the settle thresholds")
        obj.StopWhenSettled = False
        obj.addProperty("App::PropertyFloat", "SettleLinearVelocity", "Simulation",
                        "Maximum linear speed in m/s for a dynamic body to be considered settled")
        obj.SettleLinearVelocity = 0.01
        obj.addProperty("App::PropertyFloat", "SettleAngularVelocity", "Simulation",
                        "Maximum angular speed in rad/s for a dynamic body to be considered settled")
        obj.SettleAngularVelocity = 0.01
        obj.addProperty("App::PropertyFloat", "SettleDuration", "Simulation",
                        "Time in seconds that every dynamic body must remain settled before stopping")
        obj.SettleDuration = 0.5

        obj.addProperty("App::PropertyBool", "ConditionalStopEnabled", "Conditions",
                        "Stop after a dynamic body touches the selected target")
        obj.ConditionalStopEnabled = False
        obj.addProperty("App::PropertyLink", "ConditionalStopTarget", "Conditions",
                        "Non-physical target shape used for the conditional stop")
        obj.addProperty("App::PropertyEnumeration", "ConditionalStopType", "Conditions",
                        "Stop when a dynamic/fixed body contacts a shape or enters an observer")
        obj.ConditionalStopType = ["Collision Object", "Observer"]
        obj.addProperty("App::PropertyEnumeration", "ConditionalStopBodyType", "Conditions",
                        "Rigid-body type required to satisfy the conditional stop")
        obj.ConditionalStopBodyType = ["Dynamic", "Fixed", "Any"]
        obj.addProperty("App::PropertyFloat", "ConditionalStopDelay", "Conditions",
                        "Time in seconds to continue after the first target contact")
        obj.ConditionalStopDelay = 0.0

        obj.addProperty("App::PropertyBool", "ShowCollisionWireframes", "Display",
                        "Show collision-shape wireframes in the simulation panel")
        obj.ShowCollisionWireframes = False
        obj.addProperty("App::PropertyString", "PlaybackSpeed", "Playback",
                        "Selected playback speed in the simulation panel")
        obj.PlaybackSpeed = "1×"
        obj.addProperty("App::PropertyBool", "PlaybackLoop", "Playback",
                        "Loop playback in the simulation panel")
        obj.PlaybackLoop = True
        obj.addProperty("App::PropertyString", "SpeedGraphAxis", "Analysis",
                        "Horizontal axis selected for the particle speed graph")
        obj.SpeedGraphAxis = "Time"

        obj.addProperty("App::PropertyFloat", "LinearDamping", "Physics",
                        "Air damping applied to linear (translational) velocity of active "
                        "bodies each step. 0 = no damping, 1 = full stop. "
                        "Simulates air/fluid drag on translation.")
        obj.LinearDamping = 0.0

        obj.addProperty("App::PropertyFloat", "AngularDamping", "Physics",
                        "Air damping applied to angular (rotational) velocity of active "
                        "bodies each step. 0 = no damping, 1 = full stop. "
                        "Simulates air/fluid drag on rotation.")
        obj.AngularDamping = 0.0

        obj.addProperty("App::PropertyBool", "EnableVelocityLimit", "Physics",
                        "Clamp every dynamic body's Chrono linear and angular velocity")
        obj.EnableVelocityLimit = False
        obj.addProperty("App::PropertyFloat", "MaximumLinearVelocity", "Physics",
                        "Chrono linear velocity limit in m/s; active when velocity limiting is enabled")
        obj.MaximumLinearVelocity = 100.0
        obj.addProperty("App::PropertyFloat", "MaximumAngularVelocity", "Physics",
                        "Chrono angular velocity limit in rad/s; active when velocity limiting is enabled")
        obj.MaximumAngularVelocity = 100.0
        obj.addProperty("App::PropertyFloat", "MaximumPenetrationRecoverySpeed", "Physics",
                        "Maximum contact-separation speed in m/s after penetration")
        obj.MaximumPenetrationRecoverySpeed = 0.6
        obj.addProperty("App::PropertyBool", "EnableSleeping", "Physics",
                        "Allow Chrono to deactivate dynamic bodies that remain below sleep thresholds")
        obj.EnableSleeping = False
        obj.addProperty("App::PropertyFloat", "SleepTime", "Physics",
                        "Seconds below the sleep velocity thresholds before a body is deactivated")
        obj.SleepTime = 0.5
        obj.addProperty("App::PropertyFloat", "SleepLinearVelocity", "Physics",
                        "Maximum linear velocity in m/s for automatic Chrono sleeping")
        obj.SleepLinearVelocity = 0.01
        obj.addProperty("App::PropertyFloat", "SleepAngularVelocity", "Physics",
                        "Maximum angular velocity in rad/s for automatic Chrono sleeping")
        obj.SleepAngularVelocity = 0.01
        obj.addProperty("App::PropertyBool", "DisableGyroscopicTorque", "Physics",
                        "Disable gyroscopic torque for improved stability of fast-spinning non-uniform bodies")
        obj.DisableGyroscopicTorque = False

        obj.Proxy = self

    def _ensure_properties(self, obj):
        """Add any properties that are missing (handles objects created by older code)."""
        if not hasattr(obj, "EndTime"):
            obj.addProperty("App::PropertyFloat", "EndTime", "Simulation",
                            "Total simulation duration in seconds. "
                            "The number of recorded frames = EndTime / TimeStep.")
            # Migrate from old Steps property if present
            old_steps = getattr(obj, "Steps", 500)
            old_dt    = getattr(obj, "TimeStep", 1.0 / 60.0)
            obj.EndTime = old_steps * old_dt
        if not hasattr(obj, "SubSteps"):
            obj.addProperty("App::PropertyInteger", "SubSteps", "Simulation",
                            "Physics sub-steps per recorded frame. "
                            "Higher values prevent objects passing through each other "
                            "at the cost of simulation time (default 4)")
            obj.SubSteps = 4
        if not hasattr(obj, "EnableCPUParallel"):
            obj.addProperty("App::PropertyBool", "EnableCPUParallel", "Simulation",
                            "Use Chrono's multicore collision system and all available core/Eigen threads")
            obj.EnableCPUParallel = True
        if not hasattr(obj, "ContactModel"):
            obj.addProperty("App::PropertyEnumeration", "ContactModel", "Contact",
                            "Chrono contact formulation: NSC is non-smooth; SMC is penalty based")
            obj.ContactModel = ["NSC", "SMC"]
        else:
            selected_model = getattr(obj, "ContactModel", "NSC")
            obj.ContactModel = ["NSC", "SMC"]
            obj.ContactModel = selected_model if selected_model in ("NSC", "SMC") else "NSC"
        if not hasattr(obj, "LinearDamping"):
            obj.addProperty("App::PropertyFloat", "LinearDamping", "Physics",
                            "Air damping applied to linear (translational) velocity of active "
                            "bodies each step. 0 = no damping, 1 = full stop.")
            obj.LinearDamping = 0.0
        if not hasattr(obj, "AngularDamping"):
            obj.addProperty("App::PropertyFloat", "AngularDamping", "Physics",
                            "Air damping applied to angular (rotational) velocity of active "
                            "bodies each step. 0 = no damping, 1 = full stop.")
            obj.AngularDamping = 0.0
        if not hasattr(obj, "EnableVelocityLimit"):
            obj.addProperty("App::PropertyBool", "EnableVelocityLimit", "Physics",
                            "Clamp every dynamic body's Chrono linear and angular velocity")
            obj.EnableVelocityLimit = False
        if not hasattr(obj, "MaximumLinearVelocity"):
            obj.addProperty("App::PropertyFloat", "MaximumLinearVelocity", "Physics",
                            "Chrono linear velocity limit in m/s; active when velocity limiting is enabled")
            obj.MaximumLinearVelocity = 100.0
        if not hasattr(obj, "MaximumAngularVelocity"):
            obj.addProperty("App::PropertyFloat", "MaximumAngularVelocity", "Physics",
                            "Chrono angular velocity limit in rad/s; active when velocity limiting is enabled")
            obj.MaximumAngularVelocity = 100.0
        for name, description, default in (
                ("MaximumPenetrationRecoverySpeed",
                 "Maximum contact-separation speed in m/s after penetration", 0.6),
                ("EnableSleeping",
                 "Allow Chrono to deactivate dynamic bodies below sleep thresholds", False),
                ("SleepTime", "Seconds below sleep velocity thresholds before deactivation", 0.5),
                ("SleepLinearVelocity", "Maximum linear velocity in m/s for automatic Chrono sleeping", 0.01),
                ("SleepAngularVelocity", "Maximum angular velocity in rad/s for automatic Chrono sleeping", 0.01),
                ("DisableGyroscopicTorque",
                 "Disable gyroscopic torque for improved stability of fast-spinning bodies", False)):
            if not hasattr(obj, name):
                property_type = "App::PropertyBool" if isinstance(default, bool) else "App::PropertyFloat"
                obj.addProperty(property_type, name, "Physics", description)
                setattr(obj, name, default)
        if not hasattr(obj, "CollisionMargin"):
            obj.addProperty("App::PropertyFloat", "CollisionMargin", "Simulation",
                            "Chrono collision envelope in metres applied to every collision shape. "
                            "Smaller values let objects appear to touch. "
                            "Chrono's built-in default is 0.04 m.")
            obj.CollisionMargin = 0.001
        if not hasattr(obj, "StopWhenSettled"):
            obj.addProperty("App::PropertyBool", "StopWhenSettled", "Simulation",
                            "End the simulation after all active bodies remain below the settle thresholds")
            obj.StopWhenSettled = False
        if not hasattr(obj, "SettleLinearVelocity"):
            obj.addProperty("App::PropertyFloat", "SettleLinearVelocity", "Simulation",
                            "Maximum linear speed in m/s for an active body to be considered settled")
            obj.SettleLinearVelocity = 0.01
        if not hasattr(obj, "SettleAngularVelocity"):
            obj.addProperty("App::PropertyFloat", "SettleAngularVelocity", "Simulation",
                            "Maximum angular speed in rad/s for an active body to be considered settled")
            obj.SettleAngularVelocity = 0.01
        if not hasattr(obj, "SettleDuration"):
            obj.addProperty("App::PropertyFloat", "SettleDuration", "Simulation",
                            "Time in seconds that every active body must remain settled before stopping")
            obj.SettleDuration = 0.5
        if not hasattr(obj, "ConditionalStopEnabled"):
            obj.addProperty("App::PropertyBool", "ConditionalStopEnabled", "Conditions",
                            "Stop after an active body touches the selected target")
            obj.ConditionalStopEnabled = False
        if not hasattr(obj, "ConditionalStopTarget"):
            obj.addProperty("App::PropertyLink", "ConditionalStopTarget", "Conditions",
                            "Non-physical target shape used for the conditional stop")
        if not hasattr(obj, "ConditionalStopType"):
            obj.addProperty("App::PropertyEnumeration", "ConditionalStopType", "Conditions",
                            "Stop when an active/passive body contacts a shape or enters an observer")
            obj.ConditionalStopType = ["Collision Object", "Observer"]
        if not hasattr(obj, "ConditionalStopBodyType"):
            obj.addProperty("App::PropertyEnumeration", "ConditionalStopBodyType", "Conditions",
                            "Rigid-body type required to satisfy the conditional stop")
            obj.ConditionalStopBodyType = ["Dynamic", "Fixed", "Any"]
        if not hasattr(obj, "ConditionalStopDelay"):
            obj.addProperty("App::PropertyFloat", "ConditionalStopDelay", "Conditions",
                            "Time in seconds to continue after the first target contact")
            obj.ConditionalStopDelay = 0.0
        if not hasattr(obj, "ShowCollisionWireframes"):
            obj.addProperty("App::PropertyBool", "ShowCollisionWireframes", "Display",
                            "Show collision-shape wireframes in the simulation panel")
            obj.ShowCollisionWireframes = False
        if not hasattr(obj, "PlaybackSpeed"):
            obj.addProperty("App::PropertyString", "PlaybackSpeed", "Playback",
                            "Selected playback speed in the simulation panel")
            obj.PlaybackSpeed = "1×"
        if not hasattr(obj, "PlaybackLoop"):
            obj.addProperty("App::PropertyBool", "PlaybackLoop", "Playback",
                            "Loop playback in the simulation panel")
            obj.PlaybackLoop = True
        if not hasattr(obj, "SpeedGraphAxis"):
            obj.addProperty("App::PropertyString", "SpeedGraphAxis", "Analysis",
                            "Horizontal axis selected for the particle speed graph")
            obj.SpeedGraphAxis = "Time"
        selected_type = getattr(obj, "ConditionalStopBodyType", "Dynamic")
        obj.ConditionalStopBodyType = ["Dynamic", "Fixed", "Any"]
        obj.ConditionalStopBodyType = (
            "Any" if selected_type == "Any"
            else "Fixed" if selected_type in ("Fixed", "Passive") else "Dynamic")

    def execute(self, obj):
        pass

    def onDocumentRestored(self, obj):
        self._ensure_properties(obj)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class WorldSettingsPanel:
    """Task panel exposing all ChronoWorld properties as editable fields."""

    def __init__(self, world_obj):
        from PySide import QtWidgets

        self._obj = world_obj
        if hasattr(world_obj.Proxy, "_ensure_properties"):
            world_obj.Proxy._ensure_properties(world_obj)

        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle("Chrono System Settings")
        self.form.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding)

        root = QtWidgets.QVBoxLayout(self.form)

        # ── Physics ───────────────────────────────────────────────────────────
        phys_group, phys_form = collapsible_section(
            "Physics", QtWidgets.QFormLayout)

        self.inp_gravity = QtWidgets.QDoubleSpinBox()
        self.inp_gravity.setRange(0.0, 1000.0)
        self.inp_gravity.setDecimals(4)
        self.inp_gravity.setSuffix(" m/s²")
        self.inp_gravity.setToolTip("Gravitational acceleration magnitude")
        self.inp_gravity.setValue(getattr(world_obj, "Gravity", 9.81))
        phys_form.addRow("Gravity:", self.inp_gravity)

        gd = getattr(world_obj, "GravityDirection", FreeCAD.Vector(0, 0, -1))
        gdir_widget = QtWidgets.QWidget()
        gdir_layout = QtWidgets.QHBoxLayout(gdir_widget)
        gdir_layout.setContentsMargins(0, 0, 0, 0)
        self.inp_gx = QtWidgets.QDoubleSpinBox()
        self.inp_gy = QtWidgets.QDoubleSpinBox()
        self.inp_gz = QtWidgets.QDoubleSpinBox()
        for sp, label, val in ((self.inp_gx, "X", gd.x),
                               (self.inp_gy, "Y", gd.y),
                               (self.inp_gz, "Z", gd.z)):
            sp.setRange(-1.0, 1.0)
            sp.setDecimals(4)
            sp.setSingleStep(0.1)
            sp.setValue(val)
            gdir_layout.addWidget(QtWidgets.QLabel(label))
            gdir_layout.addWidget(sp)
        gdir_widget.setToolTip("Gravity direction unit vector (−Z = downward)")
        phys_form.addRow("Gravity Direction:", gdir_widget)

        self.inp_lin_damp = QtWidgets.QDoubleSpinBox()
        self.inp_lin_damp.setRange(0.0, 1.0)
        self.inp_lin_damp.setDecimals(4)
        self.inp_lin_damp.setSingleStep(0.01)
        self.inp_lin_damp.setToolTip("Linear (translational) air damping  0 = none, 1 = full stop")
        self.inp_lin_damp.setValue(getattr(world_obj, "LinearDamping", 0.0))
        phys_form.addRow("Linear Damping:", self.inp_lin_damp)

        self.inp_ang_damp = QtWidgets.QDoubleSpinBox()
        self.inp_ang_damp.setRange(0.0, 1.0)
        self.inp_ang_damp.setDecimals(4)
        self.inp_ang_damp.setSingleStep(0.01)
        self.inp_ang_damp.setToolTip("Angular (rotational) air damping  0 = none, 1 = full stop")
        self.inp_ang_damp.setValue(getattr(world_obj, "AngularDamping", 0.0))
        phys_form.addRow("Angular Damping:", self.inp_ang_damp)

        self.inp_velocity_limit = QtWidgets.QCheckBox("Enable Chrono velocity limits")
        self.inp_velocity_limit.setToolTip(
            "Clamp dynamic-body linear and angular velocities after Chrono solver updates. "
            "A safety limit, not a substitute for a stable time step.")
        self.inp_velocity_limit.setChecked(
            getattr(world_obj, "EnableVelocityLimit", False))
        phys_form.addRow(self.inp_velocity_limit)

        self.inp_max_linear_velocity = QtWidgets.QDoubleSpinBox()
        self.inp_max_linear_velocity.setRange(0.001, 1.0e8)
        self.inp_max_linear_velocity.setDecimals(4)
        self.inp_max_linear_velocity.setSuffix(" m/s")
        self.inp_max_linear_velocity.setToolTip(
            "Maximum linear body velocity while Chrono velocity limits are enabled")
        self.inp_max_linear_velocity.setValue(max(
            0.001, getattr(world_obj, "MaximumLinearVelocity", 100.0)))
        phys_form.addRow("Maximum linear velocity:", self.inp_max_linear_velocity)

        self.inp_max_angular_velocity = QtWidgets.QDoubleSpinBox()
        self.inp_max_angular_velocity.setRange(0.001, 1.0e8)
        self.inp_max_angular_velocity.setDecimals(4)
        self.inp_max_angular_velocity.setSuffix(" rad/s")
        self.inp_max_angular_velocity.setToolTip(
            "Maximum angular body velocity while Chrono velocity limits are enabled")
        self.inp_max_angular_velocity.setValue(max(
            0.001, getattr(world_obj, "MaximumAngularVelocity", 100.0)))
        phys_form.addRow("Maximum angular velocity:", self.inp_max_angular_velocity)

        self.inp_penetration_recovery = QtWidgets.QDoubleSpinBox()
        self.inp_penetration_recovery.setRange(0.0, 100000.0)
        self.inp_penetration_recovery.setDecimals(5)
        self.inp_penetration_recovery.setSuffix(" m/s")
        self.inp_penetration_recovery.setToolTip(
            "Limit the speed at which Chrono resolves penetration. Lower values reduce explosive separation.")
        self.inp_penetration_recovery.setValue(max(0.0, getattr(
            world_obj, "MaximumPenetrationRecoverySpeed", 0.6)))
        phys_form.addRow("Maximum penetration recovery speed:", self.inp_penetration_recovery)

        self.inp_sleeping = QtWidgets.QCheckBox("Enable Chrono sleeping")
        self.inp_sleeping.setToolTip(
            "Deactivate dynamic bodies after they remain below the selected velocity thresholds.")
        self.inp_sleeping.setChecked(getattr(world_obj, "EnableSleeping", False))
        phys_form.addRow(self.inp_sleeping)
        self.inp_sleep_time = QtWidgets.QDoubleSpinBox()
        self.inp_sleep_time.setRange(0.0, 3600.0)
        self.inp_sleep_time.setDecimals(4)
        self.inp_sleep_time.setSuffix(" s")
        self.inp_sleep_time.setValue(max(0.0, getattr(world_obj, "SleepTime", 0.5)))
        phys_form.addRow("Sleep time:", self.inp_sleep_time)
        self.inp_sleep_linear = QtWidgets.QDoubleSpinBox()
        self.inp_sleep_linear.setRange(0.0, 100000.0)
        self.inp_sleep_linear.setDecimals(5)
        self.inp_sleep_linear.setSuffix(" m/s")
        self.inp_sleep_linear.setValue(max(0.0, getattr(world_obj, "SleepLinearVelocity", 0.01)))
        phys_form.addRow("Sleep linear velocity:", self.inp_sleep_linear)
        self.inp_sleep_angular = QtWidgets.QDoubleSpinBox()
        self.inp_sleep_angular.setRange(0.0, 100000.0)
        self.inp_sleep_angular.setDecimals(5)
        self.inp_sleep_angular.setSuffix(" rad/s")
        self.inp_sleep_angular.setValue(max(0.0, getattr(world_obj, "SleepAngularVelocity", 0.01)))
        phys_form.addRow("Sleep angular velocity:", self.inp_sleep_angular)
        self.inp_disable_gyro = QtWidgets.QCheckBox("Disable gyroscopic torque")
        self.inp_disable_gyro.setToolTip(
            "Improves stability for fast-spinning thin or non-uniform bodies, at the cost of physical accuracy.")
        self.inp_disable_gyro.setChecked(getattr(world_obj, "DisableGyroscopicTorque", False))
        phys_form.addRow(self.inp_disable_gyro)

        root.addWidget(phys_group)

        # ── Simulation ────────────────────────────────────────────────────────
        sim_group, sim_form = collapsible_section(
            "Simulation", QtWidgets.QFormLayout)

        self.inp_end_time = QtWidgets.QDoubleSpinBox()
        self.inp_end_time.setRange(0.001, 3600.0)
        self.inp_end_time.setDecimals(3)
        self.inp_end_time.setSuffix(" s")
        self.inp_end_time.setToolTip("Total simulation duration in seconds")
        self.inp_end_time.setValue(getattr(world_obj, "EndTime", 10.0))
        sim_form.addRow("End Time:", self.inp_end_time)

        self.inp_timestep = QtWidgets.QDoubleSpinBox()
        self.inp_timestep.setRange(0.0001, 1.0)
        self.inp_timestep.setDecimals(6)
        self.inp_timestep.setSingleStep(0.001)
        self.inp_timestep.setSuffix(" s")
        self.inp_timestep.setToolTip(
            "Duration of each recorded frame in seconds (e.g. 1/60 ≈ 0.016667).\n"
            "Frames = EndTime / TimeStep.")
        self.inp_timestep.setValue(getattr(world_obj, "TimeStep", 1.0 / 60.0))
        sim_form.addRow("Time Step:", self.inp_timestep)

        self.inp_solver = QtWidgets.QSpinBox()
        self.inp_solver.setRange(1, 1000)
        self.inp_solver.setToolTip("Maximum iterations for Chrono's iterative constraint solver per integration step")
        self.inp_solver.setValue(getattr(world_obj, "SolverIterations", 10))
        sim_form.addRow("Maximum Solver Iterations:", self.inp_solver)

        self.inp_contact_model = QtWidgets.QComboBox()
        self.inp_contact_model.addItems(["NSC", "SMC"])
        self.inp_contact_model.setCurrentText(getattr(world_obj, "ContactModel", "NSC"))
        self.inp_contact_model.setToolTip(
            "NSC: non-smooth complementarity contacts, appropriate for rigid bodies.\n"
            "SMC: smooth penalty contacts, requiring stiffness/damping material data.")
        sim_form.addRow("Contact Model:", self.inp_contact_model)

        self.inp_substeps = QtWidgets.QSpinBox()
        self.inp_substeps.setRange(1, 1000)
        self.inp_substeps.setToolTip(
            "Integration substeps per recorded frame.\n"
            "Higher values improve collision accuracy at the cost of simulation time.")
        self.inp_substeps.setValue(getattr(world_obj, "SubSteps", 4))
        sim_form.addRow("Integration Substeps:", self.inp_substeps)

        self.inp_margin = QtWidgets.QDoubleSpinBox()
        self.inp_margin.setRange(0.0, 1.0)
        self.inp_margin.setDecimals(4)
        self.inp_margin.setSingleStep(0.001)
        self.inp_margin.setSuffix(" m")
        self.inp_margin.setToolTip(
            "Chrono collision envelope applied to every shape.\n"
            "Smaller values let objects appear to touch.\n"
            "0.001 m is a good default for typical FreeCAD models.\n"
            "Chrono's built-in default is 0.04 m.")
        self.inp_margin.setValue(getattr(world_obj, "CollisionMargin", 0.001))
        sim_form.addRow("Collision Envelope:", self.inp_margin)

        self.inp_stop_when_settled = QtWidgets.QCheckBox(
            "Finish simulation when dynamic bodies settle")
        self.inp_stop_when_settled.setToolTip(
            "Stop after every dynamic body stays below the selected velocity thresholds.\n"
            "Pending launcher and emitter events keep the simulation running.")
        self.inp_stop_when_settled.setChecked(
            getattr(world_obj, "StopWhenSettled", False))
        sim_form.addRow(self.inp_stop_when_settled)

        self.inp_settle_linear = QtWidgets.QDoubleSpinBox()
        self.inp_settle_linear.setRange(0.0, 1000.0)
        self.inp_settle_linear.setDecimals(5)
        self.inp_settle_linear.setSuffix(" m/s")
        self.inp_settle_linear.setValue(
            max(0.0, getattr(world_obj, "SettleLinearVelocity", 0.01)))
        sim_form.addRow("Settle linear speed:", self.inp_settle_linear)

        self.inp_settle_angular = QtWidgets.QDoubleSpinBox()
        self.inp_settle_angular.setRange(0.0, 1000.0)
        self.inp_settle_angular.setDecimals(5)
        self.inp_settle_angular.setSuffix(" rad/s")
        self.inp_settle_angular.setValue(
            max(0.0, getattr(world_obj, "SettleAngularVelocity", 0.01)))
        sim_form.addRow("Settle angular speed:", self.inp_settle_angular)

        self.inp_settle_duration = QtWidgets.QDoubleSpinBox()
        self.inp_settle_duration.setRange(0.0, 3600.0)
        self.inp_settle_duration.setDecimals(3)
        self.inp_settle_duration.setSuffix(" s")
        self.inp_settle_duration.setValue(
            max(0.0, getattr(world_obj, "SettleDuration", 0.5)))
        sim_form.addRow("Settle duration:", self.inp_settle_duration)

        root.addWidget(sim_group)
        root.addStretch(1)

        # ── Live-apply wiring ─────────────────────────────────────────────────
        self.inp_gravity.valueChanged.connect(self._apply)
        self.inp_gx.valueChanged.connect(self._apply)
        self.inp_gy.valueChanged.connect(self._apply)
        self.inp_gz.valueChanged.connect(self._apply)
        self.inp_lin_damp.valueChanged.connect(self._apply)
        self.inp_ang_damp.valueChanged.connect(self._apply)
        self.inp_velocity_limit.toggled.connect(self._update_velocity_limit_controls)
        self.inp_velocity_limit.toggled.connect(self._apply)
        self.inp_max_linear_velocity.valueChanged.connect(self._apply)
        self.inp_max_angular_velocity.valueChanged.connect(self._apply)
        self.inp_penetration_recovery.valueChanged.connect(self._apply)
        self.inp_sleeping.toggled.connect(self._update_sleep_controls)
        self.inp_sleeping.toggled.connect(self._apply)
        self.inp_sleep_time.valueChanged.connect(self._apply)
        self.inp_sleep_linear.valueChanged.connect(self._apply)
        self.inp_sleep_angular.valueChanged.connect(self._apply)
        self.inp_disable_gyro.toggled.connect(self._apply)
        self.inp_end_time.valueChanged.connect(self._apply)
        self.inp_timestep.valueChanged.connect(self._apply)
        self.inp_solver.valueChanged.connect(self._apply)
        self.inp_contact_model.currentTextChanged.connect(self._apply)
        self.inp_substeps.valueChanged.connect(self._apply)
        self.inp_margin.valueChanged.connect(self._apply)
        self.inp_stop_when_settled.toggled.connect(self._update_settle_controls)
        self.inp_stop_when_settled.toggled.connect(self._apply)
        self.inp_settle_linear.valueChanged.connect(self._apply)
        self.inp_settle_angular.valueChanged.connect(self._apply)
        self.inp_settle_duration.valueChanged.connect(self._apply)
        self._update_settle_controls()
        self._update_velocity_limit_controls()
        self._update_sleep_controls()

    def _update_velocity_limit_controls(self):
        enabled = self.inp_velocity_limit.isChecked()
        self.inp_max_linear_velocity.setEnabled(enabled)
        self.inp_max_angular_velocity.setEnabled(enabled)

    def _update_sleep_controls(self):
        enabled = self.inp_sleeping.isChecked()
        self.inp_sleep_time.setEnabled(enabled)
        self.inp_sleep_linear.setEnabled(enabled)
        self.inp_sleep_angular.setEnabled(enabled)

    def _update_settle_controls(self):
        enabled = self.inp_stop_when_settled.isChecked()
        self.inp_settle_linear.setEnabled(enabled)
        self.inp_settle_angular.setEnabled(enabled)
        self.inp_settle_duration.setEnabled(enabled)

    def _apply(self):
        obj = self._obj
        obj.Gravity          = self.inp_gravity.value()
        obj.GravityDirection = FreeCAD.Vector(
            self.inp_gx.value(), self.inp_gy.value(), self.inp_gz.value())
        obj.LinearDamping    = self.inp_lin_damp.value()
        obj.AngularDamping   = self.inp_ang_damp.value()
        obj.EnableVelocityLimit = self.inp_velocity_limit.isChecked()
        obj.MaximumLinearVelocity = self.inp_max_linear_velocity.value()
        obj.MaximumAngularVelocity = self.inp_max_angular_velocity.value()
        obj.MaximumPenetrationRecoverySpeed = self.inp_penetration_recovery.value()
        obj.EnableSleeping = self.inp_sleeping.isChecked()
        obj.SleepTime = self.inp_sleep_time.value()
        obj.SleepLinearVelocity = self.inp_sleep_linear.value()
        obj.SleepAngularVelocity = self.inp_sleep_angular.value()
        obj.DisableGyroscopicTorque = self.inp_disable_gyro.isChecked()
        obj.EndTime          = self.inp_end_time.value()
        obj.TimeStep         = self.inp_timestep.value()
        obj.SolverIterations = self.inp_solver.value()
        obj.ContactModel      = self.inp_contact_model.currentText()
        obj.SubSteps         = self.inp_substeps.value()
        obj.CollisionMargin  = self.inp_margin.value()
        obj.StopWhenSettled = self.inp_stop_when_settled.isChecked()
        obj.SettleLinearVelocity = self.inp_settle_linear.value()
        obj.SettleAngularVelocity = self.inp_settle_angular.value()
        obj.SettleDuration = self.inp_settle_duration.value()
        obj.Document.recompute()

    def reject(self):
        import FreeCADGui
        FreeCADGui.Control.closeDialog()


class ChronoWorldViewProvider:
    def __init__(self, vobj):
        vobj.Proxy = self

    def attach(self, vobj):
        self.Object = vobj.Object
        from pivy import coin
        vobj.addDisplayMode(coin.SoGroup(), "Default")

    def getDisplayModes(self, vobj):
        return ["Default"]

    def getDefaultDisplayMode(self):
        return "Default"

    def getIcon(self):
        import os
        from .. import ChronoUtils
        return os.path.join(ChronoUtils.ICONS_PATH, "ChronoWorld.svg")

    def setEdit(self, vobj, mode):
        import FreeCADGui
        if FreeCADGui.Control.activeDialog():
            FreeCADGui.Control.closeDialog()
        FreeCADGui.Control.showDialog(WorldSettingsPanel(vobj.Object))
        return True

    def unsetEdit(self, vobj, mode):
        import FreeCADGui
        FreeCADGui.Control.closeDialog()
        return True

    def doubleClicked(self, vobj):
        self.setEdit(vobj, 0)
        return True

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def find_world(doc=None):
    """Return the first ChronoWorld in the document, or None."""
    if doc is None:
        doc = FreeCAD.ActiveDocument
    if doc is None:
        return None
    for obj in doc.Objects:
        if (hasattr(obj, "Proxy")
                and type(obj.Proxy).__name__ == "ChronoWorldFeature"):
            return obj
    return None
