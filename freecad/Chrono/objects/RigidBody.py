import FreeCAD

BODY_TYPE_DYNAMIC = "Dynamic"
BODY_TYPE_FIXED = "Fixed"


def canonical_body_type(body_type):
    """Map legacy Bullet-style names to Chrono's ChBody terminology."""
    return BODY_TYPE_FIXED if body_type in ("Fixed", "Passive") else BODY_TYPE_DYNAMIC


class RigidBodyFeature:
    """
    Stores Chrono Physics properties for one solid.
    The simulation drives BodyLink (App::Link), leaving OriginalObject untouched.
    """

    def __init__(self, obj):
        obj.addProperty("App::PropertyLink", "OriginalObject", "RigidBody",
                        "The original FreeCAD solid (never modified)")
        obj.addProperty("App::PropertyLink", "BodyLink", "RigidBody",
                        "App::Link clone — simulation moves this")
        obj.addProperty("App::PropertyEnumeration", "BodyType", "RigidBody",
                        "Dynamic: integrated by Chrono. Fixed: ChBody::SetFixed(true).")
        obj.BodyType = [BODY_TYPE_DYNAMIC, BODY_TYPE_FIXED]
        obj.addProperty("App::PropertyBool", "Enabled", "RigidBody",
                        "Include this body in Chrono Physics simulations")
        obj.Enabled = True
        obj.addProperty("App::PropertyFloat", "Density", "RigidBody",
                        "Material density in kg/m³. Mass is computed automatically "
                        "as density × shape volume. Ignored for Fixed bodies.")
        obj.Density = 1000.0
        obj.addProperty("App::PropertyFloat", "Restitution", "RigidBody",
                        "Bounciness 0 (none) → 1 (perfect)")
        obj.Restitution = 0.3
        for name, description, value in (
                ("StaticFriction", "Static friction coefficient", 0.5),
                ("SlidingFriction", "Sliding friction coefficient", 0.5),
                ("RollingFriction", "Rolling resistance coefficient (m in NSC)", 0.0),
                ("SpinningFriction", "Spinning resistance coefficient (m in NSC)", 0.0),
                ("Cohesion", "Maximum tensile contact force in NSC (N)", 0.0),
                ("ContactDamping", "NSC contact damping factor (s)", 0.0),
                ("NormalCompliance", "NSC normal compliance (m/N)", 0.0),
                ("TangentialCompliance", "NSC tangential compliance (m/N)", 0.0),
                ("YoungModulus", "SMC Young's modulus (Pa)", 2.0e7),
                ("PoissonRatio", "SMC Poisson ratio", 0.3),
                ("Adhesion", "SMC constant adhesion force (N)", 0.0),
                ("NormalStiffness", "SMC optional normal stiffness (N/m; 0 uses model)", 0.0),
                ("TangentialStiffness", "SMC optional tangential stiffness (N/m; 0 uses model)", 0.0),
                ("NormalDamping", "SMC optional normal damping (N s/m; 0 uses model)", 0.0),
                ("TangentialDamping", "SMC optional tangential damping (N s/m; 0 uses model)", 0.0)):
            obj.addProperty("App::PropertyFloat", name, "Contact Material", description)
            setattr(obj, name, value)
        obj.addProperty("App::PropertyEnumeration", "ShapeOverride", "RigidBody",
                        "Override the auto-detected collision shape type. "
                        "'Auto' uses the shape detected from geometry. "
                        "'cylindrical_shell' = inward-facing analytical cylinder "
                        "for a Fixed bed (requires CPU parallelization). "
                        "'mesh' = concave BVH (static only). "
                        "'convex_hull' = convex hull (works for dynamic bodies).")
        obj.ShapeOverride = ["Auto", "box", "sphere", "cylinder",
                             "cylindrical_shell", "convex_hull", "mesh"]
        obj.addProperty("App::PropertyLength", "CylindricalShellRadius",
                        "Analytical Cylindrical Shell",
                        "Inner collision radius. Set to 0 mm to infer the "
                        "smallest cylindrical-face radius.")
        obj.CylindricalShellRadius = 0.0
        obj.addProperty("App::PropertyLength", "CylindricalShellHeight",
                        "Analytical Cylindrical Shell",
                        "Container collision height. Set to 0 mm to use the "
                        "body-local Z extent.")
        obj.CylindricalShellHeight = 0.0
        obj.addProperty("App::PropertyLength", "CylindricalShellAxialOffset",
                        "Analytical Cylindrical Shell",
                        "Offset of the shell centre along body-local +Z. Use "
                        "this with a reduced height to align a thick cavity floor.")
        obj.CylindricalShellAxialOffset = 0.0
        obj.addProperty("App::PropertyBool", "CylindricalShellBottomCap",
                        "Analytical Cylindrical Shell",
                        "Close the local -Z end with an inward-facing flat cap.")
        obj.CylindricalShellBottomCap = True
        obj.addProperty("App::PropertyBool", "CylindricalShellTopCap",
                        "Analytical Cylindrical Shell",
                        "Close the local +Z end with an inward-facing flat cap.")
        obj.CylindricalShellTopCap = False
        obj.Proxy = self

    def execute(self, obj):
        pass

    def onDocumentRestored(self, obj):
        if not hasattr(obj, "Enabled"):
            obj.addProperty("App::PropertyBool", "Enabled", "RigidBody",
                            "Include this body in Chrono Physics simulations")
            obj.Enabled = True
        # Documents created before the Chrono terminology cleanup stored
        # Bullet-style Active/Passive enum entries. Preserve their intent.
        selected = canonical_body_type(getattr(obj, "BodyType", BODY_TYPE_DYNAMIC))
        obj.BodyType = [BODY_TYPE_DYNAMIC, BODY_TYPE_FIXED]
        obj.BodyType = selected
        legacy_friction = getattr(obj, "Friction", 0.5)
        for name, value in (
                ("StaticFriction", legacy_friction),
                ("SlidingFriction", legacy_friction),
                ("RollingFriction", 0.0), ("SpinningFriction", 0.0),
                ("Cohesion", 0.0), ("ContactDamping", 0.0),
                ("NormalCompliance", 0.0), ("TangentialCompliance", 0.0),
                ("YoungModulus", 2.0e7), ("PoissonRatio", 0.3),
                ("Adhesion", 0.0), ("NormalStiffness", 0.0),
                ("TangentialStiffness", 0.0), ("NormalDamping", 0.0),
                ("TangentialDamping", 0.0)):
            if not hasattr(obj, name):
                obj.addProperty("App::PropertyFloat", name, "Contact Material")
                setattr(obj, name, value)
        # Friction was superseded by Static/SlidingFriction and tessellation is
        # now intentionally global, controlled by the Chrono System only.
        for name in ("Friction", "MeshResolution"):
            if hasattr(obj, name):
                obj.removeProperty(name)
        selected_shape = getattr(obj, "ShapeOverride", "Auto")
        shape_options = ["Auto", "box", "sphere", "cylinder",
                         "cylindrical_shell", "convex_hull", "mesh"]
        obj.ShapeOverride = shape_options
        obj.ShapeOverride = (selected_shape if selected_shape in shape_options
                             else "Auto")
        for name, property_type, description, default in (
                ("CylindricalShellRadius", "App::PropertyLength",
                 "Inner collision radius; 0 mm selects automatic detection", 0.0),
                ("CylindricalShellHeight", "App::PropertyLength",
                 "Collision height; 0 mm uses the body-local Z extent", 0.0),
                ("CylindricalShellAxialOffset", "App::PropertyLength",
                 "Shell-centre offset along body-local +Z", 0.0),
                ("CylindricalShellBottomCap", "App::PropertyBool",
                 "Close the body-local -Z end", True),
                ("CylindricalShellTopCap", "App::PropertyBool",
                 "Close the body-local +Z end", False)):
            if not hasattr(obj, name):
                obj.addProperty(property_type, name,
                                "Analytical Cylindrical Shell", description)
                setattr(obj, name, default)

    def onChanged(self, obj, prop):
        if prop == "BodyType" and getattr(obj, "BodyLink", None) is not None:
            from ..preferences.ChronoPreferences import apply_body_color
            apply_body_color(obj.BodyLink, obj.BodyType)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class RigidBodyViewProvider:
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
        obj = self.Object
        name = ("AddPassiveBody.svg"
                if hasattr(obj, "BodyType")
                and canonical_body_type(obj.BodyType) == BODY_TYPE_FIXED
                else "AddActiveBody.svg")
        return os.path.join(ChronoUtils.ICONS_PATH, name)

    def claimChildren(self):
        obj = self.Object
        if hasattr(obj, "BodyLink") and obj.BodyLink is not None:
            return [obj.BodyLink]
        return []

    def onChanged(self, vobj, prop):
        if prop == "Visibility":
            obj = self.Object
            if hasattr(obj, "BodyLink") and obj.BodyLink is not None:
                try:
                    obj.BodyLink.ViewObject.Visibility = vobj.Visibility
                except Exception:
                    pass

    def onDelete(self, vobj, subelements):
        return True

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def make_rigid_body(original_obj, body_type=BODY_TYPE_DYNAMIC, container=None, enabled=True):
    """
    Create an App::Link clone of *original_obj* and wrap it in a RigidBody
    feature.  The original is never modified.  Both are added to *container*
    if provided.  *enabled* permits future tools to create an inactive body
    that is excluded from simulation until explicitly enabled.
    """
    doc = FreeCAD.ActiveDocument

    # --- App::Link (the simulation-driven clone) ---
    link = doc.addObject("App::Link", f"Link_{original_obj.Label}")
    link.setLink(original_obj)
    link.Placement = original_obj.Placement.copy()
    link.Label = f"Link_{original_obj.Label}"

    # --- RigidBody feature ---
    rb = doc.addObject("App::FeaturePython", f"RigidBody_{original_obj.Label}")
    RigidBodyFeature(rb)
    rb.OriginalObject = original_obj
    rb.BodyLink = link
    rb.BodyType = canonical_body_type(body_type)
    rb.Enabled = bool(enabled)
    rb.Label = f"RigidBody_{original_obj.Label}"

    from ..preferences.ChronoPreferences import apply_body_color
    apply_body_color(link, rb.BodyType)

    if FreeCAD.GuiUp:
        import FreeCADGui
        RigidBodyViewProvider(rb.ViewObject)

    # --- Register in container ---
    if container is not None:
        current = list(container.RigidBodies)
        current.append(rb)
        container.RigidBodies = current

    doc.recompute()
    return rb
