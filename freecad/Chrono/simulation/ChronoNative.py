"""Small PyBullet-compatible facade implemented with Project Chrono.

The workbench UI and document model intentionally share the same workflow as
FreeCAD-BulletPhysics.  Keeping that mature simulation orchestration code and
adapting its small physics API surface here makes feature parity explicit while
all integration, contact resolution, and rigid-body state come from Chrono.
"""

from __future__ import annotations

import math
import os
import re

import pychrono as chrono


DIRECT = 0
GEOM_SPHERE = 2
GEOM_BOX = 3
GEOM_CYLINDER = 4
GEOM_MESH = 5
GEOM_COMPOUND = 6
GEOM_CYLINDRICAL_SHELL = 7
GEOM_CONCAVE_INTERNAL_EDGE = 1

# Chrono accepts any consistent unit system, while its NSC solver and collision
# defaults are tuned for values near unity.  FreeCAD models are commonly only a
# few millimetres across; solving them internally in millimetres avoids tiny
# inertias (around 1e-12 kg*m^2) without changing the metre-based adapter API.
_LENGTH_SCALE = 1000.0
# Keep the internal units coherent: metres -> millimetres and kilograms ->
# milligrams.  Scaling only length left sub-millimetre particles with masses
# around 1e-6 in Chrono's solver and could make dense NSC contacts diverge.
# Uniform mass scaling preserves accelerations, restitution, and friction.
_MASS_SCALE = 1.0e6
_CONTACT_STEP_SECONDS_PER_MM = 2.0e-4


class _Client:
    def __init__(self, cpu_parallel=True, contact_model="NSC"):
        self.contact_model = contact_model if contact_model == "SMC" else "NSC"
        self.system = (chrono.ChSystemSMC() if self.contact_model == "SMC"
                       else chrono.ChSystemNSC())
        collision_type = (
            chrono.ChCollisionSystem.Type_MULTICORE
            if cpu_parallel
            else chrono.ChCollisionSystem.Type_BULLET
        )
        self.system.SetCollisionSystemType(collision_type)
        self.cpu_parallel = bool(cpu_parallel)
        self.time_step = 1.0 / 240.0
        self.shapes = {}
        self.bodies = {}
        self.next_shape_id = 1
        self.next_body_id = 1
        if self.cpu_parallel:
            self.system.SetNumThreads(max(1, os.cpu_count() or 1))


class _BodyState:
    def __init__(self, body, material, shape_id, mass):
        self.body = body
        self.material = material
        self.shape_id = shape_id
        self.dynamic_mass = max(0.0, float(mass))
        self.linear_damping = 0.0
        self.angular_damping = 0.0
        self.query_only = False


_clients = {}
_next_client_id = 1


def _chrono_version():
    """Best-effort version lookup for PyChrono builds that expose no constant."""
    for name in ("CHRONO_VERSION", "__version__"):
        value = getattr(chrono, name, None)
        if value:
            return str(value)
    for header in (
            "/usr/include/chrono/ChVersion.h",
            "/usr/local/include/chrono/ChVersion.h"):
        try:
            with open(header, encoding="utf-8") as stream:
                match = re.search(
                    r'^#define\s+CHRONO_VERSION\s+"([^"]+)"',
                    stream.read(), re.MULTILINE)
            if match:
                return match.group(1)
        except OSError:
            continue
    return "unknown"


def getCapabilities():
    """Return JSON-safe runtime facts used by the FreeCAD setup detector."""
    collision_system = getattr(chrono, "ChCollisionSystem", None)
    return {
        "chrono_version": _chrono_version(),
        "multicore_collision_available": bool(
            collision_system is not None
            and hasattr(collision_system, "Type_MULTICORE")),
        "multicore_nsc_system_available": hasattr(
            chrono, "ChSystemMulticoreNSC"),
        "capped_cylindrical_shell_available": bool(
            hasattr(chrono, "ChCollisionShapeCylindricalShell")
            and hasattr(chrono.ChCollisionShapeCylindricalShell,
                        "IsBottomClosed")),
        "thread_count": max(1, os.cpu_count() or 1),
        # Chrono 10 has dedicated triangle narrow-phase paths for these shapes.
        # Cylinder/capsule/convex contacts use its generic path, which can
        # incorrectly bridge openings in a concave mesh.
        "multicore_concave_dynamic_shapes": ["sphere", "box"],
    }


def _client(client_id):
    try:
        return _clients[client_id]
    except KeyError as exc:
        raise ValueError(f"Unknown Chrono client {client_id}") from exc


def _vec(values):
    return chrono.ChVector3d(float(values[0]), float(values[1]), float(values[2]))


def _length_vec(values):
    return chrono.ChVector3d(
        float(values[0]) * _LENGTH_SCALE,
        float(values[1]) * _LENGTH_SCALE,
        float(values[2]) * _LENGTH_SCALE,
    )


def _quat_xyzw(values):
    return chrono.ChQuaterniond(
        float(values[3]), float(values[0]), float(values[1]), float(values[2]))


def _shape_radius(meta):
    kind = meta["kind"]
    if kind == GEOM_SPHERE:
        return meta["radius"]
    if kind == GEOM_BOX:
        return math.sqrt(sum(value * value for value in meta["half_extents"]))
    if kind == GEOM_CYLINDER:
        return math.hypot(meta["radius"], meta["height"] * 0.5)
    if kind == GEOM_CYLINDRICAL_SHELL:
        return math.hypot(meta["radius"], meta["height"] * 0.5)
    if kind == GEOM_MESH:
        return max((math.sqrt(sum(value * value for value in point))
                    for point in meta["vertices"]), default=0.0)
    if kind == GEOM_COMPOUND:
        result = 0.0
        for shape_id, position in zip(meta["children"], meta["positions"]):
            child = meta["client"].shapes[shape_id]
            result = max(result, _shape_radius(child)
                         + math.sqrt(sum(value * value for value in position)))
        return result
    return 0.0


def _shape_min_extent(meta):
    """Return the smallest useful half-size for collision-margin limiting."""
    kind = meta["kind"]
    if kind == GEOM_SPHERE:
        return meta["radius"]
    if kind == GEOM_BOX:
        return min(meta["half_extents"])
    if kind == GEOM_CYLINDER:
        return min(meta["radius"], meta["height"] * 0.5)
    if kind == GEOM_CYLINDRICAL_SHELL:
        return min(meta["radius"], meta["height"] * 0.5)
    if kind == GEOM_MESH:
        vertices = meta["vertices"]
        if not vertices:
            return 0.0
        spans = [max(point[axis] for point in vertices)
                 - min(point[axis] for point in vertices)
                 for axis in range(3)]
        positive = [span * 0.5 for span in spans if span > 0.0]
        return min(positive, default=0.0)
    if kind == GEOM_COMPOUND:
        return min(
            (_shape_min_extent(meta["client"].shapes[shape_id])
             for shape_id in meta["children"]),
            default=0.0,
        )
    return 0.0


def _inertia(meta, mass):
    """Return a stable diagonal inertia estimate for a collision shape."""
    mass = max(float(mass), 1e-12)
    kind = meta["kind"]
    if kind == GEOM_BOX:
        x, y, z = (2.0 * value for value in meta["half_extents"])
        return ((mass / 12.0) * (y * y + z * z),
                (mass / 12.0) * (x * x + z * z),
                (mass / 12.0) * (x * x + y * y))
    if kind == GEOM_SPHERE:
        value = 0.4 * mass * meta["radius"] ** 2
        return (value, value, value)
    if kind == GEOM_CYLINDER:
        radius = meta["radius"]
        height = meta["height"]
        radial = mass * (3.0 * radius * radius + height * height) / 12.0
        axial = 0.5 * mass * radius * radius
        return (radial, radial, axial)
    if kind == GEOM_CYLINDRICAL_SHELL:
        radius = meta["radius"]
        height = meta["height"]
        radial = mass * (6.0 * radius * radius + height * height) / 12.0
        axial = mass * radius * radius
        return (radial, radial, axial)
    # A conservative spherical estimate is robust for arbitrary CAD meshes and
    # compounds.  Chrono requires an inertia tensor whereas PyBullet inferred it.
    radius = max(_shape_radius(meta), 1e-6)
    value = 0.4 * mass * radius * radius
    return (value, value, value)


def _build_shape(meta, material, is_static):
    kind = meta["kind"]
    if kind == GEOM_BOX:
        hx, hy, hz = meta["half_extents"]
        return chrono.ChCollisionShapeBox(material, 2 * hx, 2 * hy, 2 * hz)
    if kind == GEOM_SPHERE:
        return chrono.ChCollisionShapeSphere(material, meta["radius"])
    if kind == GEOM_CYLINDER:
        if meta.get("analytical", False):
            return chrono.ChCollisionShapeCylinder(
                material, meta["radius"], meta["height"])
        # Preserve the workaround used for legacy triangle-mesh beds.  The
        # capped analytical shell explicitly opts cylinders into the true
        # Chrono primitive so the multicore cylinder-shell dispatcher is used.
        points = chrono.vector_ChVector3d()
        radius = meta["radius"]
        half_height = meta["height"] * 0.5
        for z in (-half_height, half_height):
            for index in range(24):
                angle = 2.0 * math.pi * index / 24.0
                points.push_back(chrono.ChVector3d(
                    radius * math.cos(angle), radius * math.sin(angle), z))
        return chrono.ChCollisionShapeConvexHull(material, points)
    if kind == GEOM_CYLINDRICAL_SHELL:
        return chrono.ChCollisionShapeCylindricalShell(
            material, meta["radius"], meta["height"],
            meta["closed_bottom"], meta["closed_top"])
    if kind == GEOM_MESH:
        vertices = meta["vertices"]
        indices = meta.get("indices")
        if indices:
            mesh = chrono.ChTriangleMeshConnected()
            for offset in range(0, len(indices), 3):
                mesh.AddTriangle(
                    _vec(vertices[indices[offset]]),
                    _vec(vertices[indices[offset + 1]]),
                    _vec(vertices[indices[offset + 2]]),
                )
            # AddTriangle stores independent vertices.  Weld coincident CAD
            # vertices so shared mesh edges do not produce duplicate contact
            # constraints (a major source of NSC divergence in dense fills).
            mesh.RepairDuplicateVertexes(1.0e-9)
            return chrono.ChCollisionShapeTriangleMesh(
                material, mesh, bool(is_static), False, 0.0)
        points = chrono.vector_ChVector3d()
        for point in vertices:
            points.push_back(_vec(point))
        return chrono.ChCollisionShapeConvexHull(material, points)
    raise ValueError(f"Unsupported individual Chrono shape type {kind}")


def _add_shape_to_body(client, body, shape_id, material, is_static):
    meta = client.shapes[shape_id]
    if meta["kind"] != GEOM_COMPOUND:
        shape = _build_shape(meta, material, is_static)
        if meta["kind"] == GEOM_CYLINDRICAL_SHELL:
            frame = chrono.ChFramed(
                chrono.ChVector3d(0.0, 0.0, meta["axial_offset"]),
                chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0))
            body.AddCollisionShape(shape, frame)
        else:
            body.AddCollisionShape(shape)
        return
    for child_id, position, orientation in zip(
            meta["children"], meta["positions"], meta["orientations"]):
        child = _build_shape(client.shapes[child_id], material, is_static)
        frame = chrono.ChFramed(_vec(position), _quat_xyzw(orientation))
        body.AddCollisionShape(child, frame)


def connect(_mode=DIRECT, cpuParallel=True, contactModel="NSC"):
    global _next_client_id
    client_id = _next_client_id
    _next_client_id += 1
    _clients[client_id] = _Client(cpuParallel, contactModel)
    return client_id


def disconnect(physicsClientId=0):
    _clients.pop(physicsClientId, None)


def setGravity(x, y, z, physicsClientId=0):
    _client(physicsClientId).system.SetGravitationalAcceleration(
        chrono.ChVector3d(
            float(x) * _LENGTH_SCALE,
            float(y) * _LENGTH_SCALE,
            float(z) * _LENGTH_SCALE,
        ))


def setTimeStep(timeStep, physicsClientId=0):
    _client(physicsClientId).time_step = float(timeStep)


def setPhysicsEngineParameter(physicsClientId=0, **parameters):
    client = _client(physicsClientId)
    if "numSolverIterations" in parameters:
        solver = client.system.GetSolver()
        if solver.IsIterative():
            solver.AsIterative().SetMaxIterations(
                max(1, int(parameters["numSolverIterations"])))
    if "numSolverThreads" in parameters:
        threads = max(1, int(parameters["numSolverThreads"]))
        client.system.SetNumThreads(threads)
    if "maximumPenetrationRecoverySpeed" in parameters:
        client.system.SetMaxPenetrationRecoverySpeed(max(
            0.0, float(parameters["maximumPenetrationRecoverySpeed"]) * _LENGTH_SCALE))


def createCollisionShape(shapeType, physicsClientId=0, **parameters):
    client = _client(physicsClientId)
    shape_id = client.next_shape_id
    client.next_shape_id += 1
    meta = {"kind": shapeType, "client": client}
    if shapeType == GEOM_BOX:
        meta["half_extents"] = tuple(
            float(v) * _LENGTH_SCALE for v in parameters["halfExtents"])
    elif shapeType == GEOM_SPHERE:
        meta["radius"] = float(parameters["radius"]) * _LENGTH_SCALE
    elif shapeType == GEOM_CYLINDER:
        meta["radius"] = float(parameters["radius"]) * _LENGTH_SCALE
        meta["height"] = float(parameters["height"]) * _LENGTH_SCALE
        meta["analytical"] = bool(parameters.get("analytical", False))
    elif shapeType == GEOM_CYLINDRICAL_SHELL:
        meta["radius"] = float(parameters["radius"]) * _LENGTH_SCALE
        meta["height"] = float(parameters["height"]) * _LENGTH_SCALE
        meta["axial_offset"] = float(
            parameters.get("axialOffset", 0.0)) * _LENGTH_SCALE
        meta["closed_bottom"] = bool(parameters.get("closedBottom", True))
        meta["closed_top"] = bool(parameters.get("closedTop", False))
    elif shapeType == GEOM_MESH:
        meta["vertices"] = [tuple(float(v) * _LENGTH_SCALE for v in point)
                            for point in parameters["vertices"]]
        indices = parameters.get("indices")
        meta["indices"] = tuple(int(i) for i in indices) if indices else ()
        meta["flags"] = int(parameters.get("flags", 0))
    elif shapeType == GEOM_COMPOUND:
        meta["children"] = tuple(parameters.get("children", ()))
        meta["positions"] = tuple(
            tuple(float(v) * _LENGTH_SCALE for v in position)
            for position in parameters.get("childPositions", ()))
        meta["orientations"] = tuple(
            tuple(float(v) for v in orientation)
            for orientation in parameters.get("childOrientations", ()))
    else:
        raise ValueError(f"Unsupported Chrono collision shape type {shapeType}")
    client.shapes[shape_id] = meta
    return shape_id


def createMultiBody(baseMass, baseCollisionShapeIndex, basePosition,
                    baseOrientation=(0.0, 0.0, 0.0, 1.0), physicsClientId=0):
    client = _client(physicsClientId)
    body_id = client.next_body_id
    client.next_body_id += 1
    external_mass = max(0.0, float(baseMass))
    mass = external_mass * _MASS_SCALE
    body = chrono.ChBody()
    body.SetName(f"freecad-chrono:{body_id}")
    body.SetPos(_length_vec(basePosition))
    body.SetRot(_quat_xyzw(baseOrientation))
    material = (chrono.ChContactMaterialSMC()
                if client.contact_model == "SMC"
                else chrono.ChContactMaterialNSC())
    _add_shape_to_body(
        client, body, baseCollisionShapeIndex, material, mass <= 0.0)
    body.EnableCollision(True)
    body.SetFixed(mass <= 0.0)
    if mass > 0.0:
        body.SetMass(mass)
        body.SetInertiaXX(_vec(_inertia(client.shapes[baseCollisionShapeIndex], mass)))
    client.system.AddBody(body)
    # ChSystem binds all pre-existing collision models during its first
    # initialization. Bodies created later (emitters) must be explicitly bound
    # to the already-initialized collision system or they participate in
    # dynamics but pass through every collider. Do not bind early: the
    # MULTICORE collision backend would bind them again during initialization
    # and report duplicate contacts.
    collision_system = client.system.GetCollisionSystem()
    if collision_system is not None and collision_system.IsInitialized():
        collision_system.BindItem(body)
    client.bodies[body_id] = _BodyState(
        body, material, baseCollisionShapeIndex, external_mass)
    return body_id


def changeDynamics(bodyUniqueId, _linkIndex, physicsClientId=0, **parameters):
    client = _client(physicsClientId)
    state = client.bodies[bodyUniqueId]
    body = state.body
    if "restitution" in parameters:
        state.material.SetRestitution(float(parameters["restitution"]))
    if "lateralFriction" in parameters:
        state.material.SetFriction(float(parameters["lateralFriction"]))
    for parameter, setter in (
            ("staticFriction", "SetStaticFriction"),
            ("slidingFriction", "SetSlidingFriction"),
            ("rollingFriction", "SetRollingFriction"),
            ("spinningFriction", "SetSpinningFriction"),
            ("cohesion", "SetCohesion"),
            ("contactDamping", "SetDampingF"),
            ("normalCompliance", "SetCompliance"),
            ("tangentialCompliance", "SetComplianceT"),
            ("youngModulus", "SetYoungModulus"),
            ("poissonRatio", "SetPoissonRatio"),
            ("adhesion", "SetAdhesion"),
            ("normalStiffness", "SetKn"),
            ("tangentialStiffness", "SetKt"),
            ("normalDamping", "SetGn"),
            ("tangentialDamping", "SetGt")):
        if parameter in parameters and hasattr(state.material, setter):
            getattr(state.material, setter)(float(parameters[parameter]))
    if "collisionMargin" in parameters:
        requested = max(
            0.0, float(parameters["collisionMargin"]) * _LENGTH_SCALE)
        # Chrono recommends an NSC envelope of 5-10% of the smallest shape.
        # A global 1 mm margin, for example, is twice the radius of the rods in
        # packed_bed_void_chrono.FCStd and makes contact constraints singular.
        extent = _shape_min_extent(client.shapes[state.shape_id])
        if extent > 0.0:
            # Preserve a user-visible zero margin while retaining the tiny
            # non-zero safe margin required by Chrono's collision algorithms.
            # Cap it so a large concave container is not shrunk appreciably.
            numerical_margin = min(extent * 0.05, 0.025)
            margin = numerical_margin if requested == 0.0 else min(
                requested, numerical_margin)
        else:
            margin = requested
        body.GetCollisionModel().SetEnvelope(margin)
        body.GetCollisionModel().SetSafeMargin(margin)
    if "linearDamping" in parameters:
        state.linear_damping = max(0.0, float(parameters["linearDamping"]))
    if "angularDamping" in parameters:
        state.angular_damping = max(0.0, float(parameters["angularDamping"]))
    if "enableVelocityLimit" in parameters:
        body.SetLimitSpeed(bool(parameters["enableVelocityLimit"]))
    if "maximumLinearVelocity" in parameters:
        body.SetMaxLinVel(max(
            0.0, float(parameters["maximumLinearVelocity"]) * _LENGTH_SCALE))
    if "maximumAngularVelocity" in parameters:
        body.SetMaxAngVel(max(0.0, float(parameters["maximumAngularVelocity"])))
    if "sleepingAllowed" in parameters:
        body.SetSleepingAllowed(bool(parameters["sleepingAllowed"]))
    if "sleepTime" in parameters:
        body.SetSleepTime(max(0.0, float(parameters["sleepTime"])))
    if "sleepLinearVelocity" in parameters:
        body.SetSleepMinLinVel(max(
            0.0, float(parameters["sleepLinearVelocity"]) * _LENGTH_SCALE))
    if "sleepAngularVelocity" in parameters:
        body.SetSleepMinAngVel(max(0.0, float(parameters["sleepAngularVelocity"])))
    if "disableGyroscopicTorque" in parameters:
        body.SetUseGyroTorque(not bool(parameters["disableGyroscopicTorque"]))
    if "mass" in parameters:
        external_mass = max(0.0, float(parameters["mass"]))
        mass = external_mass * _MASS_SCALE
        if external_mass > 0.0:
            state.dynamic_mass = external_mass
            body.SetMass(mass)
            body.SetInertiaXX(_vec(_inertia(client.shapes[state.shape_id], mass)))
            body.SetFixed(False)
        else:
            body.SetFixed(True)


def resetBaseVelocity(bodyUniqueId, linearVelocity=None, angularVelocity=None,
                      physicsClientId=0):
    body = _client(physicsClientId).bodies[bodyUniqueId].body
    if linearVelocity is not None:
        body.SetLinVel(_length_vec(linearVelocity))
    if angularVelocity is not None:
        body.SetAngVelParent(_vec(angularVelocity))


def getBaseVelocity(bodyUniqueId, physicsClientId=0):
    body = _client(physicsClientId).bodies[bodyUniqueId].body
    linear = body.GetLinVel()
    angular = body.GetAngVelParent()
    return ((linear.x / _LENGTH_SCALE,
             linear.y / _LENGTH_SCALE,
             linear.z / _LENGTH_SCALE),
            (angular.x, angular.y, angular.z))


def getBasePositionAndOrientation(bodyUniqueId, physicsClientId=0):
    body = _client(physicsClientId).bodies[bodyUniqueId].body
    position = body.GetPos()
    rotation = body.GetRot()
    return ((position.x / _LENGTH_SCALE,
             position.y / _LENGTH_SCALE,
             position.z / _LENGTH_SCALE),
            (rotation.e1, rotation.e2, rotation.e3, rotation.e0))


def setCollisionFilterGroupMask(bodyUniqueId, _linkIndex, collisionFilterGroup,
                                collisionFilterMask, physicsClientId=0):
    state = _client(physicsClientId).bodies[bodyUniqueId]
    state.query_only = not collisionFilterGroup or not collisionFilterMask
    state.body.EnableCollision(not state.query_only)


def _body_id(contactable):
    try:
        name = contactable.GetPhysicsItem().GetName()
        if name.startswith("freecad-chrono:"):
            return int(name.rsplit(":", 1)[1])
    except Exception:
        pass
    return None


def getContactPoints(bodyA=None, bodyB=None, physicsClientId=0):
    client = _client(physicsClientId)
    contacts = []

    class Reporter(chrono.ReportContactCallback):
        def __init__(self):
            super().__init__()

        def OnReportContact(self, _p_a, _p_b, _plane, _distance, _radius,
                            _forces, _torques, object_a, object_b, _offset):
            id_a = _body_id(object_a)
            id_b = _body_id(object_b)
            if id_a is None or id_b is None:
                return True
            if bodyA is not None:
                if id_a == bodyA and (bodyB is None or id_b == bodyB):
                    contacts.append((0, bodyA, id_b, -1, -1))
                elif id_b == bodyA and (bodyB is None or id_a == bodyB):
                    contacts.append((0, bodyA, id_a, -1, -1))
            elif bodyB is None or id_a == bodyB or id_b == bodyB:
                contacts.append((0, id_a, id_b, -1, -1))
            return True

    client.system.GetContactContainer().ReportAllContacts(Reporter())
    return contacts


def getClosestPoints(bodyA, bodyB, distance=0.0, physicsClientId=0):
    """Conservative proximity query used by non-colliding observer volumes."""
    client = _client(physicsClientId)
    state_a = client.bodies.get(bodyA)
    state_b = client.bodies.get(bodyB)
    if state_a is None or state_b is None:
        return []
    pos_a = state_a.body.GetPos()
    pos_b = state_b.body.GetPos()
    separation = math.sqrt(
        (pos_a.x - pos_b.x) ** 2
        + (pos_a.y - pos_b.y) ** 2
        + (pos_a.z - pos_b.z) ** 2)
    radius_a = _shape_radius(client.shapes[state_a.shape_id])
    radius_b = _shape_radius(client.shapes[state_b.shape_id])
    scaled_distance = max(0.0, float(distance)) * _LENGTH_SCALE
    if separation <= radius_a + radius_b + scaled_distance:
        return [(0, bodyA, bodyB, -1, -1)]
    return []


def removeBody(bodyUniqueId, physicsClientId=0):
    client = _client(physicsClientId)
    state = client.bodies.get(bodyUniqueId)
    if state is None:
        return

    collision_system = client.system.GetCollisionSystem()
    if (client.cpu_parallel and collision_system is not None
            and collision_system.IsInitialized()):
        # Chrono 10's multicore collision system throws from RemoveBody after
        # initialization because ChCollisionSystemMulticore::Remove() is not
        # implemented. Retire a destroyed/emitted body instead: collision
        # masks make it inert immediately and moving the now-fixed body out of
        # the scene keeps it out of broad-phase queries as well.
        model = state.body.GetCollisionModel()
        for family in range(16):
            model.DisallowCollisionsWith(family)
        state.body.SetLinVel(chrono.ChVector3d(0.0, 0.0, 0.0))
        state.body.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, 0.0))
        state.body.SetFixed(True)
        retired = 1.0e6 + float(bodyUniqueId)
        state.body.SetPos(chrono.ChVector3d(retired, retired, retired))
        state.body.SyncCollisionModels()
    else:
        client.system.RemoveBody(state.body)
    client.bodies.pop(bodyUniqueId, None)


def stepSimulation(physicsClientId=0):
    client = _client(physicsClientId)
    dt = client.time_step
    # PyChrono 10 does not expose per-body CCD.  Refine only concave-mesh
    # scenes, with a scale-aware limit: a 0.5 mm particle gets a 0.1 ms
    # collision step, while ordinary metre-scale scenes avoid needless work.
    has_concave_mesh = any(
        shape["kind"] == GEOM_MESH and shape.get("indices")
        for shape in client.shapes.values())
    dynamic_extents = [
        _shape_min_extent(client.shapes[state.shape_id])
        for state in client.bodies.values()
        if not state.body.IsFixed()
    ]
    if has_concave_mesh and dynamic_extents:
        smallest_extent = min(
            (extent for extent in dynamic_extents if extent > 0.0),
            default=0.0)
        max_contact_step = (max(
            1.0e-5,
            smallest_extent * _CONTACT_STEP_SECONDS_PER_MM,
        ) if smallest_extent > 0.0 else dt)
    else:
        max_contact_step = dt
    internal_steps = max(1, int(math.ceil(dt / max_contact_step)))
    internal_dt = dt / internal_steps
    # ChBody in Chrono 10 does not expose direct damping coefficients through
    # PyChrono, so apply the configured global damping deterministically.
    for _internal_step in range(internal_steps):
        for state in client.bodies.values():
            if state.body.IsFixed():
                continue
            if state.linear_damping:
                velocity = state.body.GetLinVel()
                factor = max(0.0, 1.0 - state.linear_damping * internal_dt)
                state.body.SetLinVel(chrono.ChVector3d(
                    velocity.x * factor,
                    velocity.y * factor,
                    velocity.z * factor))
            if state.angular_damping:
                velocity = state.body.GetAngVelParent()
                factor = max(0.0, 1.0 - state.angular_damping * internal_dt)
                state.body.SetAngVelParent(chrono.ChVector3d(
                    velocity.x * factor,
                    velocity.y * factor,
                    velocity.z * factor))
        client.system.DoStepDynamics(internal_dt)
