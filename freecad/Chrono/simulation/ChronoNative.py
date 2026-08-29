"""Small PyBullet-compatible facade implemented with Project Chrono.

The workbench UI and document model intentionally share the same workflow as
FreeCAD-BulletPhysics.  Keeping that mature simulation orchestration code and
adapting its small physics API surface here makes feature parity explicit while
all integration, contact resolution, and rigid-body state come from Chrono.
"""

from __future__ import annotations

import math
import os

import pychrono as chrono


DIRECT = 0
GEOM_SPHERE = 2
GEOM_BOX = 3
GEOM_CYLINDER = 4
GEOM_MESH = 5
GEOM_COMPOUND = 6
GEOM_CONCAVE_INTERNAL_EDGE = 1


class _Client:
    def __init__(self, cpu_parallel=True):
        self.system = chrono.ChSystemNSC()
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


def _client(client_id):
    try:
        return _clients[client_id]
    except KeyError as exc:
        raise ValueError(f"Unknown Chrono client {client_id}") from exc


def _vec(values):
    return chrono.ChVector3d(float(values[0]), float(values[1]), float(values[2]))


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
        return chrono.ChCollisionShapeCylinder(
            material, meta["radius"], meta["height"])
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
        body.AddCollisionShape(_build_shape(meta, material, is_static))
        return
    for child_id, position, orientation in zip(
            meta["children"], meta["positions"], meta["orientations"]):
        child = _build_shape(client.shapes[child_id], material, is_static)
        frame = chrono.ChFramed(_vec(position), _quat_xyzw(orientation))
        body.AddCollisionShape(child, frame)


def connect(_mode=DIRECT, cpuParallel=True):
    global _next_client_id
    client_id = _next_client_id
    _next_client_id += 1
    _clients[client_id] = _Client(cpuParallel)
    return client_id


def disconnect(physicsClientId=0):
    _clients.pop(physicsClientId, None)


def setGravity(x, y, z, physicsClientId=0):
    _client(physicsClientId).system.SetGravitationalAcceleration(
        chrono.ChVector3d(float(x), float(y), float(z)))


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


def createCollisionShape(shapeType, physicsClientId=0, **parameters):
    client = _client(physicsClientId)
    shape_id = client.next_shape_id
    client.next_shape_id += 1
    meta = {"kind": shapeType, "client": client}
    if shapeType == GEOM_BOX:
        meta["half_extents"] = tuple(float(v) for v in parameters["halfExtents"])
    elif shapeType == GEOM_SPHERE:
        meta["radius"] = float(parameters["radius"])
    elif shapeType == GEOM_CYLINDER:
        meta["radius"] = float(parameters["radius"])
        meta["height"] = float(parameters["height"])
    elif shapeType == GEOM_MESH:
        meta["vertices"] = [tuple(float(v) for v in point)
                            for point in parameters["vertices"]]
        indices = parameters.get("indices")
        meta["indices"] = tuple(int(i) for i in indices) if indices else ()
        meta["flags"] = int(parameters.get("flags", 0))
    elif shapeType == GEOM_COMPOUND:
        meta["children"] = tuple(parameters.get("children", ()))
        meta["positions"] = tuple(
            tuple(float(v) for v in position)
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
    mass = max(0.0, float(baseMass))
    body = chrono.ChBody()
    body.SetName(f"freecad-chrono:{body_id}")
    body.SetPos(_vec(basePosition))
    body.SetRot(_quat_xyzw(baseOrientation))
    material = chrono.ChContactMaterialNSC()
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
        body, material, baseCollisionShapeIndex, mass)
    return body_id


def changeDynamics(bodyUniqueId, _linkIndex, physicsClientId=0, **parameters):
    client = _client(physicsClientId)
    state = client.bodies[bodyUniqueId]
    body = state.body
    if "restitution" in parameters:
        state.material.SetRestitution(float(parameters["restitution"]))
    if "lateralFriction" in parameters:
        state.material.SetFriction(float(parameters["lateralFriction"]))
    if "collisionMargin" in parameters:
        margin = max(0.0, float(parameters["collisionMargin"]))
        body.GetCollisionModel().SetEnvelope(margin)
        body.GetCollisionModel().SetSafeMargin(margin)
    if "linearDamping" in parameters:
        state.linear_damping = max(0.0, float(parameters["linearDamping"]))
    if "angularDamping" in parameters:
        state.angular_damping = max(0.0, float(parameters["angularDamping"]))
    if "mass" in parameters:
        mass = max(0.0, float(parameters["mass"]))
        if mass > 0.0:
            state.dynamic_mass = mass
            body.SetMass(mass)
            body.SetInertiaXX(_vec(_inertia(client.shapes[state.shape_id], mass)))
            body.SetFixed(False)
        else:
            body.SetFixed(True)


def resetBaseVelocity(bodyUniqueId, linearVelocity=None, angularVelocity=None,
                      physicsClientId=0):
    body = _client(physicsClientId).bodies[bodyUniqueId].body
    if linearVelocity is not None:
        body.SetLinVel(_vec(linearVelocity))
    if angularVelocity is not None:
        body.SetAngVelParent(_vec(angularVelocity))


def getBaseVelocity(bodyUniqueId, physicsClientId=0):
    body = _client(physicsClientId).bodies[bodyUniqueId].body
    linear = body.GetLinVel()
    angular = body.GetAngVelParent()
    return ((linear.x, linear.y, linear.z),
            (angular.x, angular.y, angular.z))


def getBasePositionAndOrientation(bodyUniqueId, physicsClientId=0):
    body = _client(physicsClientId).bodies[bodyUniqueId].body
    position = body.GetPos()
    rotation = body.GetRot()
    return ((position.x, position.y, position.z),
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
    if separation <= radius_a + radius_b + max(0.0, float(distance)):
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
    # ChBody in Chrono 10 does not expose direct damping coefficients through
    # PyChrono, so apply the configured global damping deterministically.
    for state in client.bodies.values():
        if state.body.IsFixed():
            continue
        if state.linear_damping:
            velocity = state.body.GetLinVel()
            factor = max(0.0, 1.0 - state.linear_damping * dt)
            state.body.SetLinVel(chrono.ChVector3d(
                velocity.x * factor, velocity.y * factor, velocity.z * factor))
        if state.angular_damping:
            velocity = state.body.GetAngVelParent()
            factor = max(0.0, 1.0 - state.angular_damping * dt)
            state.body.SetAngVelParent(chrono.ChVector3d(
                velocity.x * factor, velocity.y * factor, velocity.z * factor))
    client.system.DoStepDynamics(dt)
