"""Headless smoke test for the exact FreeCAD runtime used by the workbench."""

import math
import sys

import FreeCAD

from freecad.Chrono.simulation import ChronoAdapter as physics


print("FreeCAD:", FreeCAD.Version())
print("Python:", sys.version)
print("Backend:", physics.backend_description())

client = physics.connect(physics.DIRECT, cpuParallel=True)
try:
    print("Connected backend:", physics.backend_description())
    physics.setGravity(0.0, 0.0, -9.81, physicsClientId=client)
    physics.setTimeStep(0.005, physicsClientId=client)
    ground_shape = physics.createCollisionShape(
        physics.GEOM_BOX, halfExtents=(2.0, 2.0, 0.25),
        physicsClientId=client)
    sphere_shape = physics.createCollisionShape(
        physics.GEOM_SPHERE, radius=0.2, physicsClientId=client)
    ground = physics.createMultiBody(
        0.0, ground_shape, (0.0, 0.0, -0.25), physicsClientId=client)
    sphere = physics.createMultiBody(
        1.0, sphere_shape, (0.0, 0.0, 1.0), physicsClientId=client)
    for _step in range(200):
        physics.stepSimulation(physicsClientId=client)
    position, _rotation = physics.getBasePositionAndOrientation(
        sphere, physicsClientId=client)
    contacts = physics.getContactPoints(
        bodyA=ground, physicsClientId=client)
    assert abs(position[2] - 0.2) < 0.01, position
    assert contacts, "Chrono reported no ground contact"

    # Regression: emitter bodies are inserted after Chrono's collision system
    # has initialized. The late sphere must collide with the first sphere and
    # settle on top instead of falling through the scene.
    late_sphere = physics.createMultiBody(
        1.0, sphere_shape, (0.0, 0.0, 1.0), physicsClientId=client)
    for _step in range(250):
        physics.stepSimulation(physicsClientId=client)
    late_position, _rotation = physics.getBasePositionAndOrientation(
        late_sphere, physicsClientId=client)
    sphere_contacts = physics.getContactPoints(
        bodyA=sphere, bodyB=late_sphere, physicsClientId=client)
    assert 0.55 < late_position[2] < 0.65, late_position
    assert sphere_contacts, "Late-created sphere did not collide with first sphere"

    # Regression: millimetre cylinders must enter a concave container and
    # settle on its triangle floor without tunnelling or producing NaNs.
    mesh_shape = physics.createCollisionShape(
        physics.GEOM_MESH,
        vertices=[(-0.01, -0.01, 0.0),
                  (0.01, -0.01, 0.0),
                  (0.01, 0.01, 0.0),
                  (-0.01, 0.01, 0.0)],
        indices=(0, 1, 2, 0, 2, 3),
        physicsClientId=client,
    )
    rod_shape = physics.createCollisionShape(
        physics.GEOM_CYLINDER, radius=0.0005, height=0.005,
        physicsClientId=client,
    )
    mesh_floor = physics.createMultiBody(
        0.0, mesh_shape, (5.0, 0.0, 0.0), physicsClientId=client)
    rod = physics.createMultiBody(
        4.0e-6, rod_shape, (5.0, 0.0, 0.02), physicsClientId=client)
    physics.changeDynamics(
        mesh_floor, -1, collisionMargin=0.0, physicsClientId=client)
    physics.changeDynamics(
        rod, -1, collisionMargin=0.0, physicsClientId=client)
    for _step in range(200):
        physics.stepSimulation(physicsClientId=client)
    rod_position, _rotation = physics.getBasePositionAndOrientation(
        rod, physicsClientId=client)
    rod_contacts = physics.getContactPoints(
        bodyA=mesh_floor, bodyB=rod, physicsClientId=client)
    assert all(math.isfinite(value) for value in rod_position), rod_position
    assert 0.0003 < rod_position[2] < 0.003, rod_position
    assert rod_contacts, "Millimetre cylinder did not contact triangle floor"

    # Patched Chrono regression: true analytical cylinders must remain inside
    # an open-top capped cylindrical shell using MULTICORE collision.
    physics.setTimeStep(2.0e-5, physicsClientId=client)
    shell_shape = physics.createCollisionShape(
        physics.GEOM_CYLINDRICAL_SHELL,
        radius=0.006,
        height=0.030,
        axialOffset=0.0,
        closedBottom=True,
        closedTop=False,
        physicsClientId=client,
    )
    analytical_rod_shape = physics.createCollisionShape(
        physics.GEOM_CYLINDER,
        radius=0.00048,
        height=0.00090,
        analytical=True,
        physicsClientId=client,
    )
    shell = physics.createMultiBody(
        0.0, shell_shape, (10.0, 0.0, 0.0), physicsClientId=client)
    # Initialize the collision system before creating particles, matching the
    # workbench emitter path where each generated body is bound at runtime.
    physics.stepSimulation(physicsClientId=client)
    shell_particles = []
    for index in range(24):
        x = 10.0 + ((index % 4) - 1.5) * 0.0011
        y = (((index // 4) % 3) - 1.0) * 0.0011
        z = -0.011 + (index // 12) * 0.0012
        shell_particles.append(physics.createMultiBody(
            4.0e-7, analytical_rod_shape, (x, y, z),
            physicsClientId=client))
    for _step in range(1500):
        physics.stepSimulation(physicsClientId=client)
    for particle in shell_particles:
        position, _rotation = physics.getBasePositionAndOrientation(
            particle, physicsClientId=client)
        radial = math.hypot(position[0] - 10.0, position[1])
        assert all(math.isfinite(value) for value in position), position
        assert radial < 0.0065, position
        assert position[2] > -0.016, position
    shell_contacts = physics.getContactPoints(
        bodyA=shell, physicsClientId=client)
    assert shell_contacts, "Analytical cylindrical shell reported no contacts"

    print("Chrono rigid-body smoke test: PASS")
finally:
    physics.disconnect(client)
