"""Headless smoke test for the exact FreeCAD runtime used by the workbench."""

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
    print("Chrono rigid-body smoke test: PASS")
finally:
    physics.disconnect(client)
