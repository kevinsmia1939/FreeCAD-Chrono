"""Select a direct or bridged PyChrono backend.

FreeCAD AppImages can embed a different CPython ABI than the host openSUSE
installation.  When PyChrono imports directly, calls stay in-process.  When it
does not, this module keeps one system-Python worker alive for the duration of
the simulation.  The protocol is local JSON over pipes; no network service or
temporary simulation data is involved.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import atexit


DIRECT = 0
GEOM_SPHERE = 2
GEOM_BOX = 3
GEOM_CYLINDER = 4
GEOM_MESH = 5
GEOM_COMPOUND = 6
GEOM_CONCAVE_INTERNAL_EDGE = 1


try:
    from . import ChronoNative as _native
    _native_error = ""
except (ImportError, OSError) as exc:
    _native = None
    _native_error = str(exc)


class _RemoteClient:
    def __init__(self, process, remote_id, executable):
        self.process = process
        self.remote_id = remote_id
        self.executable = executable
        self.request_id = 0

    def call(self, method, args=(), kwargs=None):
        if kwargs is None:
            kwargs = {}
        self.request_id += 1
        request = {
            "id": self.request_id,
            "method": method,
            "args": list(args),
            "kwargs": kwargs,
        }
        try:
            self.process.stdin.write(json.dumps(request) + "\n")
            self.process.stdin.flush()
            response_line = self.process.stdout.readline()
        except (BrokenPipeError, OSError) as exc:
            raise RuntimeError(f"PyChrono worker communication failed: {exc}") from exc
        if not response_line:
            details = self.process.stderr.read().strip()
            raise RuntimeError(
                "PyChrono worker stopped unexpectedly"
                + (f": {details}" if details else ""))
        response = json.loads(response_line)
        if not response.get("ok"):
            raise RuntimeError(response.get("error", "Unknown PyChrono worker error"))
        return response.get("result")

    def close(self):
        try:
            if self.process.stdin:
                self.process.stdin.close()
            self.process.wait(timeout=5)
        except Exception:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except Exception:
                self.process.kill()


_remote_clients = {}
_next_remote_client = 1


def _python_candidates():
    configured = os.environ.get("FREECAD_CHRONO_PYTHON", "").strip()
    names = [configured, "python3.13", "/usr/bin/python3.13", "python3"]
    seen = set()
    for name in names:
        if not name:
            continue
        resolved = shutil.which(name) if not os.path.isabs(name) else name
        if resolved and resolved not in seen and os.path.isfile(resolved):
            seen.add(resolved)
            yield resolved


def _start_worker():
    worker = os.path.join(os.path.dirname(__file__), "ChronoWorker.py")
    errors = []
    for executable in _python_candidates():
        environment = os.environ.copy()
        # The AppImage exports PYTHONHOME for its embedded Python 3.11.  A host
        # Python 3.13 subprocess must use its own standard library instead.
        environment.pop("PYTHONHOME", None)
        process = subprocess.Popen(
            [executable, "-u", worker],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=environment,
        )
        ready_line = process.stdout.readline()
        if ready_line:
            try:
                ready = json.loads(ready_line)
            except json.JSONDecodeError:
                ready = {}
            if ready.get("ready"):
                return process, executable
            errors.append(f"{executable}: {ready.get('error', ready_line.strip())}")
        else:
            errors.append(f"{executable}: {process.stderr.read().strip()}")
        process.wait()
    detail = "; ".join(errors) or "no suitable system Python was found"
    raise ImportError(
        "PyChrono cannot load in FreeCAD's Python ABI and the helper could not "
        f"start ({detail}). Direct import error: {_native_error}")


def backend_description():
    if _native is not None:
        return "direct PyChrono"
    if _remote_clients:
        client = next(iter(_remote_clients.values()))
        return f"PyChrono bridge via {client.executable}"
    return "system-Python PyChrono bridge"


def connect(mode=DIRECT, cpuParallel=True):
    global _next_remote_client
    if _native is not None:
        return _native.connect(mode, cpuParallel=cpuParallel)
    process, executable = _start_worker()
    temporary = _RemoteClient(process, None, executable)
    try:
        remote_id = temporary.call(
            "connect", (mode,), {"cpuParallel": bool(cpuParallel)})
    except Exception:
        temporary.close()
        raise
    client_id = _next_remote_client
    _next_remote_client += 1
    temporary.remote_id = remote_id
    _remote_clients[client_id] = temporary
    return client_id


def _call(method, physics_client_id, args=(), kwargs=None):
    if kwargs is None:
        kwargs = {}
    if _native is not None:
        kwargs["physicsClientId"] = physics_client_id
        return getattr(_native, method)(*args, **kwargs)
    try:
        client = _remote_clients[physics_client_id]
    except KeyError as exc:
        raise ValueError(f"Unknown bridged Chrono client {physics_client_id}") from exc
    kwargs["physicsClientId"] = client.remote_id
    return client.call(method, args, kwargs)


def disconnect(physicsClientId=0):
    if _native is not None:
        return _native.disconnect(physicsClientId)
    client = _remote_clients.pop(physicsClientId, None)
    if client is not None:
        try:
            client.call("disconnect", kwargs={
                "physicsClientId": client.remote_id})
        finally:
            client.close()


def setGravity(x, y, z, physicsClientId=0):
    return _call("setGravity", physicsClientId, (x, y, z))


def setTimeStep(timeStep, physicsClientId=0):
    return _call("setTimeStep", physicsClientId, (timeStep,))


def setPhysicsEngineParameter(physicsClientId=0, **parameters):
    return _call("setPhysicsEngineParameter", physicsClientId, kwargs=parameters)


def createCollisionShape(shapeType, physicsClientId=0, **parameters):
    return _call(
        "createCollisionShape", physicsClientId, (shapeType,), parameters)


def createMultiBody(baseMass, baseCollisionShapeIndex, basePosition,
                    baseOrientation=(0.0, 0.0, 0.0, 1.0), physicsClientId=0):
    return _call(
        "createMultiBody", physicsClientId,
        (baseMass, baseCollisionShapeIndex, basePosition, baseOrientation))


def changeDynamics(bodyUniqueId, linkIndex, physicsClientId=0, **parameters):
    return _call(
        "changeDynamics", physicsClientId,
        (bodyUniqueId, linkIndex), parameters)


def resetBaseVelocity(bodyUniqueId, linearVelocity=None, angularVelocity=None,
                      physicsClientId=0):
    return _call(
        "resetBaseVelocity", physicsClientId, (bodyUniqueId,), {
            "linearVelocity": linearVelocity,
            "angularVelocity": angularVelocity,
        })


def getBaseVelocity(bodyUniqueId, physicsClientId=0):
    result = _call("getBaseVelocity", physicsClientId, (bodyUniqueId,))
    return tuple(result[0]), tuple(result[1])


def getBasePositionAndOrientation(bodyUniqueId, physicsClientId=0):
    result = _call(
        "getBasePositionAndOrientation", physicsClientId, (bodyUniqueId,))
    return tuple(result[0]), tuple(result[1])


def setCollisionFilterGroupMask(bodyUniqueId, linkIndex, collisionFilterGroup,
                                collisionFilterMask, physicsClientId=0):
    return _call(
        "setCollisionFilterGroupMask", physicsClientId,
        (bodyUniqueId, linkIndex, collisionFilterGroup, collisionFilterMask))


def getContactPoints(bodyA=None, bodyB=None, physicsClientId=0):
    result = _call(
        "getContactPoints", physicsClientId, kwargs={
            "bodyA": bodyA, "bodyB": bodyB})
    return [tuple(contact) for contact in result]


def getClosestPoints(bodyA, bodyB, distance=0.0, physicsClientId=0):
    result = _call(
        "getClosestPoints", physicsClientId, (bodyA, bodyB), {
            "distance": distance})
    return [tuple(contact) for contact in result]


def removeBody(bodyUniqueId, physicsClientId=0):
    return _call("removeBody", physicsClientId, (bodyUniqueId,))


def stepSimulation(physicsClientId=0):
    return _call("stepSimulation", physicsClientId)


def _cleanup_remote_clients():
    for client_id in list(_remote_clients):
        try:
            disconnect(client_id)
        except Exception:
            pass


atexit.register(_cleanup_remote_clients)
