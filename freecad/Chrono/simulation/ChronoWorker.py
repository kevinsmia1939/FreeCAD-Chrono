"""Persistent system-Python worker for FreeCAD/PyChrono ABI mismatches."""

import importlib.util
import json
import os
import sys
import traceback


def _load_native():
    path = os.path.join(os.path.dirname(__file__), "ChronoNative.py")
    spec = importlib.util.spec_from_file_location("freecad_chrono_native", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    native = _load_native()
    print(json.dumps({"ready": True}), flush=True)
except Exception:
    print(json.dumps({
        "ready": False,
        "error": traceback.format_exc(),
    }), flush=True)
    raise SystemExit(1)


for line in sys.stdin:
    try:
        request = json.loads(line)
        function = getattr(native, request["method"])
        result = function(
            *request.get("args", ()), **request.get("kwargs", {}))
        response = {"id": request.get("id"), "ok": True, "result": result}
    except Exception:
        response = {
            "id": request.get("id") if "request" in locals() else None,
            "ok": False,
            "error": traceback.format_exc(),
        }
    print(json.dumps(response), flush=True)
