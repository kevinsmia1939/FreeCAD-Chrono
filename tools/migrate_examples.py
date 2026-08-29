"""Convert Bullet workbench FCStd examples to Chrono proxy references."""

from pathlib import Path
import os
import tempfile
import zipfile


SOURCE = Path(
    "/home/kevin/.local/share/FreeCAD/v26-3/Mod/FreeCAD-BulletPhysics/example")
TARGET = Path(__file__).resolve().parents[1] / "example"

REPLACEMENTS = (
    (b'module="objects.', b'module="freecad.Chrono.objects.'),
    (b"freecad.BulletPhysics", b"freecad.Chrono"),
    (b"BulletPhysics", b"Chrono"),
    (b"Bullet", b"Chrono"),
    (b"_Bt", b"_Ch"),
)


def migrate(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.stem + ".", suffix=".FCStd", dir=target.parent)
    os.close(descriptor)
    try:
        with zipfile.ZipFile(source, "r") as source_zip:
            with zipfile.ZipFile(temporary_name, "w", allowZip64=True) as target_zip:
                target_zip.comment = source_zip.comment or b"FreeCAD Document"
                for info in source_zip.infolist():
                    data = source_zip.read(info.filename)
                    if info.filename.endswith((".xml", ".txt")):
                        for old, new in REPLACEMENTS:
                            data = data.replace(old, new)
                    target_zip.writestr(info, data)
        os.replace(temporary_name, target)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)


for source_file in sorted(SOURCE.glob("*.FCStd")):
    migrate(source_file, TARGET / source_file.name)
    print(f"Migrated {source_file.name}")
