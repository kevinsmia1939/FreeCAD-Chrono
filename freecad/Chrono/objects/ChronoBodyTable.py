import FreeCAD


class ChronoBodyTableFeature:
    def __init__(self, obj):
        obj.Proxy = self

    def execute(self, obj):
        pass

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


# ---------------------------------------------------------------------------
# Task panel
# ---------------------------------------------------------------------------

class BodyTablePanel:
    """Table of all rigid bodies with editable cells and bulk-apply input row."""

    COL_NAME    = 0
    COL_TYPE    = 1
    COL_DENSITY = 2
    COL_MESH    = 3

    HEADERS = [
        "Name",
        "Type",
        "Density (kg/m³)",
        "Collision Shape",
    ]

    _SHAPE_OPTIONS = ["Auto", "box", "sphere", "cylinder", "convex_hull", "mesh"]

    def __init__(self):
        from PySide import QtCore, QtWidgets

        self._rb_list  = []
        self._updating = False

        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle("Rigid Body Summary")
        self.form.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding)
        layout = QtWidgets.QVBoxLayout(self.form)

        # ── Table ──────────────────────────────────────────────────────────
        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(len(self.HEADERS))
        self.table.setHorizontalHeaderLabels(self.HEADERS)
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        layout.addWidget(self.table, 1)

        # ── Refresh ────────────────────────────────────────────────────────
        refresh_btn = QtWidgets.QPushButton("Refresh")
        try:
            refresh_btn.setIcon(
                self.form.style().standardIcon(
                    QtWidgets.QStyle.SP_BrowserReload))
        except Exception:
            pass
        layout.addWidget(refresh_btn)

        # ── Bulk-apply section ─────────────────────────────────────────────
        apply_group = QtWidgets.QGroupBox("Apply to selected rows")
        grid = QtWidgets.QGridLayout(apply_group)
        grid.setColumnStretch(1, 1)

        # Type
        grid.addWidget(QtWidgets.QLabel("Type:"), 0, 0)
        self.inp_type = QtWidgets.QComboBox()
        self.inp_type.addItems(["", "Dynamic", "Fixed"])
        self.inp_type.setToolTip("Select Dynamic or Fixed, then click Apply")
        grid.addWidget(self.inp_type, 0, 1)
        self.btn_apply_type = QtWidgets.QPushButton("Apply")
        grid.addWidget(self.btn_apply_type, 0, 2)

        # Density
        grid.addWidget(QtWidgets.QLabel("Density (kg/m³):"), 1, 0)
        self.inp_density = QtWidgets.QLineEdit()
        self.inp_density.setPlaceholderText("e.g. 7850  (steel)")
        grid.addWidget(self.inp_density, 1, 1)
        self.btn_apply_density = QtWidgets.QPushButton("Apply")
        grid.addWidget(self.btn_apply_density, 1, 2)

        # Collision Shape override
        grid.addWidget(QtWidgets.QLabel("Collision Shape:"), 2, 0)
        self.inp_shape = QtWidgets.QComboBox()
        self.inp_shape.addItems([""] + self._SHAPE_OPTIONS)
        self.inp_shape.setToolTip(
            "Override the collision shape type for the selected rows.\n"
            "'Auto' restores automatic detection from geometry.")
        grid.addWidget(self.inp_shape, 2, 1)
        self.btn_apply_shape = QtWidgets.QPushButton("Apply")
        grid.addWidget(self.btn_apply_shape, 2, 2)

        layout.addWidget(apply_group)

        # ── Wiring ─────────────────────────────────────────────────────────
        refresh_btn.clicked.connect(self._populate)
        self.table.itemChanged.connect(self._on_item_changed)
        self.btn_apply_type.clicked.connect(self._apply_type)
        self.btn_apply_density.clicked.connect(self._apply_density)
        self.btn_apply_shape.clicked.connect(self._apply_shape)

        self._populate()

    # ── Populate ────────────────────────────────────────────────────────────

    def _populate(self):
        from PySide import QtCore, QtWidgets

        from ..simulation.ChronoSimulation import (
            collect_rigid_bodies, _detect_freecad_shape_type)
        self._updating = True
        try:
            # Show disabled bodies too so they can be re-enabled in the
            # Property editor without disappearing from this management table.
            self._rb_list = collect_rigid_bodies(enabled_only=False)
            self.table.setRowCount(len(self._rb_list))

            RO = QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable
            ED = RO | QtCore.Qt.ItemIsEditable

            for row, rb in enumerate(self._rb_list):
                try:
                    detected = _detect_freecad_shape_type(rb.OriginalObject.Shape)
                except Exception:
                    detected = "mesh"

                override = getattr(rb, "ShapeOverride", "Auto")
                effective = detected if override == "Auto" else override

                density = getattr(rb, "Density", 1000.0)

                def _ro(text):
                    it = QtWidgets.QTableWidgetItem(str(text))
                    it.setFlags(RO)
                    return it

                def _ed(text):
                    it = QtWidgets.QTableWidgetItem(str(text))
                    it.setFlags(ED)
                    return it

                self.table.setItem(row, self.COL_NAME,    _ro(rb.OriginalObject.Label))
                self.table.setItem(row, self.COL_TYPE,    _ro(rb.BodyType))
                self.table.setItem(row, self.COL_DENSITY, _ed(f"{density:.2f}"))

                # Collision shape combo — one per row
                # Each item stores the ShapeOverride value as item data so the
                # "Auto" entry can display the detected type without ambiguity.
                combo = QtWidgets.QComboBox()
                combo.addItem(f"{detected} (auto)", "Auto")
                for opt in self._SHAPE_OPTIONS[1:]:   # skip "Auto"
                    combo.addItem(opt, opt)
                # Select by stored data value
                target = override if override in self._SHAPE_OPTIONS else "Auto"
                for i in range(combo.count()):
                    if combo.itemData(i) == target:
                        combo.setCurrentIndex(i)
                        break
                combo.setToolTip(f"Auto-detected geometry type: {detected}")
                combo.currentIndexChanged.connect(
                    lambda _idx, r=row: self._on_shape_combo_changed(r))
                self.table.setCellWidget(row, self.COL_MESH, combo)

            self.table.resizeColumnsToContents()
        finally:
            self._updating = False

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _effective_mesh_type(self, row):
        """Effective collision shape type for a row (respects override)."""
        combo = self.table.cellWidget(row, self.COL_MESH)
        if combo is None:
            return "mesh"
        actual = combo.currentData()
        if actual and actual != "Auto":
            return actual
        if row >= len(self._rb_list):
            return "mesh"
        try:
            from ..simulation.ChronoSimulation import _detect_freecad_shape_type
            return _detect_freecad_shape_type(self._rb_list[row].OriginalObject.Shape)
        except Exception:
            return "mesh"

    # ── Selection ───────────────────────────────────────────────────────────

    def _selected_rows(self):
        return sorted(set(idx.row() for idx in self.table.selectedIndexes()))

    def _recompute_document(self):
        if self._rb_list and self._rb_list[0].Document is not None:
            self._rb_list[0].Document.recompute()

    # ── Inline cell / combo edits ────────────────────────────────────────────

    def _on_item_changed(self, item):
        if self._updating:
            return
        row = item.row()
        col = item.column()
        if row >= len(self._rb_list):
            return
        rb = self._rb_list[row]
        try:
            if col == self.COL_DENSITY:
                rb.Density = max(0.0, float(item.text()))
        except ValueError:
            pass
        else:
            self._recompute_document()

    def _on_shape_combo_changed(self, row):
        if self._updating or row >= len(self._rb_list):
            return
        combo = self.table.cellWidget(row, self.COL_MESH)
        actual = combo.currentData() if combo else "Auto"
        rb = self._rb_list[row]
        try:
            rb.ShapeOverride = actual
        except Exception:
            pass
        self._recompute_document()

    # ── Bulk apply ──────────────────────────────────────────────────────────

    def _apply_type(self):
        val = self.inp_type.currentText()
        if not val:
            return
        self._updating = True
        try:
            for row in self._selected_rows():
                rb = self._rb_list[row]
                rb.BodyType = val
                it = self.table.item(row, self.COL_TYPE)
                if it:
                    it.setText(val)
        finally:
            self._updating = False
        self._recompute_document()

    def _apply_density(self):
        try:
            val = max(0.0, float(self.inp_density.text()))
        except ValueError:
            return
        self._updating = True
        try:
            for row in self._selected_rows():
                rb = self._rb_list[row]
                rb.Density = val
                it = self.table.item(row, self.COL_DENSITY)
                if it:
                    it.setText(f"{val:.2f}")
        finally:
            self._updating = False
        self._recompute_document()

    def _apply_shape(self):
        val = self.inp_shape.currentText()
        if not val:
            return
        self._updating = True
        try:
            for row in self._selected_rows():
                rb = self._rb_list[row]
                try:
                    rb.ShapeOverride = val
                except Exception:
                    pass
                combo = self.table.cellWidget(row, self.COL_MESH)
                if combo:
                    for i in range(combo.count()):
                        if combo.itemData(i) == val:
                            combo.setCurrentIndex(i)
                            break
        finally:
            self._updating = False
        self._recompute_document()

    def needsFullSpace(self):
        return True

    def reject(self):
        import FreeCADGui
        FreeCADGui.Control.closeDialog()


# ---------------------------------------------------------------------------
# View provider
# ---------------------------------------------------------------------------

class ChronoBodyTableViewProvider:
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
        return os.path.join(ChronoUtils.ICONS_PATH, "ChronoBodyTable.svg")

    def setEdit(self, vobj, mode):
        import FreeCADGui
        if FreeCADGui.Control.activeDialog():
            FreeCADGui.Control.closeDialog()
        FreeCADGui.Control.showDialog(BodyTablePanel())
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


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_body_table(doc=None):
    if doc is None:
        doc = FreeCAD.ActiveDocument
    obj = doc.addObject("App::FeaturePython", "ChronoBodyTable")
    ChronoBodyTableFeature(obj)
    obj.Label = "Rigid Body Summary"
    if FreeCAD.GuiUp:
        import FreeCADGui
        ChronoBodyTableViewProvider(obj.ViewObject)
    return obj
