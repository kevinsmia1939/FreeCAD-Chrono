"""Small reusable Qt widgets for the Chrono workbench."""


def collapsible_section(title, layout_type, expanded=True):
    """Return a section widget and its content layout.

    The header uses a right-aligned arrow so dense task panels can be reduced
    without hiding the section title or its current purpose.
    """
    from PySide import QtCore, QtWidgets

    section = QtWidgets.QWidget()
    outer = QtWidgets.QVBoxLayout(section)
    outer.setContentsMargins(0, 0, 0, 0)
    outer.setSpacing(2)

    header = QtWidgets.QWidget()
    header_layout = QtWidgets.QHBoxLayout(header)
    header_layout.setContentsMargins(4, 2, 4, 2)
    header_layout.setSpacing(4)
    label = QtWidgets.QLabel(f"<b>{title}</b>")
    header_layout.addWidget(label)
    header_layout.addStretch()
    toggle = QtWidgets.QToolButton()
    toggle.setCheckable(True)
    toggle.setChecked(bool(expanded))
    toggle.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
    header_layout.addWidget(toggle)
    outer.addWidget(header)

    content = QtWidgets.QWidget()
    content_layout = layout_type(content)
    outer.addWidget(content)

    def update(checked):
        content.setVisible(checked)
        toggle.setArrowType(
            QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow)

    toggle.toggled.connect(update)
    toggle.clicked.connect(lambda: None)
    header.mousePressEvent = lambda _event: toggle.toggle()
    update(bool(expanded))
    return section, content_layout
