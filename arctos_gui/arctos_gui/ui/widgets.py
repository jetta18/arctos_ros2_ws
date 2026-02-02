"""Reusable UI widget factories.

Keep common widget configuration and layout behavior centralized so individual
tabs stay focused on behavior.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QResizeEvent
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QWidget,
)

from .theme import set_role, set_variant


_ACTION_MIN_HEIGHT_PX = 28


class _NoWheelMixin:
    def wheelEvent(self, event) -> None:  # noqa: N802 (Qt override)
        if not self.hasFocus():
            event.ignore()
            return
        super().wheelEvent(event)


class NoWheelComboBox(_NoWheelMixin, QComboBox):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)


class NoWheelSpinBox(_NoWheelMixin, QSpinBox):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)


class NoWheelDoubleSpinBox(_NoWheelMixin, QDoubleSpinBox):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)


def connect(signal: Any, slot: Callable[..., object]) -> None:
    """Connect a Qt signal to a slot.

    We intentionally avoid strict typing here because PyQt type stubs are
    commonly missing in ROS setups.
    """

    getattr(signal, "connect")(slot)  # type: ignore


def field_label(text: str) -> QLabel:
    """Create a label styled like a form field label."""

    label = QLabel(text)
    set_role(label, "fieldLabel")
    return label


def action_button(text: str, variant: Optional[str] = None) -> QPushButton:
    """Create a primary action button.

    The theme styles buttons with `role=action` and the optional `variant`.
    """

    button = QPushButton(text)
    set_role(button, "action")
    set_variant(button, variant)
    button.setMinimumHeight(_ACTION_MIN_HEIGHT_PX)
    button.setMinimumWidth(0)
    button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    return button


def combo_box(*, items: Optional[Sequence[str]] = None) -> QComboBox:
    box = NoWheelComboBox()
    if items:
        box.addItems(list(items))
    box.setFocusPolicy(Qt.StrongFocus)
    box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    return box


def scroll_container(content: QWidget) -> QScrollArea:
    area = QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QScrollArea.NoFrame)
    area.setWidget(content)
    return area


def double_spinbox(
    *,
    decimals: int,
    min_value: float,
    max_value: float,
    step: float,
    value: float,
    suffix: str = "",
) -> QDoubleSpinBox:
    """Create a consistently configured `QDoubleSpinBox`."""

    box = NoWheelDoubleSpinBox()
    box.setDecimals(decimals)
    box.setRange(min_value, max_value)
    box.setSingleStep(step)
    box.setValue(value)
    if suffix:
        box.setSuffix(suffix)

    box.setAccelerated(True)
    box.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
    box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    return box


def int_spinbox(
    *,
    min_value: int,
    max_value: int,
    step: int = 1,
    value: int = 0,
    suffix: str = "",
) -> QSpinBox:
    """Create a consistently configured `QSpinBox`."""

    box = NoWheelSpinBox()
    box.setRange(min_value, max_value)
    box.setSingleStep(step)
    box.setValue(value)
    if suffix:
        box.setSuffix(suffix)

    box.setAccelerated(True)
    box.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
    box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    return box


@dataclass(frozen=True)
class WrapGridConfig:
    min_cell_width_px: int = 320
    h_spacing_px: int = 10
    v_spacing_px: int = 10


class WrapGrid(QWidget):
    """A simple responsive grid that wraps children into new rows.

    Useful for action button rows: when the window is narrow, buttons stack
    vertically instead of being squeezed.
    """

    def __init__(
        self,
        widgets: Sequence[QWidget],
        config: WrapGridConfig = WrapGridConfig(),
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._widgets = list(widgets)
        self._config = config

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(config.h_spacing_px)
        self._grid.setVerticalSpacing(config.v_spacing_px)

        for w in self._widgets:
            w.setParent(self)

        self._relayout()

    def showEvent(self, a0) -> None:  # noqa: N802 (Qt override)
        super().showEvent(a0)
        self._relayout()

    def resizeEvent(self, a0: QResizeEvent) -> None:  # noqa: N802 (Qt override)
        super().resizeEvent(a0)
        self._relayout()

    def _relayout(self) -> None:
        while self._grid.count():
            self._grid.takeAt(0)

        cols = self._compute_columns()
        row = 0
        col = 0

        for w in self._widgets:
            self._grid.addWidget(w, row, col)
            col += 1
            if col >= cols:
                row += 1
                col = 0

        for c in range(cols):
            self._grid.setColumnStretch(c, 1)

        self.updateGeometry()

    def _compute_columns(self) -> int:
        spacing = max(0, self._grid.horizontalSpacing())
        widget_min = 0
        for w in self._widgets:
            widget_min = max(widget_min, int(w.minimumWidth()), int(w.minimumSizeHint().width()))

        cell = max(self._config.min_cell_width_px, widget_min)
        available = max(1, self.width())
        cols = max(1, (available + spacing) // (cell + spacing))
        return min(cols, max(1, len(self._widgets)))


def action_button_row(*buttons: QPushButton, min_cell_width_px: int = 60) -> WrapGrid:
    """Create a wrapping row/ grid for action buttons."""

    return WrapGrid(
        list(buttons),
        config=WrapGridConfig(min_cell_width_px=min_cell_width_px),
    )
