"""Confirmation modal screen."""
from __future__ import annotations

import shlex
import threading

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import (
    Button,
    DataTable,
    Footer,
    Input,
    Label,
    ProgressBar,
    RichLog,
    Rule,
    SelectionList,
    Static,
    Tree,
)

class ConfirmScreen(ModalScreen[bool]):
    BINDINGS = [
        Binding("y", "confirm_yes", "Yes", priority=True),
        Binding("n", "confirm_no", "No", priority=True),
        Binding("escape", "confirm_no", "Cancel", priority=True),
    ]

    def __init__(self, message: str, yes_label: str = "Yes, Proceed (y)", no_label: str = "No, Cancel (n)"):
        super().__init__()
        self.message = message
        self._yes_label = yes_label
        self._no_label = no_label

    def compose(self) -> ComposeResult:
        yield Vertical(
            Label(self.message),
            Rule(),
            Label("[dim]y=yes  n/escape=no[/]"),
            Horizontal(
                Button(self._yes_label, variant="error", id="btn_yes"),
                Button(self._no_label, variant="primary", id="btn_no"),
            ),
            id="confirm_kill_dialog"
        )

    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(event.button.id == "btn_yes")

    def action_confirm_yes(self):
        self.dismiss(True)

    def action_confirm_no(self):
        self.dismiss(False)

