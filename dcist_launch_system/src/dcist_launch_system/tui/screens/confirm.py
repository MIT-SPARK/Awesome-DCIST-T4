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


class TypeConfirmScreen(ModalScreen[bool]):
    """Confirmation screen that requires typing a specific phrase to proceed.

    Used when the stakes are high enough that a casual key-press is insufficient
    (e.g. overwriting a directory larger than 1 GB).
    """

    BINDINGS = [
        Binding("escape", "confirm_no", "Cancel", priority=True),
    ]

    def __init__(self, message: str, required_text: str = "confirm"):
        super().__init__()
        self.message = message
        self._required_text = required_text

    def compose(self) -> ComposeResult:
        yield Vertical(
            Label(self.message),
            Rule(),
            Label(f'Type [bold]{self._required_text}[/bold] and press Enter to proceed:'),
            Input(placeholder=self._required_text, id="type_confirm_input"),
            Rule(),
            Horizontal(
                Button("Proceed", variant="error", id="btn_yes", disabled=True),
                Button("Cancel", variant="primary", id="btn_no"),
            ),
            id="type_confirm_dialog"
        )

    def on_input_changed(self, event: Input.Changed) -> None:
        btn = self.query_one("#btn_yes", Button)
        btn.disabled = event.value.strip().lower() != self._required_text.lower()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.value.strip().lower() == self._required_text.lower():
            self.dismiss(True)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(event.button.id == "btn_yes")

    def action_confirm_no(self) -> None:
        self.dismiss(False)

