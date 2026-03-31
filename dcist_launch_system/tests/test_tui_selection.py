"""Tests for TUI SelectionList preset behavior.

Uses Textual's built-in test framework to verify that auto-populate
logic (m=mapping, r=relocalize) correctly selects the right options
in SelectionList widgets.

Run: pytest dcist_launch_system/tests/test_tui_selection.py -v
"""

import pytest

try:
    from textual.app import App, ComposeResult
    from textual.widgets import SelectionList, Label, Footer
    from textual.binding import Binding
    from textual.screen import ModalScreen
    from textual.containers import VerticalScroll

    HAS_TEXTUAL = True
except ImportError:
    HAS_TEXTUAL = False

pytestmark = pytest.mark.skipif(not HAS_TEXTUAL, reason="textual not installed")


# ---------------------------------------------------------------------------
# Minimal reproduction of SelectionList behavior
# ---------------------------------------------------------------------------


class SelectionTestApp(App):
    """Minimal app to test SelectionList initial selection."""

    BINDINGS = [Binding("t", "test_select", "Test")]

    def compose(self) -> ComposeResult:
        yield SelectionList(
            ("Alpha", "alpha", True),
            ("Beta", "beta", False),
            ("Gamma", "gamma", True),
            id="test_list",
        )

    def action_test_select(self):
        sel = self.query_one("#test_list", SelectionList)
        self._test_result = list(sel.selected)


@pytest.mark.asyncio
async def test_selection_list_initial_state():
    """Verify SelectionList constructor sets initial selections correctly."""
    app = SelectionTestApp()
    async with app.run_test() as pilot:
        sel = app.query_one("#test_list", SelectionList)
        selected = list(sel.selected)
        assert "alpha" in selected, f"alpha should be selected, got {selected}"
        assert "gamma" in selected, f"gamma should be selected, got {selected}"
        assert "beta" not in selected, f"beta should NOT be selected, got {selected}"


@pytest.mark.asyncio
async def test_selection_list_select_method():
    """Verify SelectionList.select() works after construction."""
    app = SelectionTestApp()
    async with app.run_test() as pilot:
        sel = app.query_one("#test_list", SelectionList)
        sel.select("beta")
        selected = list(sel.selected)
        assert "beta" in selected, f"beta should be selected after select(), got {selected}"


@pytest.mark.asyncio
async def test_selection_list_deselect_all():
    """Verify deselect_all + select works."""
    app = SelectionTestApp()
    async with app.run_test() as pilot:
        sel = app.query_one("#test_list", SelectionList)
        sel.deselect_all()
        selected = list(sel.selected)
        assert len(selected) == 0, f"should be empty after deselect_all, got {selected}"
        sel.select("beta")
        selected = list(sel.selected)
        assert selected == ["beta"], f"should have only beta, got {selected}"


# ---------------------------------------------------------------------------
# Test widget replacement pattern (used in _apply_preset)
# ---------------------------------------------------------------------------


class ReplaceTestApp(App):
    """Test replacing a SelectionList widget entirely."""

    BINDINGS = [Binding("r", "replace_list", "Replace")]

    def compose(self) -> ComposeResult:
        yield VerticalScroll(
            Label("Before:"),
            SelectionList(
                ("A", "a", False),
                ("B", "b", False),
                id="my_list",
            ),
            Label("After:"),
        )

    def action_replace_list(self):
        old = self.query_one("#my_list", SelectionList)
        parent = old.parent
        idx = list(parent.children).index(old)
        old.remove()
        new_sel = SelectionList(
            ("A", "a", True),
            ("B", "b", False),
            ("C", "c", True),
            id="my_list",
        )
        parent.mount(new_sel, before=idx)


@pytest.mark.asyncio
async def test_replace_selection_list():
    """Verify widget replacement preserves selections."""
    app = ReplaceTestApp()
    async with app.run_test() as pilot:
        # Initially nothing selected
        sel = app.query_one("#my_list", SelectionList)
        assert list(sel.selected) == []

        # Replace the widget
        await pilot.press("r")
        await pilot.pause()

        # Query the NEW widget
        sel = app.query_one("#my_list", SelectionList)
        selected = list(sel.selected)
        assert "a" in selected, f"a should be selected after replace, got {selected}"
        assert "c" in selected, f"c should be selected after replace, got {selected}"
        assert "b" not in selected, f"b should NOT be selected, got {selected}"


# ---------------------------------------------------------------------------
# Test that screen caching doesn't break widget queries
# ---------------------------------------------------------------------------


class CachedScreenApp(App):
    """Test pushing/dismissing/re-pushing a cached screen."""

    BINDINGS = [Binding("l", "show_screen", "Screen")]

    def __init__(self):
        super().__init__()
        self._cached_screen = None

    class TestScreen(ModalScreen):
        BINDINGS = [Binding("escape", "dismiss", "Back")]

        def compose(self) -> ComposeResult:
            yield SelectionList(
                ("One", "one", True),
                ("Two", "two", False),
                id="cached_list",
            )
            yield Footer()

    def compose(self) -> ComposeResult:
        yield Label("Main")

    def action_show_screen(self):
        if self._cached_screen is None:
            self._cached_screen = self.TestScreen()
        self.push_screen(self._cached_screen)


@pytest.mark.asyncio
async def test_cached_screen_preserves_selection():
    """Verify selections persist across dismiss/re-push of cached screen."""
    app = CachedScreenApp()
    async with app.run_test() as pilot:
        # Open screen
        await pilot.press("l")
        await pilot.pause()

        sel = app.query_one("#cached_list", SelectionList)
        assert "one" in list(sel.selected)

        # Select "two" as well
        sel.select("two")
        assert "two" in list(sel.selected)

        # Dismiss
        await pilot.press("escape")
        await pilot.pause()

        # Re-open — should still have both selected
        await pilot.press("l")
        await pilot.pause()

        sel = app.query_one("#cached_list", SelectionList)
        selected = list(sel.selected)
        assert "one" in selected, f"one should persist, got {selected}"
        assert "two" in selected, f"two should persist, got {selected}"


# ---------------------------------------------------------------------------
# Test generate_zenoh_endpoints scoping
# ---------------------------------------------------------------------------


def test_zenoh_endpoints_scoped(sample_topology):
    """Verify generate_zenoh_endpoints respects robot_filter."""
    from dcist_launch_system.fleet_helpers import generate_zenoh_endpoints

    sample_topology["zenoh"] = {"port": 7447}

    # Base station with no filter → gets all robots
    eps = generate_zenoh_endpoints(sample_topology, "mit_wifi", "gamma")
    assert len(eps) == 2  # alpha + beta

    # Base station with filter → only filtered robots
    eps = generate_zenoh_endpoints(
        sample_topology, "mit_wifi", "gamma", robot_filter=["alpha"]
    )
    assert len(eps) == 1
    assert "10.29.1.1" in eps[0]

    # Robot → always empty (listeners)
    eps = generate_zenoh_endpoints(sample_topology, "mit_wifi", "alpha")
    assert eps == []


def test_zenoh_endpoints_empty_filter(sample_topology):
    """Verify robot_filter=[] gives no endpoints."""
    from dcist_launch_system.fleet_helpers import generate_zenoh_endpoints

    sample_topology["zenoh"] = {"port": 7447}
    eps = generate_zenoh_endpoints(
        sample_topology, "mit_wifi", "gamma", robot_filter=[]
    )
    assert eps == []
