# arctos_gui — Coding Rules

Rules for the `arctos_gui` ROS 2 Python package. All contributors and AI agents must follow these rules when adding or modifying code.

---

## 0. Package-level Backend

- A dedicated `backend/` package provides all non-ROS application state and persistence.
- Settings are stored as **YAML** in the package's config directory (`share/arctos_gui/config/settings.yaml`), resolved at runtime via `ament_index_python`. The source file lives at `config/settings.yaml` in the package root and is registered in `setup.py`.
- A single `SettingsManager` class owns all read/write access to the settings file. No component may access the settings file directly.
- `backend/` modules must **not** import `PyQt5` or `rclpy`. They are pure Python.
- Runtime state that needs to be shared across components lives in `app_state.py`.

```
arctos_gui/
└── backend/
    ├── __init__.py
    ├── settings_manager.py   # Load / save / get / set settings (YAML)
    └── app_state.py          # Optional: runtime state shared across components
```

---

## 1. Code Style

- Follow **PEP 8** strictly. Enforced via `ruff` or `flake8`. Maximum line length: **99 characters**.
- Add `from __future__ import annotations` at the top of every module.
- All public functions, methods, and classes must have **type hints**.
- All public classes and functions must have a **docstring** (Google style).
- No magic numbers. Replace literals with named constants.
- No `print()` in production code (see §8 Logging).

---

## 2. Frontend / Backend Separation

Three distinct layers exist. Each layer has a strict import boundary:

| Layer | Location | May import |
|---|---|---|
| **UI (frontend)** | `components/<feature>/<feature>_widget.py` | `PyQt5`, `ui/`, protocol interface |
| **Protocol interface** | `components/<feature>/<feature>_client_protocol.py` | stdlib only |
| **ROS client (backend)** | `components/<feature>/ros_<feature>_client.py` | `rclpy`, stdlib, protocol interface |

- A **widget** must never import `rclpy` or any ROS message type.
- A **ROS client** must never import `PyQt5`.
- The widget depends on the **protocol interface** (`typing.Protocol`), not on the concrete ROS client. This keeps widgets testable without a running ROS environment.

---

## 3. Directory & File Structure

Every feature (tab, panel, card) lives in its own sub-package under `components/`:

```
arctos_gui/
├── backend/
│   ├── __init__.py
│   ├── settings_manager.py
│   └── app_state.py
├── ui/
│   ├── __init__.py
│   ├── theme.py              # Color tokens, QSS, apply_app_theme()
│   └── widgets.py            # Reusable widget factories
├── main/
│   ├── __init__.py
│   └── main_window.py        # Assembles tabs — no business logic
├── components/
│   └── <feature>/
│       ├── __init__.py                     # Re-exports public symbols only
│       ├── <feature>_client_protocol.py    # Protocol interface — no ROS, no Qt
│       ├── <feature>_widget.py             # Qt widget (frontend)
│       └── ros_<feature>_client.py         # ROS 2 backend
└── arctos_gui_main.py                      # Entry point: ROS init, Qt app, shutdown
```

**Structural rules:**
- When a component is renamed or moved, its directory, all files inside it, and **all import references** must be updated atomically. No orphaned files or stale imports.
- `main_window.py` only assembles tabs — zero business logic.
- `arctos_gui_main.py` only wires ROS init, Qt app lifecycle, and shutdown — nothing else.
- Components must not import from each other. Cross-component communication goes through shared backend state or ROS topics.

---

## 4. Theme & Visual Consistency

- **Single source of truth:** all colors, border radii, spacings, and font sizes are defined as module-level constants in `ui/theme.py`.
- **No per-widget stylesheets.** Widgets use Qt dynamic properties (`variant`, `role`, `status`) that are resolved by the global QSS defined in `theme.py`.
- Never call `setStyleSheet()` on individual widgets.

### Buttons
- Always create buttons via `action_button(text, variant)` from `ui/widgets.py`. Never instantiate `QPushButton` directly in component code.
- Allowed `variant` values: `"primary"`, `"danger"`, `"success"`, `"warning"`, `None` (default/neutral).

### Labels
- Allowed `role` values: `"title"`, `"muted"`, `"accent"`, `"danger"`, `"warning"`, `"fieldLabel"`, `"statusDot"`, `"statusText"`.
- Set roles via `set_role(widget, role)` from `ui/theme.py`.

### Spacing & Radii
- Border radius: **8 px** for inputs and buttons, **10 px** for cards and tabs. Never hardcode a different value.
- Outer margins: **12 px**. Widget spacing: **8–12 px**.
- All spacing values must reference the constants in `theme.py`, not inline literals.

---

## 5. Widget Factories

- All reusable widget construction lives in `ui/widgets.py`.
- When a new widget type is needed in more than one component, add a factory function to `ui/widgets.py` — not inline in a component file.
- Factory functions must be **pure**: no side effects, no ROS calls, no global state mutation.

---

## 6. Tabs & Panels

- Each tab is a self-contained component sub-package (see §3).
- Tabs are registered in `main_window.py` only. Components must not know about each other.
- The main tab bar uses `QTabWidget` with `objectName="mainTabs"`.
- Sub-tabs within a component use `QTabWidget` with `objectName="subTabs"` for correct QSS targeting.

---

## 7. Testing

- Unit-test **backend** clients and protocol logic independently of Qt (no `QApplication` required).
- Widget tests use `pytest-qt`. Mock the protocol interface — never spin up a live ROS node in a widget test.
- Aim for **≥ 90 % coverage** on backend logic.
- Do not use arbitrary `time.sleep()` in tests. Use synchronization primitives or timeouts.

---

## 8. Logging

- Inside ROS clients: use the `rclpy` node logger (`self.get_logger()`).
- In `backend/` and other non-ROS helpers: use Python's `logging` module with a module-level logger (`logging.getLogger(__name__)`).
- Never use `print()` in production code.
- Avoid log spam on high-frequency paths; use throttled logging where appropriate.
