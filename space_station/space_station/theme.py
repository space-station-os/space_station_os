# space_station/theme.py
"""
SSOS Mission-Control design system.

Everything visual is centralized here:
  * Color palettes (VAST / AXIOM / SSOS), selectable via ACTIVE_THEME.
  * build_stylesheet(theme) -> the full app QSS, generated from a palette dict.
  * Font loading (bundled TTFs under assets/fonts) with graceful fallback.
  * Small helpers (mono_font, label_font, apply_tracking) used by widgets.

Swap the whole console look by changing ACTIVE_THEME (or calling
apply_theme(app, AXIOM_THEME)). No panel file should hardcode a hex color;
pull from theme.color(name) / ACTIVE_THEME instead.
"""

from PyQt5.QtGui import QFontDatabase, QFont, QColor
from PyQt5.QtWidgets import QApplication

# ---------------------------------------------------------------------------
# Palettes
# ---------------------------------------------------------------------------

VAST_THEME = {
    "name":      "VAST",
    "bg":        "#0c0c0d",   # app background (near-black)
    "bg2":       "#0f0f10",   # bars, secondary surfaces
    "panel":     "#141415",   # cards/blocks
    "panel2":    "#1a1a1c",   # nested surfaces
    "line":      "#242426",   # hairline borders
    "line2":     "#303033",   # emphasis borders
    "txt":       "#f6f6f4",   # primary text (off-white)
    "txt2":      "#c2c2bd",   # secondary text (lightened for dark-bg legibility)
    "txt3":      "#94948f",   # tertiary / labels (lightened)
    "accent":    "#e8e6e1",   # near-white accent (chart lines, fills)
    "amber":     "#d98a2b",   # caution / desorbing / warning
    "red":       "#d65745",   # critical / fault
    "green":     "#7a9a6d",   # nominal / healthy
    "blue":      "#6d8aab",   # info (used sparingly)
}

# A cooler, slightly bluer monochrome variant. Same keys -> swappable.
AXIOM_THEME = {
    "name":      "AXIOM",
    "bg":        "#0a0d10",
    "bg2":       "#0d1116",
    "panel":     "#11161c",
    "panel2":    "#161c24",
    "line":      "#1f262e",
    "line2":     "#2b343e",
    "txt":       "#eef2f6",
    "txt2":      "#b6c0cb",
    "txt3":      "#8893a0",
    "accent":    "#dfe8f0",
    "amber":     "#d99a3b",
    "red":       "#d6604f",
    "green":     "#6fa288",
    "blue":      "#5f8fc0",
}

# Warmer legacy-SSOS variant (keeps the original lime as its accent).
SSOS_THEME = {
    "name":      "SSOS",
    "bg":        "#0b0b0c",
    "bg2":       "#101012",
    "panel":     "#16161a",
    "panel2":    "#1d1d22",
    "line":      "#2a2a30",
    "line2":     "#383842",
    "txt":       "#f2f3f5",
    "txt2":      "#bcbdc4",
    "txt3":      "#8c8d96",
    "accent":    "#9bef00",
    "amber":     "#e0a72c",
    "red":       "#e05545",
    "green":     "#5fc36a",
    "blue":      "#46b6d6",
}

# Active palette -- change this one line to restyle the whole app.
ACTIVE_THEME = VAST_THEME


def color(name: str) -> str:
    """Return a hex string from the active theme (safe fallback to txt)."""
    return ACTIVE_THEME.get(name, ACTIVE_THEME["txt"])


def qcolor(name: str) -> QColor:
    return QColor(ACTIVE_THEME.get(name, ACTIVE_THEME["txt"]))


# ---------------------------------------------------------------------------
# Fonts
# ---------------------------------------------------------------------------

# Resolved at load time. Fall back to Qt defaults if the TTFs aren't present.
FONT_SANS = "Sans Serif"
FONT_MONO = "Monospace"

_SANS_CANDIDATES = ["Inter", "DM Sans", "Inter Tight"]
_MONO_CANDIDATES = ["JetBrains Mono", "JetBrainsMono", "JetBrains Mono NL"]

_FONTS_LOADED = False


def load_fonts():
    """
    Load bundled TTFs from assets/fonts via QFontDatabase. Idempotent and
    crash-proof: if a font can't be found/loaded we keep the Qt default.
    Must be called after a QApplication exists.
    """
    global FONT_SANS, FONT_MONO, _FONTS_LOADED
    if _FONTS_LOADED:
        return
    _FONTS_LOADED = True

    loaded_families = set()
    try:
        from importlib.resources import files
        font_dir = files("space_station.assets.fonts")
        for entry in font_dir.iterdir():
            try:
                if str(entry).lower().endswith((".ttf", ".otf")):
                    fid = QFontDatabase.addApplicationFont(str(entry))
                    for fam in QFontDatabase.applicationFontFamilies(fid):
                        loaded_families.add(fam)
            except Exception:
                continue
    except Exception:
        # No bundled fonts (e.g. assets/fonts missing) -- fall back gracefully.
        pass

    available = set(QFontDatabase().families()) | loaded_families

    for cand in _SANS_CANDIDATES:
        if cand in available:
            FONT_SANS = cand
            break
    for cand in _MONO_CANDIDATES:
        if cand in available:
            FONT_MONO = cand
            break
    if FONT_MONO == "Monospace" and "Monospace" not in available:
        fixed = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        FONT_MONO = fixed.family()


def mono_font(size: int = 16, bold: bool = False) -> QFont:
    f = QFont(FONT_MONO, size)
    f.setStyleHint(QFont.Monospace)
    f.setBold(bold)
    return f


def label_font(size: int = 13, tracking: float = 1.5, bold: bool = False) -> QFont:
    """Uppercase wide-tracked label font (the signature Vast look)."""
    f = QFont(FONT_SANS, size)
    f.setBold(bold)
    if tracking:
        f.setLetterSpacing(QFont.AbsoluteSpacing, tracking)
    return f


def sans_font(size: int = 15, bold: bool = False) -> QFont:
    f = QFont(FONT_SANS, size)
    f.setBold(bold)
    return f


def apply_tracking(widget, tracking: float = 2.0):
    """Apply absolute letter-spacing to an existing widget's font."""
    f = widget.font()
    f.setLetterSpacing(QFont.AbsoluteSpacing, tracking)
    widget.setFont(f)


# ---------------------------------------------------------------------------
# QSS generation
# ---------------------------------------------------------------------------

def build_stylesheet(theme: dict) -> str:
    """Return the full application QSS string built from a palette dict."""
    t = theme
    return f"""
* {{
    font-family: "{FONT_SANS}";
    outline: none;
}}

QMainWindow, QDialog {{
    background-color: {t['bg']};
}}

QWidget {{
    background-color: {t['bg']};
    color: {t['txt']};
    font-size: 16px;
}}

/* ---- Cards / blocks ---- */
QFrame[class="card"], QGroupBox {{
    background-color: {t['panel']};
    border: 1px solid {t['line']};
    border-radius: 10px;
}}

QFrame[class="nested"] {{
    background-color: {t['panel2']};
    border: 1px solid {t['line']};
    border-radius: 8px;
}}

QFrame[class="flush"] {{
    background-color: {t['panel']};
    border: 1px solid {t['line']};
    border-radius: 10px;
}}

QFrame[class="hline"] {{
    background-color: {t['line']};
    border: none;
    max-height: 1px;
    min-height: 1px;
}}

QFrame[class="vline"] {{
    background-color: {t['line']};
    border: none;
    max-width: 1px;
    min-width: 1px;
}}

/* ---- GroupBox (legacy panels still use these) ---- */
QGroupBox {{
    margin-top: 16px;
    padding: 10px;
    font-size: 13px;
    color: {t['txt3']};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 4px;
    color: {t['txt3']};
}}

/* ---- Labels ---- */
QLabel {{
    background: transparent;
    color: {t['txt']};
}}
QLabel[class="label"] {{
    color: {t['txt3']};
    font-size: 13px;
}}
QLabel[class="metric-value"] {{
    color: {t['txt']};
    font-family: "{FONT_MONO}";
    font-size: 36px;
}}
QLabel[class="value"] {{
    color: {t['txt']};
    font-family: "{FONT_MONO}";
    font-size: 19px;
}}
QLabel[class="title"] {{
    color: {t['txt']};
    font-size: 34px;
}}
QLabel[class="subtitle"] {{
    color: {t['txt3']};
    font-size: 14px;
}}
QLabel[class="tag"] {{
    color: {t['txt2']};
    font-family: "{FONT_MONO}";
    font-size: 14px;
    border: 1px solid {t['line']};
    border-radius: 4px;
    padding: 3px 8px;
}}
QLabel[class="footer-green"] {{ color: {t['green']}; font-size: 14px; }}
QLabel[class="footer-amber"] {{ color: {t['amber']}; font-size: 14px; }}
QLabel[class="footer-red"]   {{ color: {t['red']};   font-size: 14px; }}
QLabel[class="footer-muted"] {{ color: {t['txt3']};  font-size: 14px; }}

/* ---- Buttons ---- */
QPushButton {{
    background-color: {t['panel2']};
    color: {t['txt2']};
    border: 1px solid {t['line']};
    border-radius: 6px;
    padding: 6px 12px;
    font-size: 15px;
}}
QPushButton:hover {{
    color: {t['txt']};
    border: 1px solid {t['line2']};
}}
QPushButton:pressed {{
    background-color: {t['panel']};
}}

/* nav tab buttons */
QPushButton[class="nav-tab"] {{
    background: transparent;
    border: none;
    border-bottom: 1px solid transparent;
    border-radius: 0px;
    color: {t['txt3']};
    padding: 8px 16px;
    font-size: 15px;
}}
QPushButton[class="nav-tab"]:hover {{
    color: {t['txt2']};
}}
QPushButton[class="nav-tab"][active="true"] {{
    color: {t['txt']};
    border-bottom: 1px solid {t['accent']};
}}

/* ---- Inputs ---- */
QLineEdit, QTextEdit, QComboBox {{
    background-color: {t['panel2']};
    color: {t['txt']};
    border: 1px solid {t['line']};
    border-radius: 6px;
    padding: 5px 8px;
    selection-background-color: {t['line2']};
}}
QLineEdit:focus, QTextEdit:focus, QComboBox:focus {{
    border: 1px solid {t['line2']};
}}
QComboBox QAbstractItemView {{
    background-color: {t['panel2']};
    color: {t['txt']};
    border: 1px solid {t['line']};
    selection-background-color: {t['line2']};
}}

/* ---- Lists ---- */
QListWidget {{
    background-color: {t['panel']};
    color: {t['txt']};
    border: 1px solid {t['line']};
    border-radius: 8px;
}}

/* ---- Tables ---- */
QTableWidget, QTableView {{
    background-color: {t['panel']};
    alternate-background-color: {t['panel2']};
    color: {t['txt']};
    gridline-color: {t['line']};
    border: 1px solid {t['line']};
    border-radius: 8px;
}}
QHeaderView::section {{
    background-color: {t['bg2']};
    color: {t['txt3']};
    border: none;
    border-bottom: 1px solid {t['line']};
    padding: 6px;
    font-size: 13px;
}}
QTableCornerButton::section {{
    background-color: {t['bg2']};
    border: none;
}}

/* ---- Progress bars ---- */
QProgressBar {{
    background-color: {t['panel2']};
    border: 1px solid {t['line']};
    border-radius: 6px;
    text-align: center;
    color: {t['txt2']};
    font-size: 14px;
}}
QProgressBar::chunk {{
    background-color: {t['accent']};
    border-radius: 5px;
}}

/* ---- Scroll areas / bars ---- */
QScrollArea {{
    background: transparent;
    border: none;
}}
QScrollBar:vertical {{
    background: {t['bg2']};
    width: 8px;
    margin: 0px;
}}
QScrollBar::handle:vertical {{
    background: {t['line2']};
    border-radius: 4px;
    min-height: 24px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QScrollBar:horizontal {{
    background: {t['bg2']};
    height: 8px;
}}
QScrollBar::handle:horizontal {{
    background: {t['line2']};
    border-radius: 4px;
    min-width: 24px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0px;
}}

/* ---- Tooltips ---- */
QToolTip {{
    background-color: {t['panel2']};
    color: {t['txt']};
    border: 1px solid {t['line2']};
    padding: 4px;
}}

/* ---- Tab widget (legacy) ---- */
QTabWidget::pane {{
    border: 1px solid {t['line']};
    border-radius: 8px;
}}
QTabBar::tab {{
    background: transparent;
    color: {t['txt3']};
    padding: 8px 16px;
    border: none;
    font-size: 14px;
}}
QTabBar::tab:selected {{
    color: {t['txt']};
    border-bottom: 1px solid {t['accent']};
}}
"""


def apply_theme(app: QApplication, theme: dict = None):
    """Load fonts (once) and apply the generated stylesheet to the app."""
    global ACTIVE_THEME
    if theme is not None:
        ACTIVE_THEME = theme
    load_fonts()
    app.setStyle("Fusion")
    app.setStyleSheet(build_stylesheet(ACTIVE_THEME))


# ---------------------------------------------------------------------------
# Back-compat shims (legacy callers expect these names)
# ---------------------------------------------------------------------------

def load_dark_theme(app: QApplication):
    """Legacy entry point -- now applies the active mission-control theme."""
    apply_theme(app, ACTIVE_THEME)


def load_light_theme(app: QApplication):
    """Light mode is not part of the mission-control aesthetic; keep dark."""
    apply_theme(app, ACTIVE_THEME)
