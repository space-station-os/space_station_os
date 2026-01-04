from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton, QTextEdit, QLabel
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, pyqtSignal


class LeftPanel(QWidget):
    """
    Minimal panel: ONLY the AI Assist UI.
    - Transcript (read-only)
    - Input box + Ask button
    Colored, labeled lines:
      Astronaut:  <cyan>
      SSOS-AI:    <lime>
    """

    ask_ai = pyqtSignal(str)  # Emitted when the astronaut submits a question

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)

        header = QLabel("SSOS-NOVA")
        header.setFont(QFont("Arial", 12, QFont.Bold))
        header.setStyleSheet("color: white;")
        layout.addWidget(header)

        # Transcript (renders basic HTML)
        self.ai_output = QTextEdit()
        self.ai_output.setReadOnly(True)
        self.ai_output.setStyleSheet(
            "background:#0b0b0b; color:#ddd; border:1px solid #333; font-size:12px;"
        )
        self.ai_output.setFixedHeight(320)
        self.ai_output.setPlaceholderText("Ask a question below…")
        layout.addWidget(self.ai_output)

        # Input row
        row = QHBoxLayout()
        self.ai_input = QLineEdit()
        self.ai_input.setPlaceholderText("e.g., How much oxygen do I have?")
        self.ai_input.returnPressed.connect(self._emit_question)
        ask_btn = QPushButton("Ask")
        ask_btn.clicked.connect(self._emit_question)
        row.addWidget(self.ai_input)
        row.addWidget(ask_btn)
        layout.addLayout(row)

        layout.addStretch()
        self.setLayout(layout)

    # ---- Public slots ----
    def append_ai_response(self, text: str):
        """
        Called by the AI agent signal. We wrap it with label + color.
        """
        safe = self._escape(text)
        html = (
            "<div style='margin:6px 0;'>"
            "<span style='color:#9BEF00; font-weight:700;'>SSOS-AI:</span> "
            f"<span style='color:#d7ffd7;'>{safe}</span>"
            "</div>"
        )
        self.ai_output.append(html)

    def _emit_question(self):
        q = self.ai_input.text().strip()
        if not q:
            return

        safe = self._escape(q)
        html = (
            "<div style='margin:6px 0;'>"
            "<span style='color:#00E5FF; font-weight:700;'>Astronaut:</span> "
            f"<span style='color:#d0f6ff;'>{safe}</span>"
            "</div>"
        )
        self.ai_output.append(html)
        self.ai_output.moveCursor(self.ai_output.textCursor().End)  # auto-scroll
        self.ask_ai.emit(q)
        self.ai_input.clear()

    def _escape(self, s: str) -> str:
        # minimal HTML escape
        return (
            s.replace("&", "&amp;")
             .replace("<", "&lt;")
             .replace(">", "&gt;")
             .replace("\n", "<br>")
        )

   
    def update_co2(self, value): 
        pass

    def update_o2(self, percent): 
        pass

    def update_temp(self, temp): 
        pass

    def update_failure(self, msg, is_critical=False): 
        pass

    def update_goal_summary(self, summary): 
        pass
