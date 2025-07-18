from PyQt5.QtWidgets import QDialog, QLabel, QSpinBox, QDoubleSpinBox, QComboBox, QFormLayout, QDialogButtonBox
from rclpy.parameter import Parameter

class SubsystemParamDialog(QDialog):
    def __init__(self, subsystem, params_dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"{subsystem} Parameters")
        self.subsystem = subsystem
        self.layout = QFormLayout(self)
        self.inputs = {}

        for name, default in params_dict.items():
            if isinstance(default, bool):
                box = QSpinBox()
                box.setRange(0, 1)
                box.setValue(1 if default else 0)
            elif isinstance(default, int):
                box = QSpinBox()
                box.setMaximum(int(1e6))
                box.setValue(default)
            elif isinstance(default, float):
                box = QDoubleSpinBox()
                box.setDecimals(4)
                box.setMaximum(1e6)
                box.setValue(default)
            else:
                box = QComboBox()
                box.addItem(str(default))
            self.inputs[name] = box
            self.layout.addRow(QLabel(name), box)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        self.layout.addRow(buttons)

    def get_parameters(self):
        result = []
        for name, widget in self.inputs.items():
            if isinstance(widget, QSpinBox):
                value = widget.value()
                param_type = Parameter.Type.INTEGER
            elif isinstance(widget, QDoubleSpinBox):
                value = widget.value()
                param_type = Parameter.Type.DOUBLE
            elif isinstance(widget, QComboBox):
                value = widget.currentText()
                param_type = Parameter.Type.STRING
            else:
                continue
            result.append(Parameter(name, param_type, value))
        return result
