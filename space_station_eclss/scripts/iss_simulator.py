#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ROSParam
from rcl_interfaces.msg import ParameterValue, ParameterType
import time

class ISSParameterSimulator(Node):
    def __init__(self):
        super().__init__('iss_parameter_simulator')
        self.get_logger().info("🚀 ISS Parameter Simulator Initialized (System-wide Parameters)")
        self.target_nodes = [
            'adsorbent_bed_1',
            'adsorbent_bed_2',
            'air_collector',
            'desiccant_bed_1',
            'desiccant_bed_2'
        ]

    def set_parameter_on_node(self, node_name, param_name, param_value, param_type):
        client = self.create_client(SetParameters, f'/{node_name}/set_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"❌ {node_name} service unavailable.")
            return

        request = SetParameters.Request()
        ros_param = ROSParam()
        ros_param.name = param_name
        ros_param.value = ParameterValue(type=param_type)

        if param_type == ParameterType.PARAMETER_STRING:
            ros_param.value.string_value = param_value
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            ros_param.value.double_value = param_value
        elif param_type == ParameterType.PARAMETER_INTEGER:
            ros_param.value.integer_value = param_value

        request.parameters = [ros_param]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

    def update_all(self, param_set):
        for node in self.target_nodes:
            for name, (value, param_type) in param_set.items():
                self.set_parameter_on_node(node, name, value, param_type)

    def simulate_day_night_cycle(self):
        for cycle in range(6):
            self.get_logger().info(f"==================== 🌞 START OF ISS DAY {cycle + 1} ====================")

            day_params = {
                "mode_of_operation": ("exercise", ParameterType.PARAMETER_STRING),
                "crew_onboard": (6, ParameterType.PARAMETER_INTEGER),
                "cabin_pressure": (15.3, ParameterType.PARAMETER_DOUBLE),
                "temperature_cutoff": (470.0, ParameterType.PARAMETER_DOUBLE),
                "moisture_removal_rate": (0.30, ParameterType.PARAMETER_DOUBLE),
                "contaminant_removal_rate": (0.25, ParameterType.PARAMETER_DOUBLE),
                "humidification_rate": (1.7, ParameterType.PARAMETER_DOUBLE),
                "co2_adsorption_rate_constant": (0.02, ParameterType.PARAMETER_DOUBLE),
                "desired_temperature": (440.0, ParameterType.PARAMETER_DOUBLE),
                "co2_desorption_rate_constant": (0.06, ParameterType.PARAMETER_DOUBLE),
            }
            self.update_all(day_params)
            time.sleep(15)

            self.get_logger().info(f"==================== 🌙 START OF ISS NIGHT {cycle + 1} ====================")

            night_params = {
                "mode_of_operation": ("standby", ParameterType.PARAMETER_STRING),
                "crew_onboard": (3, ParameterType.PARAMETER_INTEGER),
                "cabin_pressure": (14.3, ParameterType.PARAMETER_DOUBLE),
                "temperature_cutoff": (440.0, ParameterType.PARAMETER_DOUBLE),
                "moisture_removal_rate": (0.15, ParameterType.PARAMETER_DOUBLE),
                "contaminant_removal_rate": (0.18, ParameterType.PARAMETER_DOUBLE),
                "humidification_rate": (1.3, ParameterType.PARAMETER_DOUBLE),
                "co2_adsorption_rate_constant": (0.012, ParameterType.PARAMETER_DOUBLE),
                "desired_temperature": (410.0, ParameterType.PARAMETER_DOUBLE),
                "co2_desorption_rate_constant": (0.045, ParameterType.PARAMETER_DOUBLE),
            }
            self.update_all(night_params)
            time.sleep(15)

def main():
    rclpy.init()
    node = ISSParameterSimulator()
    try:
        node.simulate_day_night_cycle()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
