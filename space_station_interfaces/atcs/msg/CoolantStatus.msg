# Coolant-loop monitoring status, published periodically by ssos_thermal's
# coolant_node on /thermal/coolant/status. Read-only telemetry: subscribing
# has no side effects, unlike sending a /coolant_heat_transfer goal, which
# actuates the loop.
std_msgs/Header header
bool active                 # true while a cooling goal is executing
string component_id         # component of the goal in flight (empty when idle)
float64 internal_temp_c     # internal loop temperature [degC]
float64 ammonia_temp_c      # ammonia loop temperature [degC]
float64 vented_heat_kj      # heat vented during the current or last cycle [kJ]
