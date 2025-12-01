# Operating mode and key telemetry from the BCDU
std_msgs/Header header
string mode                  # idle/charging/discharging/fault/safe
float64 bus_voltage          # V
float64 regulation_voltage   # V (BCDU target/regulation setpoint)
float64 current_draw         # A (positive = discharge to bus, negative = charging from bus)
bool fault
string fault_message
