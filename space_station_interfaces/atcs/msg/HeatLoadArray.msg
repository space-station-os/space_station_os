# All heat a subsystem dissipates into the thermal network. Each source
# publishes its complete set; the latest message per source replaces the last.
std_msgs/Header header
string source                # publishing subsystem
HeatLoad[] loads
