# DC-to-DC converter unit: regulates primary power onto the secondary bus
std_msgs/Header header
string mode                  # off/regulating/overload/tripped
int32 mbsu_index             # MBSU feeding this DDCU
float64 input_voltage        # V
float64 output_voltage       # V
float64 output_power         # W delivered to loads
float64 input_power          # W drawn from the MBSU
float64 rated_power          # W
float64 heat_loss            # W
float64 temperature          # C
int32 active_loads
int32 shed_loads
bool fault
string fault_message
