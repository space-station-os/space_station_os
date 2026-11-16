# Main bus switching unit: routes primary power from its channels to its DDCUs
std_msgs/Header header
float64 bus_voltage          # V (mean of the available source channels)
int32[] source_channels      # power channels wired to this MBSU
bool[] source_available      # availability of each source channel
int32 available_sources
float64 demand_power         # W requested by its DDCUs
float64 supplied_power       # W delivered from the source channels
bool unstable                # no usable source or bus voltage outside limits
bool fault
string fault_message
