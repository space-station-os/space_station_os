# Global SSOS system state
uint8 INIT=0
uint8 NOMINAL=1
uint8 DEGRADED=2
uint8 RECOVERY=3
uint8 SAFE=4

builtin_interfaces/Time stamp
uint8 state
string triggering_event
string[] degraded_subsystems