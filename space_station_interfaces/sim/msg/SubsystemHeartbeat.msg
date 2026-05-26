# Periodic health report from each subsystem
builtin_interfaces/Time stamp
string subsystem_name
uint8 LIFECYCLE_UNCONFIGURED=0
uint8 LIFECYCLE_INACTIVE=1
uint8 LIFECYCLE_ACTIVE=2
uint8 LIFECYCLE_FINALIZED=3
uint8 lifecycle_state
bool healthy
string status_message