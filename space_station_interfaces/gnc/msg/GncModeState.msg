# Current actuation mode of the GNC subsystem.
#
# Momentum unloading is reported as a flag rather than a mode: it runs
# concurrently with CMG control, dumping momentum through the thrusters while
# the CMGs retain the projected attitude torque.

builtin_interfaces/Time stamp

uint8 MODE_CMG=0
uint8 MODE_THRUSTER=1
uint8 mode

# Seconds spent in the current mode, and how many transitions have occurred.
float64 time_in_mode
int32 transition_count

# True when the current mode was forced by a fault rather than requested.
bool forced_by_fault

# True while CMG momentum unloading is active.
bool unload_active

# Array momentum as a fraction of the saturation envelope. Telemetry only.
float64 cmg_momentum_fraction

# sqrt(det(A A^T)) for the CMG array. Approaches zero near a steering
# singularity. Telemetry only in this iteration.
float64 manipulability
