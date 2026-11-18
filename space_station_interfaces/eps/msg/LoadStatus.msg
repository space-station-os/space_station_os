# Supply state of one load on the EPS secondary bus
string load_id
uint8 priority               # 0 = most critical
int32 ddcu_index             # DDCU carrying the load (-1 if unassigned)
float64 requested_power      # W
float64 served_power         # W actually delivered
bool shed                    # switched off by EPS
