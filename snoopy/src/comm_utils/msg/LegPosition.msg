string ENV_SIM = "sim"
string ENV_REAL = "real"
int32 LEFT_LEG_CONSTANT = 1
int32 RIGHT_LEG_CONSTANT = -1

std_msgs/Header header
string env
float64[12] foot_position # LF_x, LF_y, LF_z, RF_x, RF_y, RF_z, LH_x, LH_y, LH_z, RH_x, RH_y, RH_z