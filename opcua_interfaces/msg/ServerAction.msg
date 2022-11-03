# AMR status requirement
string action_request
# AMR E-Stop status
bool estop
# AMR Requested Position
float64[3] goto_pos
# AMR Set Pause/Ready
bool pause_amr
bool ready_amr
#TM Set Pick/Place Status [Set one true]
bool pick_operation
bool place_operation
