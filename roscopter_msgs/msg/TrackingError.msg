# How far the vehicle actually is from the trajectory it was told to fly.
#
# This is the quantity the collision-freedom argument rests on. The planner guarantees the
# reference trajectory is clear of obstacles by `safety_margin` metres (the drone radius the
# SFCs are inflated by), so the VEHICLE is only actually safe while its tracking error stays
# below that margin. Nothing measured it before, which left a gap in the middle of the
# argument: a corridor can be provably clear while the aircraft is not inside it.

std_msgs/Header header

# Commanded minus actual, in NED metres.
float64[3] position_error
float64 position_error_norm

# Commanded minus actual, in NED m/s.
float64[3] velocity_error
float64 velocity_error_norm

# Obstacle clearance the planner built into the corridors (metres). Tracking error must stay
# below this for the "following the trajectory is safe" guarantee to hold.
float64 safety_margin

# position_error_norm > safety_margin -- the vehicle is outside the volume the planner
# proved clear, even though the reference trajectory itself is fine.
bool margin_exceeded

# Largest position_error_norm seen since the node started. Worth watching more than the
# instantaneous value: the guarantee is broken by the worst excursion, not the average.
float64 peak_position_error

# --- Is this CONTROL error or ESTIMATOR error? -------------------------------------------
#
# Everything above is computed against `estimated_state` -- the same signal the controller
# regulates. That makes it blind to exactly one failure mode, and an important one: if the
# state estimate is biased, the controller drives the ESTIMATED error to zero while the true
# vehicle sits somewhere else entirely. position_error then reads small while the aircraft is
# visibly off the trajectory, and tightening the control gains only tracks the wrong number
# harder.
#
# When sim/truth_state is available these separate the two. In flight it is not, so
# has_truth is false and the fields stay zero.
bool has_truth

# Commanded minus TRUTH, NED metres. This is the error that actually matters for collision
# freedom -- compare it against safety_margin, not position_error.
float64[3] true_position_error
float64 true_position_error_norm

# Truth minus ESTIMATE, NED metres. The state estimator's own error. If this carries most of
# true_position_error, the fix is in the EKF, not in the controller gains.
float64[3] estimator_error
float64 estimator_error_norm
