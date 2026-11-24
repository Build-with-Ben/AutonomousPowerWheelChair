# Device-independent operator input, published on /operator/input (requirement 2.1.2).
# See docs/requirements/02_operator_input.md.
#
# wheelchair_input produces this from a raw device (/joy) and a device profile, so that
# wheelchair_teleop never sees an axis or button index. Any controller works as long as
# some node publishes this message; the profile holds everything device specific.

std_msgs/Header header        # header.stamp = when the raw device message was received

string source                 # profile name of the device that produced this, e.g. 'xbox_generic'

# Drive demand, already normalised and sign corrected to ROS conventions (2.2.1).
# Teleop applies the deadzone, speed scale and maximum speeds on top of these.
float32 linear                # -1.0 .. +1.0, positive is forward
float32 angular               # -1.0 .. +1.0, positive is a left turn (counter-clockwise)

# Function states. All are the *held* state of the control; teleop takes the rising
# edge where a requirement asks for a press rather than a hold.
bool deadman                  # non-zero output is allowed only while this is held (2.3.1)
bool estop                    # software e-stop request (2.3.4)
bool speed_up                 # next speed scale step (2.2.5)
bool speed_down               # previous speed scale step (2.2.5)
bool mode_manual              # request Manual mode from Mode Management (2.3.5)
