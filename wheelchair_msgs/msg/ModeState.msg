# Current system mode, published on /mode/state (requirement 3.1.2).
# See docs/requirements/03_mode_management.md.

std_msgs/Header header        # header.stamp = when this state was published

string mode                   # idle | manual | test | auto | fault | estop (3.1.1)
string last_change_reason     # why the mode last changed (3.3.2)
