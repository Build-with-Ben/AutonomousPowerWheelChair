# Safety gate state, published on /safety/state (requirement 5.6.1).
# See docs/requirements/05_safety.md.

std_msgs/Header header        # header.stamp = when this state was published

bool ok                       # true only when no fault is latched and no e-stop is active (5.6.1.1)
string[] faults               # every active fault, by name (5.6.1.2)
bool latched                  # true while any fault is latched (5.6.1.3)
string[] warnings             # active warnings that do not stop the chair (e.g. low_battery)
