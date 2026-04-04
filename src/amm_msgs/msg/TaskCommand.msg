# Structured task command parsed from a natural language prompt
std_msgs/Header header

string object_name        # e.g. "apple"
int32  source_room        # 1, 2, or 3
string destination        # e.g. "base"
string action_type        # e.g. "pick_and_place"
