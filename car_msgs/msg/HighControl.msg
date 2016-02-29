# Timestamped high level RACECAR control

Header      header
string      module
string      msg_type

# use these flags at the beginning and end of a module's lifetime to register in the car controller
bool     subscribe
bool     unsubscribe

DirectDrive drive_msg