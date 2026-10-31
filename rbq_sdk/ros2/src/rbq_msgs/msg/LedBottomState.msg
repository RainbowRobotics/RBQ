# Bottom status LED state, 1 Hz on /rbq/led_bottom (CAN FD robots only). Mirrors idl/rbq/LedBottomState_.idl.
int32 age_ms        # ms since the IF board last answered, -1 = no answer yet
bool configured     # false = the IF board rebooted and holds no setting (Motion resends the last one)
uint32 if_fw        # IF firmware version (yyyymmddnn), 0 = unknown
LedBottomSide right
LedBottomSide left
