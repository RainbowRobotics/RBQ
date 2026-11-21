# One bottom status LED as the IF board reports it. Mirrors idl/rbq/LedBottomSide_.idl.
uint8 mode          # 0 off / 1 on / 2 blink
uint8[3] rgb        # set colour
uint16 on_ms        # blink time [ms], 0 unless blinking
uint16 off_ms
uint8 count         # blink repeats, 0 = forever, 0 unless blinking
uint8[3] actual     # last 10 ms output after the power limit, 0 when off
bool limited        # the power limit reduced the output within the last second
bool legacy         # set through the old command, so the setting shown is approximate
float32 lpf_w       # filtered LED power [W]
