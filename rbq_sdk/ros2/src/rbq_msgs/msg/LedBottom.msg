# Bottom status LED command. Mirrors idl/rbq/LedBottom_.idl (FINAL extensibility).
# The same fields as PUT /api/led/bottom. Network checks the ranges and rejects the
# command on robots without CAN FD.
uint8 SIDE_RIGHT=1
uint8 SIDE_LEFT=2
uint8 SIDE_BOTH=3
uint8 MODE_OFF=0
uint8 MODE_ON=1
uint8 MODE_BLINK=2

uint8 side          # SIDE_*; the other LED keeps its setting
uint8 mode          # MODE_*
uint8[3] rgb        # 0-255
uint16 on_ms        # blink only, 10-2550 (10 ms steps)
uint16 off_ms       # blink only, 10-2550 (10 ms steps)
uint8 count         # blink only, repeats, 0 = forever
