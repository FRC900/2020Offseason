Header header

float64 voltage
float64 temperature
float64 totalCurrent
float64 totalPower
float64 totalEnergy

float64[20] current
string[20] thingsPluggedIn
bool[20] channelBrownout
bool[20] stickyChannelBrownout

bool    enabled
bool    switchableChannelState
bool    brownout
bool    stickyBrownout
bool    canWarning
bool    stickyCanWarning
bool    stickyCanBusOff
bool    hardwareFault
bool    stickyHardwareFault
bool    stickyFirmwareFault
bool    stickyHasReset
uint32  firmwareMajor
uint32  firmwareMinor
uint32  firmwareFix
uint32  hardwareRev
