Header header

uint32  pingId              # incrementing counter inside the sonar
uint32  pingFiringDate      # Ping firing date (sonar internal clock, microseconds)
                            # /!\ Will overlap in a bit more than 1h. Ths sonar
                            # itself gives this value in 32bits. Maximum
                            # number of usec in 32bits gives ~ 1h.

float64 range               # Maximum range value in this ping
float64 gainPercent         # Percentage of gain (not documented)

float64 frequency           # Ping acoustic frequency (Hz)
float64 speedOfSoundUsed    # Speed of sound used by the sonar for range calculations (m/s)
float64 rangeResolution     # Distance between 2 rows in the ping image data.


float64 temperature         # External temperature (C)
float64 pressure            # External pressure    (bar)

uint8   masterMode          # 1 is "low frequency" (1.2MHz), 2 is high frequency (2.1 MHz)
bool    hasGains            # Each row in the image data comes with a gain (to
                            # be compensated for consistent acoustic readings.
                            # See documentation for more details)

uint16  nRanges             # Height of the ping image data.
uint16  nBeams              # Width  of the ping image data.
uint32  step                # Size in bytes of each row in the ping data image.
uint8   sampleSize          # Size in bytes of each "pixel" in the ping data.

int16[] bearings            # Bearing angle of each column of the sonar data
                            # (in 100th of a degree, multiply by 0.01 to get a
                            # value in degrees).
                            # The sonar image is not sampled uniformly in the
                            # bearing direction.

uint8[] pingData            # Ping data (is a row major image in little-endian). 
                            # Size in bytes of each pixel is given in the
                            # sampleSize field. The size in bytes of each line
                            # is given in the step field.  If the hasGains
                            # field is true, each row starts with 4 bytes
                            # containing gain of the row (encoded as a little
                            # endian uint32. Divide the whole row by the square
                            # root of this gain to have consistent value across
                            # the image data).

