#!/usr/bin/python

# Control options.
CAMERA_TILT_RATE = 7e-4 * 4
CAMERA_FACING_RATE = 7e-4 * 4

# Smoothness options.
CAMERA_SMOOTHING_FACTOR = 0.3
USE_MOVE_PREDICTION = True
USE_JITTER_CORRECTION = True

# Server settings.
SERVER_STATE_FRAMES = 90 * 3
SERVER_PATCHES_FRAMES = 30

#STATE_COMPRESS = lambda s: s
#STATE_DECOMPRESS = lambda s: s

STATE_COMPRESS = lambda s: s.encode("zlib")
STATE_DECOMPRESS = lambda s: s.decode("zlib")

# Global values that don't need to be reconfigured ever.
ENTITY_CREATION_OFFSET = 1000000000

