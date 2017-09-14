#!/usr/bin/python

# Control options.
CAMERA_TILT_RATE = 7e-4 * 4
CAMERA_FACING_RATE = 7e-4 * 4

# Smoothness options.
CAMERA_SMOOTHING_FACTOR = 0.3
USE_MOVE_PREDICTION = True
USE_JITTER_CORRECTION = True

# Server settings.
SERVER_STATE_FRAMES = 90
SERVER_PATCHES_FRAMES = 3

#STATE_COMPRESS = lambda s: s
#STATE_DECOMPRESS = lambda s: s

STATE_COMPRESS = lambda s: s.encode("zlib")
STATE_DECOMPRESS = lambda s: s.decode("zlib")

