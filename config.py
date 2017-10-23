
# Display Settings
# ----------------
debug = True        # Set to False for no data display

# Camera Settings
# ---------------
CAMERA_WIDTH = 320    # default = 320 PiCamera image width can be greater if quad core RPI
CAMERA_HEIGHT = 240   # default = 240 PiCamera image height
CAMERA_HFLIP = False  # True=flip camera image horizontally
CAMERA_VFLIP = False  # True=flip camera image vertically
CAMERA_ROTATION = 0   # Rotate camera image valid values 0, 90, 180, 270
CAMERA_FRAMERATE = 25 # default = 25 lower for USB Web Cam. Try different settings

# Motion Track Settings
# ---------------------
MIN_AREA = 2       # excludes all contours less than or equal to this Area
THRESHOLD_SENSITIVITY = 15
BLUR_SIZE = 3   
