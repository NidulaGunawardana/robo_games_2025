# Webots Makefile system
include $(WEBOTS_HOME)/resources/Makefile.include

# Set the OpenCV installation path
OPENCV_DIR ?= C:/opencv/build    # Windows
# OPENCV_DIR ?= /usr/local        # Linux

# Include OpenCV headers
INCLUDE = -I$(OPENCV_DIR)/include/opencv4

# Link OpenCV libraries
LIBRARIES = -L"$(OPENCV_DIR)/lib" -lopencv_core -lopencv_imgproc -lopencv_highgui

# Ensure Webots can use these settings
USE_C_API = false
# End of Makefile snippet