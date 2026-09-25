# Makefile for ball_detector Webots controller
# 球检测控制器：PD 站立/trot + 橙色球纯 C 像素检测（无 OpenCV）
# Uses Webots standard Makefile.include

# WEBOTS_HOME defaults to /usr/local/webots (apt installed)
# Override if needed: export WEBOTS_HOME=/path/to/webots

# Use Webots C API (headers in include/controller/c/)
USE_C_API = true

# Do not modify the following: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
