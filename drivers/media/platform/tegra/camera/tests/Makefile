#######################################
# Free-standing Tegra Camera Kernel Tests

#######################################
# Tegra Camera Kernel Tests Utilities
obj-y += utils/tegracam_log.o

#######################################
# Sensor Kernel Tests Module
obj-$(CONFIG_VIDEO_CAMERA_SKT) += sensor_kernel_tests.o
sensor_kernel_tests-y += modules/sensor_kernel_tests_core.o
sensor_kernel_tests-y += modules/sensor_kernel_tests_runner.o
