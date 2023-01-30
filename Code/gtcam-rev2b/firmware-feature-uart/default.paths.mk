# This makefile includes paths that might be different between different users.
# It should not be included in VCS commits with values other than the defaults.

# default.paths.mk should be copied to paths.mk and customized

# CUBE_PATH should point to the root directory of the appropriate STM32CubeXX SDK for the
# microcontroller used in this project. For instance, if the microcontroller is a STM32F769I, then
# CUBE_PATH should point to a copy of STM32CubeF7.
CUBE_PATH = /opt/stm/STM32CubeL4
