#! /bin/bash

# Install ardupilot and the L1AC customized file pack.
# This script is for Ubuntu 20.04 LTS.

# Clone all the submodules
git submodule update --init --recursive

# Install ardupilot
cd ./ardupilot/
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
cd ../

# Install the L1AC customized file pack
rm ./ardupilot/ArduCopter/Copter.h ./ardupilot/ArduCopter/Parameters.cpp ./ardupilot/ArduCopter/Parameters.h ./ardupilot/ArduCopter/config.h ./ardupilot/ArduCopter/mode.cpp ./ardupilot/ArduCopter/mode.h ./ardupilot/ArduCopter/motors.cpp
cp ./L1AC_customization/ArduCopter/ACRL_trajectories.cpp ./L1AC_customization/ArduCopter/ACRL_trajectories.h ./L1AC_customization/ArduCopter/mode_adaptive.cpp ./L1AC_customization/ArduCopter/Copter.h ./L1AC_customization/ArduCopter/Parameters.cpp ./L1AC_customization/ArduCopter/Parameters.h ./L1AC_customization/ArduCopter/config.h ./L1AC_customization/ArduCopter/mode.cpp ./L1AC_customization/ArduCopter/mode.h ./L1AC_customization/ArduCopter/motors.cpp ./ardupilot/ArduCopter/



