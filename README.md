# L1AC_ardupilot
This project contains an implementation of the L1 Adaptive Control (L1AC) for ardupilot.

## Installation
Our project adds a self-defined mode for L1AC, so the installation of ardupilot is needed to run the controller. For more information about ardupilot, please check the link: [ArduPilot Setup in Linux](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux). For convenience, we integrated the commands needed into a bash script, so you only need to run installation.sh after cloning this repository, and it will finish all the installation for you, both for ardupilot and our L1AC customized files.

```bash
# clone this repository
git clone https://github.com/sigma-pi/L1AC_ardupilot
cd L1AC_ardupilot

# make the bash script executable and then install
chmod +x installation.sh
./installation.sh
```

## Settings
We added mode 29 (Mode Adaptive) as the mode for L1AC. L1AC takes the geometric controller as a baseline, and adds L1 augmentation to improve its behavior. See the references for more information: 
[L1 Adaptive Augmentation for Geometric Tracking Control of Quadrotors](https://ieeexplore.ieee.org/document/9811946),
[L1Quad: L1 Adaptive Augmentation of Geometric Control for Agile Quadrotors with Performance Guarantees](https://arxiv.org/abs/2302.07208),
[Geometric tracking control of a quadrotor UAV on SE(3)](https://ieeexplore.ieee.org/document/5717652).

As a result, there are many parameters introduced, which you can modify in terminal during testing. Feel free to use our default gains for SITL, but you may need to tune the gains for real drone tests.
```bash
# L1 adaptive controller
ASV            # As for the velocity state
ASOMEGA        # As for the rotational velocity state
CTOFFQ1THRUST  # LPF1's cutoff frequency for thrust channel
CTOFFQ1MOMENT  # LPF1's cutoff frequency for moment channel
CTOFFQ2MOMENT  # LPF2's cutoff frequency for moment channel
L1ENABLE       # enable switch for L1 adaptive controller

# Geometric controller
# position P term
GEOCTRL_KPX    # kpx for geometric controller
GEOCTRL_KPY    # kpy for geometric controller
GEOCTRL_KPZ    # kpz for geometric controller
# position D term
GEOCTRL_KVX    # kvx for geometric controller
GEOCTRL_KVY    # kvy for geometric controller
GEOCTRL_KVZ    # kvz for geometric controller
# angular P term
GEOCTRL_KRX    # kRx for geometric controller
GEOCTRL_KRY    # kRy for geometric controller
GEOCTRL_KRZ    # kRz for geometric controller
# angular D term
GEOCTRL_KOX    # kOmegax for geometric controller
GEOCTRL_KOY    # kOmegay for geometric controller
GEOCTRL_KOZ    # kOmegaz for geometric controller

# Other parameters
CIRCSPEED      # circulating speed of the circular trajectory
CIRCRADIUSX    # circle radius or figure8's x radius
CIRCRADIUSY    # figure8's y radius (not used for circle radius)
TRAJINDEX      # index of the trajectory to run
LANDFLAG       # landing control signal, 1 for landing and will disable takeoff, make sure 0 before flight
```

The list of trajectories:
```bash
# TRAJINDEX default value is 0, which means nothing
# you need to specify the trajectory before flight
param set TRAJINDEX 1  # variable yaw circular
param set TRAJINDEX 2  # fixed yaw circular
param set TRAJINDEX 3  # fixed yaw figure8
param set TRAJINDEX 4  # fixed yaw figure8 with tilted altitude 
```

There is an important setting in ardupilot/ArduCopter/config.h. **We added a flag REAL_OR_SITL to tell the code whether to use the settings for real drone tests or SITL simulations (only one place to change this value).** Set 0 for SITL, and 1 for real drone firmware. Check this value before compiling.

## Usage
The basic usage of this project is similar to the official ardupilot. 

For simulation, we list the steps of SITL tests below.
```bash
# for SITL, first make sure REAL_OR_SITL in ardupilot/ArduCopter/config.h is 0
# under L1AC_ardupilot, go to autotest and run the simulation
cd ./ardupilot/Tools/autotest/
python3 sim_vehicle.py --console -A "--uartF=sim:vicon:" --map -v ArduCopter -f X
```
After the console shows "prearm good", you should see an icon of quadcopter on the map. Check parameters and your settings on terminal. For example, we use fixed yaw figure8 (the Lissajous curve which looks like "8") trajectory, and set CIRCSPEED to be 2 m/s. 
```bash
# check the gains
param show GEOCTRL_K*
param show AS*
param show CTOFFQ*

# set the trajectory before flight
param set CIRCRADIUSX 2  # m
param set CIRCRADIUSY 1  # m
param set CIRCSPEED 2    # m/s

# LANDFLAG must be 0 before flight, it would also be better to set L1 off before flight
param set LANDFLAG 0
param set L1ENABLE 0
```
After all the preparations above, arm the drone and then enter the adaptive mode. You need to arm the drone before entering mode 29, and the drone will immediately takeoff once enter the mode.
```bash
arm throttle
rc 3 1200
mode 29
```
You can turn L1 on by setting L1ENABLE to 1, and land the drone by setting LANDFLAG to 1. 

For real donr flights, you can follow the offical way to compile and load the firmware to your board (Pixhawk4-mini, CubeOrange, CubeOrangePlus have been tested).
```bash
# under ardupilot, first make sure REAL_OR_SITL in ardupilot/ArduCopter/config.h is 1
./waf configure --board CubeOrangePlus
./waf copter
```

