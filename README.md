# L1Quad
This project contains an implementation of the L1 Adaptive Control (L1AC) for quadrotors based on ardupilot. The details on the algorithm and experimental results can be found in the following paper:
* [L1Quad: L1 Adaptive Augmentation of Geometric Control for Agile Quadrotors with Performance Guarantees](https://arxiv.org/abs/2302.07208). 

The demo can be accessed [here](https://youtu.be/18-2OqTRJ50?si=T6rJvlOqwevCKzZk).

## Installation
Follow the steps below to clone this repository and install it on your computer. It has been tested on Ubuntu 20.04. You can alternatively set up the Ardupilot development environment on your own following [ArduPilot Setup in Linux](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux).

```bash
# clone this repository
git clone https://github.com/sigma-pi/L1Quad
cd L1Quad

# make the bash script executable and then install
chmod +x installation.sh
./installation.sh
```

## Usage
Below, we show the usage of L1Quad in Ardupilot's software-in-the-loop (SITL) simulation. 

```bash
# for SITL, first make sure REAL_OR_SITL in ardupilot/ArduCopter/config.h is 0
# under L1Quad, go to autotest and run the simulation
cd ./ardupilot/Tools/autotest/
python3 sim_vehicle.py --console -A "--uartF=sim:vicon:" --map -v ArduCopter -f X
```
After the console shows "prearm good", you should see an icon of quadcopter on the map. Check the parameters and settings in the same terminal (as the one running python script above). We will command the quadrotor to fly a figure8 trajectory. 
```bash
# set the figure8 trajectory (2 m long and 1 m wide, with a maximum linear speed of 2 m/s) before the flight
param set TRAJINDEX 3  # figure8 trajectory
param set CIRCRADIUSX 2  # m
param set CIRCRADIUSY 1  # m
param set CIRCSPEED 2    # m/s

# LANDFLAG must be 0 before the flight. It would also be better to set L1 off (L1ENABLE = 0) before the flight
param set LANDFLAG 0
param set L1ENABLE 0
```
Now you can arm the quadrotor and then enter the adaptive mode (mode 29). You need to arm the quadrotor before entering mode 29: it will immediately take off and gradually accelerate to 2 m/s while flying the figure8 trajectory.
```bash
arm throttle
rc 3 1200
mode 29
```

You can turn L1 on by setting L1ENABLE to 1 at any time during the flight by
```bash
param set L1ENABLE 1  # this will turn L1 on
```

You can land the quadrotor with the following command:
```bash
param set LANDFLAG 1
```

## Fly L1Quad on a real quadrotor

Please contact [Sheng Cheng](https://github.com/Sheng-Cheng) if you want to use it for flights with a real quadrotor: You may need to tune the controller gains for flight tests with real quadrotors.

You will follow the Ardupilot's firmware compile procedure and load the custom firmware to your Pixhawk (Pixhawk4-mini, CubeOrange, CubeOrangePlus have been tested).
```bash
# under ardupilot, first make sure REAL_OR_SITL in ardupilot/ArduCopter/config.h is 1
./waf configure --board CubeOrangePlus
./waf copter
```

## Details on configuration and settings
We implemented L1Quad as a custom mode (29, Mode Adaptive) following the [instructions by Ardupilot](https://ardupilot.org/dev/docs/apmcopter-adding-a-new-flight-mode.html). For our implementation, we use the [geometric controller](https://ieeexplore.ieee.org/document/5717652) as the baseline control for trajectory tracking.

Below we list relevant parameters that you can modify. You may need to tune the controller gains for flight tests with real quadrotors.
```bash
# L1 adaptive controller
ASV            # As for translational dynamics in L1's state predictor
ASOMEGA        # As for the rotational dynamics in L1's state predictor
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
CIRCSPEED      # speed of a circular trajectory (maximum speed of a figure8 trajectory)
CIRCRADIUSX    # circle radius or figure8's x radius (length)
CIRCRADIUSY    # figure8's y radius (width) (Note: this parameter is not used for a circular trajectory)
TRAJINDEX      # index of the trajectory to fly (see the list below)
LANDFLAG       # landing control signal. Setting it to 1 to switch to landing operation and it will disable takeoff.  Make sure to set it to 0 before the flight.
```

List of trajectories:
```bash
# TRAJINDEX default value is 0, which will be default hover at 1 m.
# You need to specify the trajectory before the flight
param set TRAJINDEX 1  # variable yaw circular
param set TRAJINDEX 2  # fixed yaw circular
param set TRAJINDEX 3  # fixed yaw figure8
param set TRAJINDEX 4  # fixed yaw figure8 with tilted altitude 
```

**Note**:
In ardupilot/ArduCopter/config.h, **we added a flag REAL_OR_SITL to tell the compiler whether to use the settings for real drone tests or SITL simulations.** Set it to 0 (default) for SITL, and 1 for compiling firmware for a Pixhawk. Check this value before compiling.

## License
Please read the license attached to this repository.

## Citation
If this repository is helpful for your project, please cite the following papers:
```bash
@inproceedings{wu20221L1adaptive,
  title={L1 adaptive augmentation for geometric tracking control of quadrotors},
  author={Wu, Zhuohuan and Cheng, Sheng and Ackerman, Kasey A and Gahlawat, Aditya and Lakshmanan, Arun and Zhao, Pan and Hovakimyan, Naira},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)},
  pages={1329--1336},
  year={2022},
  organization={IEEE}
}
@article{wu2025l1quad,
  title={{L1Quad}: L1 Adaptive Augmentation of Geometric Control for Agile Quadrotors With Performance Guarantees},
  author={Wu, Zhuohuan and Cheng, Sheng and Zhao, Pan and Gahlawat, Aditya and Ackerman, Kasey A and Lakshmanan, Arun and Yang, Chengyu and Yu, Jiahao and Hovakimyan, Naira},
  journal={IEEE Transactions on Control Systems Technology},
  volume={33},
  number={2},
  pages={597--612},
  year={2025},
  publisher={IEEE}
}
```
