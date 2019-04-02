# easy_handeye_demo
This is a demo for [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) which you can try out without any hardware. It uses MoveIt! and a "noisy" tf publisher to simulate a robot and a realistic tracking system.

This package includes a `calibrate.launch` file which you can use to play around with the calibration software, and a `check_calibration.launch` file to see the impact of a successful calibration.

`calibrate.launch` starts the [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack) MoveIt! demo, along with the easy_handeye calibration software and a script which simulates the output of a tracking system according to the current state of the simulated robot.
You can adjust the noise added to the computed transformation with the respective launch file arguments.

