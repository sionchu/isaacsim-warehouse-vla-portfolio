# Technical Sources and Wording Notes

## Tolerance wording

Avoid presenting `0.3–0.8 mm` as a universal aircraft manufacturing tolerance. Requirements depend on the feature, material stack, assembly method, and measurement definition.

A defensible public phrase is:

> Aerospace robotic drilling often operates under sub-millimeter positioning requirements, with published systems reporting requirements or demonstrated performance ranging from approximately ±0.5 mm to below 0.3 mm, and tighter values for some fastener-hole applications.

Published examples:

- A robotic drilling and riveting end effector reported positioning precision within ±0.5 mm and perpendicular accuracy within 0.3 degrees.
- A moving-rail robotic drilling calibration study reduced absolute positioning error from nearly 2 mm to below 0.3 mm.
- A vision-based robotic drilling paper cites ±0.2 mm fastener-hole positioning requirements.
- A robot calibration study reported an improvement from 2 mm to 0.1 mm in its experimental drilling region.

## Why OLP has been important

Dassault Systèmes describes DELMIA Robotics Offline Programming as supporting production programming without stopping the physical cell, calibration of the virtual workcell against the shop floor, native robot-program exchange, collision validation, and reach analysis.

RoboDK defines the TCP as the transformation from the robot flange to the point used for Cartesian robot targets and emphasizes that correct TCP definition is essential for both online and offline programming.

NIST notes that industrial robot arms may exhibit average Cartesian absolute-positioning errors of several millimeters even when their mechanical construction and repeatability are good. Calibration of robot kinematic parameters and joint zero offsets is therefore critical for simulation-assisted high-tolerance work.

## References

- [DELMIA Robotics Offline Programming datasheet](https://www.3ds.com/fileadmin/PRODUCTS-SERVICES/DELMIA/PDF/DM-12844-Robotics-Offline-Programming-Datasheet-Equity-Update_HR.pdf)
- [Dassault Systèmes aerospace drilling and riveting virtual commissioning](https://r1132100503382-eu1-my3dexperience.3dexperience.3ds.com/welcome/compass-world/3dexperience-industries/aerospace-and-defense/ready-for-rate/drilling-and-riveting-for-assembly/virtual-commissioning-analyst)
- [DELMIA Robotics F-35 project case study](https://www.3ds.com/newsroom/press-releases/variation-reduction-solutions-inc-selects-dassault-systemes-delmia-robotics-f-35-jsf-project)
- [RoboDK TCP definition and calibration](https://robodk.com/doc/en/General-Define-Tool-TCP.html)
- [NIST: Efficiently Improving and Quantifying Robot Accuracy In Situ](https://www.nist.gov/publications/efficiently-improving-and-quantifying-robot-accuracy-situ)
- [Calibration of robotic drilling systems with a moving rail](https://doi.org/10.1016/j.cja.2014.10.028)
- [Measurement error analysis for robotic drilling](https://doi.org/10.1016/j.rcim.2013.09.014)
- [Development of a Non-Parametric Robot Calibration Method to Improve Drilling Accuracy](https://doi.org/10.4271/2021-01-0003)
- [Design of a drilling and riveting multifunctional end effector](https://tnuaa.nuaa.edu.cn/html/2018/3/20180316.htm)
- [NVIDIA Isaac Sim ROS 2 Bridge](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.ros2.bridge/docs/index.html)
