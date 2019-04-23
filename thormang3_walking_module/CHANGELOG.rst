^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package thormang3_walking_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2016-08-19)
-----------
* modify CMakeLists.txt for sync
* file name is changed
* bug fix
* Contributors: Jay Song

0.1.0 (2016-08-18)
-----------
* first public release for Kinetic
* modified package information for release
* modified thormang3_walking_module/package.xml
* removed some codes that commented out.
* fifth trajectory is applied
* modified File name
  thormang3_online_walking.h
  thormang3_online_walking.cpp
  thormang3_walking_module.h
  thormang3_wakling_module.cpp
* removed walking_module_math & modify thormang3_balance_control
  added set enable function for each balance algorithm
  removed walking module math
* modified target force calculation
  modified thormang3_walking_module/src/robotis_online_walking.cpp
  add target force calculation for fx fy
* added Function - action play for only specified joints
  modified action_module.h
  modified action_module.cpp
  modified message in walking_module.cpp
* StepData can be changed in walking.
  modify thormang3_walking_module/src/wakling_module.cpp
* ROS C++ coding style is applied.
* fixed high CPU consumption due to busy waits
* Contributors: Alexander Stumpf, Jay Song, Zerom, Pyo, SCH
