^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ridgeback_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2022-05-17)
------------------

0.3.1 (2021-06-15)
------------------

0.3.0 (2020-11-12)
------------------
* [ridgeback_base] Added dependency.
* [ridgeback_base] Updated to use new functions from puma_motor_drivers.
* [ridgeback_base] Used ros::ok() for while condition on thread loops.
* Updates for C++11.
* Contributors: Tony Baltovski

0.2.6 (2020-11-12)
------------------
* Bump CMake version to avoid CMP0048 warning.
* [ridgeback_base] Fixed missing forward slash.
* Contributors: Tony Baltovski

0.2.5 (2020-10-19)
------------------
* Use eval + find to properly load the default mag config file
* Added RIDGEBACK_MAG_CONFIG to madgwick filter and set a default optenv
* Removed env-hooks
* Contributors: Chris Iverach-Brereton, Dave Niewinski

0.2.4 (2019-11-22)
------------------
* Merge pull request `#26 <https://github.com/ridgeback/ridgeback_robot/issues/26>`_ from ridgeback/RPSW-119
  Added the ability to disable using the MCU
* [ridgeback_base] Added param type for use_mcu.
* [ridgeback_base] Added lighting and cooling under use_mcu flag.
* [ridgeback_base] Added envar for using MCU.
* Contributors: Tony Baltovski

0.2.3 (2019-03-23)
------------------
* [ridgeback_base] Updated compute_calibration to use a MagneticField message.
* Contributors: Tony Baltovski

0.2.1 (2018-08-02)
------------------

0.2.0 (2018-05-23)
------------------
* [ridgeback_base] Added absolute value checking for fans.
* Updated to package format 2.
* [ridgeback_base] Switched to rosserial_server_udp.
* Updated bringup for kinetic
* [ridgeback_base] Fixed a typo.
* Updated maintainer.
* Fixed temperature warning for PCB and MCU temperature.
* Contributors: Dave Niewinski, Tony Baltovski

0.1.7 (2016-10-03)
------------------
* Removed GPS dependencies.
* Contributors: Johannes Meyer, Tony Baltovski

0.1.6 (2016-04-22)
------------------
* Combined hostname and version for hardware ID.
* Fixed initialization of constant doubles.
* Added hardware ID based hostname.
* Contributors: Tony Baltovski

0.1.5 (2016-03-02)
------------------
* Fixed state order for lighting.
* Separated passive joint into header.
* Contributors: Tony Baltovski

0.1.4 (2015-12-01)
------------------

0.1.3 (2015-11-20)
------------------
* Install the ridgeback_node target.
* Contributors: Mike Purvis

0.1.2 (2015-11-20)
------------------
* Fixed dependency.
* Contributors: Tony Baltovski

0.1.1 (2015-11-20)
------------------
* Added lighting.
* Contributors: Tony Baltovski

0.1.0 (2015-11-19)
------------------
* Initial Ridgeback release.
* Contributors: Mike Purvis, Tony Baltovski
