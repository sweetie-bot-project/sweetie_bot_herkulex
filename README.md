SweetieBot Herkulex servos interface
======================================

This package contains OROCOS components to control Herkulex DRS 101/202 servos.
[OROCOS](http://orocos.org) is C++ framework targeting the implementation of real-time control systems. It is compatible with ROS and OROCOS 
components can be easily accessed from ROS nodes, see [`rtt_ros_integration` package](http://wiki.ros.org/rtt_ros_integration).
Full documentation is available in Russian [here](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-herkulex-alt).

Package contains three OROCOS components `HerkulexDriver`, `HerkulexArray` and `HerkulexSched`, startup scripts and launch files.

`HerkulexDriver` component implements data link layer. It handles serial port interaction, checks packet checksums, translates data frames 
to `sweetie_bot_herkulex_msgs::HerkulexPacket` messages and backward. 

`HerkulexArray` is high-level configuration and monitoring subsystem. This component is not meant to work in real-time environment.
It can be used to configure servos array (associate names with hardware ID, automatically set servo register to specific values at startup,
configure reverse flag, offsets for each servos). Also `HerkulexArray` provides operations to access registers, read servo statuses, 
control movements and etc.

`HerkulexSched` provides real-time interface. It operates in control cycle (discretization period). Each control cycle is divided into three rounds:
1. **`RT_JOG`**. At this round set goal (Herkulex `SJOG`) command is sent to servos. Target positions are read from `in_goals` port (`herkulex_msgs::ServoGoal`).
1. **`RT_read`**. During this round `HerkulexSched` sends read request to servos which are listed in `poll_list` parameter. It requests positions and velocities 
    form each servo and publish them using `out_joints` port (`sensor_msgs::JointState`). If `detailed_state` parameter is set also pwm and trajectory generator variables 
    are acqured and publised. See component interface for more information.

    If `poll_round_size` property less then zero component attempts to request states of all servos. If `poll_round_size` greate or equal to zero 
    `HerkulexSched` requests states only of `poll_round_size` servos and on the next `RT_read` round proceeds to next group of `poll_round_size` servos from poll list.
1. **`CM`**. During this round requests from `HerkulexArray` component are forwarded to data link layer (`HerkulexDriver`). So real-time and non real-time
    submodules can function simulteniously.

The start of next control cycle is triggered by `sync` port messages. Duration of rounds are chosen via `HerkulexSched` component properties.
`period_CM`, `period_RT_read`, `period_RT_JOG` properties declare time periods when a command *can be send to servo*. So actual duration of time rounds 
may be on `timeout` longer. `timeout` declare maximal duration of one exchange operation (send command to and receive answer from servo).

    min_control_cyle_duration = period_RT_JOG + (period_RT_read + timeout) + (period_CM + timeout) 
                                                   RT read exchange round     CM exchange round

Servo array configuration can be loaded to `HerkulexArray` properties from OROCOS `*.cpf` file. Example configuration for two servos is provided in `scripts/test_servo_array.cpf`

*Configuration hints*: 
* At 115600 baurdrate use 4-5 ms `timeout` value. 
* It is better to set `period_RT_JOG` to `SJOG` command duration (depends on number of servos) plus 2-3 ms. During this time servos would be able process received command.
* Set `period_RT_read` to `(poll_round_size-1)*timeout + 1 ms`. So maximal duration of `RT_read` round would be `poll_round_size*timeout + 1 ms`
* Control cycle duration can be controlled using external timer component from OCL library.

### Limitations

1. `HerkulexDriver` `waitSendPacket()` operation does not function properly with USB-to-serial converters due to `tcdrain()` syscall returns control prematurely.
    So if actual `JOG` request execution time is larger then `period_RT_JOG` this can cause next read operation timeout.
1. `HerkulexDriver` reset receiver state after each send operations. So it is impossible efficiently mix commands with ACK and without ACK.
2. `HerkulexArray` read `in_goals` port one time  per control cycle. This can cause unwanted behaviour in presence of multiple writers or if port bufferization is turned on.

### Konown issues

if `reset_delay` property of HerkulexArray is to small it is unable to reset and discover servos. In some cases it can case unpredictable servo behavior 
probably due command misinterpretation.

### Usage

**TODO**: rttlua deployment scripts (from `sweetie_bot_deploy`).


**TODO**: launch as separate ROS node.


Test array must contains two servos with hardware ID 0 and 1. Serial port `/dev/ttyUSB0` is used with baudrate 115600.
See files in script folder for more details.

To test **configuration non-realtime confuguration** (without `HerkulexSched`) run:

    $ roslaunch sweetie_bot_herkulex_control test_herkulex_array.launch

Test script loads two components `herk_driver` (`HerkulexDriver`) и `herk_array` (HerkulexArray). 
Configuration file `test_servos.cpf` contains description of two servo with ID=0 and ID=1.
Your can call various `HerkulexArray` operations from deployer (use	`ls herk_array` to browse HerkulexArray interface):

    > herk_array.printAllServosStatuses()
    > herk_array.setGoal("servo", a.JOG_SPEED, 3.14 / 4, 0) 
    > herk_array.setGoal("servo", a.JOG_POSITION, 0, 0.5)
    > herk_array.setRegisterRAM("test_servo", "led_control", 2)
    > herk_array.printAllRegistersRAM("test_servo")

Servo setting can be modified via properties of `herk_array`. After modification call `cleanup` and `configure` operations to apply changes.

To test **real-time configuration** run:

    $ roslaunch sweetie_bot_herkulex_control test_herkulex_sched.launch

The script loads following components:
1. `sweetie_bot_hardware_herkulex` components: `herk_driver` (`HerkulexDriver`), `herk_array` (HerkulexArray) and `herk_sched` (`HerkulexSched`).
2. `herk_timer` (`OCL::TimerComponent`) which generates sync messages for `herk_sched`.
2. `reporter_herk_state` and `reporter_herk_stat` (`OCL::TimerComponent`) store servo positions and communication statistics to files.

Configuration file `test_servos.cpf` contains description of two servo with ID=0 and ID=1.
Script sequentiality sets different goal positions for `servo1`.

Time rounds setting can be modified via properties of `herk_sched`. After modifications reconfuguration is necessary.  
Control cycle period is controlled by `herk_timer` settings.

