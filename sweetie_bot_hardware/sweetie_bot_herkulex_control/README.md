SweetieBot Herkulex servos interface
======================================

This package is part of [Sweetie Bot project](sweetiebot.net). It contains `HerkulexDriver`, `HerkulexArray` and `HerkulexSched` 
[OROCOS](http://orocos.org) components which represents [hardware interface of Sweetie Bot](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-herkulex-alt)
for Herkulex DRS 101/202 servos. Nevertheless this package is designed for specific robot it is universal and can be used to control 
any Herkulex servos array. Also [`rtt_ros_integration` package]() can be used to interface OROCOS components from ROS nodes.

`HerkulexDriver` implements data link layer. It interacts with serial port, checks packet checksums, translates data frames 
to `sweetie_bot_herkulex_msgs::HerkulexPacket` messages and backward. 

`HerkulexArray` is high-level configuration and monitoring subsystem. It properties contains servo array descriptions and configuration 
(reverse flag, offset, register values). `HerkulexArray` provides operations to access registers, read statuses, set individual goal. 
This component is not meant to work in real-time environment.

`HerkulexSched` provides real-time interface. Control cycle (discretization period) is divided into three rounds:
1. **`RT_JOG`**. During this round set goal (Herkulex `SJOG`) command is being sent to servos. Servo goals are read from `in_goals` port.
1. **`RT_read`**. During this round states of servos from `poll_list` are acquired. State includes position, velocity and trajectory generator variables 
    (see component interface for more information).

    If `poll_round_size` property < 0 component attempts to request states of all servos. If `poll_round_size` >=0 it requests states only of `poll_round_size` servos
    and on the next `RT_read` round proceeds to next group of `poll_round_size` servos from list.
1. **`CM`**. During this round requests from `HerkulexArray` component are forwarded to data link layer (`HerkulexDriver`). So real-time and non real-time
    submodules can function simulteniously.

The start of next control cycle is triggered by `sync` port messages. Duration of rounds are coosen via `HerkulexSched` component properties.
`period_CM`, `period_RT_read`, `period_RT_JOG` properties declare time periods when a command *can be send to servo*. So actual duration of time rounds 
may be on `timeout` longer. `timeout` declare maximal duration of one exchange opertion (send command to and receive answer from servo).

    min_control_cyle_duration = period_RT_JOG + (period_RT_read + timeout) + (period_CM + timeout) 
                                                   RT read exchange round     CM exchange round

Servo array configuration can be loaded to `HerkulexArray` properties from OROCOS `*.cpf` file. Example configuration for two servos is provided in `scripts/test_servo_array.cpf`

### Limitations

1. `HerkulexDriver` `waitSendPacket()` operation does not function properly with USB-to-serial converters due to `tcdrain()` syscall returns prematurely.
    So if actual `JOG` request execution time is larger then `period_RT_JOG` this can cause next read operation timeout.
1. `HerkulexDriver` reset receiver state after each send opertions. So it is impossible efficently mix commands with ACK and without ACK.
2. `HerkulexArray` read `in_goals` port one time  per control cycle. This can cause unwanted behaviour in presence of multiple writers or if port bufferization is turned on.

### Usage

To test **configuration non-realtime confuguration** (without `HerkulexSched`) run:

    $ deployer -s `rospack find sweetie_bot_hardware_herkulex`/scripts/test_herkulex_array.ops

Test script loads two components `herk_driver` (`HerkulexDriver`) и `herk_array` (HerkulexArray). 
Configuration file `test_servos.cpf` contains description of rwo servo with ID=0 and ID=1.
Your can call various opertions from deployer:

    > herk_array.setGoal("test_servo", a.JOG_SPEED, 3.14 / 4, 0) 
    > herk_array.setGoal("test_servo", a.JOG_POSITION, 0, 0.5)
    > herk_array.setRegisterRAM("test_servo", "led_control", 2)
    > herk_array.printAllRegistersRAM("test_servo")

Servo setting can be modifed via properties of `herk_array`. After modification call `configure` operation to apply changes.

To test **real-time configuration** run:

    $ deployer -s `rospack find sweetie_bot_hardware_herkulex`/scripts/test_herkulex_sched.ops

Test script loads following components 
1. `sweetie_bot_hardware_herkulex` components: `herk_driver` (`HerkulexDriver`), `herk_array` (HerkulexArray) and `herk_sched` (`HerkulexSched`).
2. `herk_timer` (`OCL::TimerComponent`) generates sync messages for `herk_sched`.
2. `reporter_herk_state` and `reporter_herk_stat` (`OCL::TimerComponent`) store servo positions and communication statistics to files.

Configuration file `test_servos.cpf` contains description of rwo servo with ID=0 and ID=1.
Script sequentallty sets different goal positions for `servo1`.

Time rounds setting can be modifed via properties of `herk_sched`. After modification reconfuguration is necessary.
Control cycle period is controlled by `herk_timer` settings.


