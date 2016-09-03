SweetieBot hardware interface
=============================

Package `sweetie_bot_hardware_herkulex`
--------------------------------------

Package contains `HerkulexDriver`, `HerkulexArray` and `HerkulexSched` components implementation: [link](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-herkulex-alt).

`HerkulexDriver` represents data link layer. It interacts with serial port, checks packet checksums, translates data frames 
in `sweetie_bot_hardware_herkulex_msgs::HerkulexPacket` messages and backward.

`HerkulexArray` is high-level configuration and monitoring subsystem. It properties coontains servo array descriprions and intial servos 
configuration (reverse falg, offset, register values).  `HerkulexArray` provides operations to access registers, read statuses, set individual goal. 
This component is not meant to work in real-time enviroment.

`HerkulexSched` provides real-time interface. Control cycle (discretization period) is divided into three rounds:
1. **`RT_JOG`**. During this round set goal (Herkulex `SJOG`) command is being send to servos. Servo goals are read from `goals` port.
1. **`RT_read`**. During this round states of servos from `poll_list` are acquared. State includes position, velocity and possibly trajectory generator variables.
1. **`CM`**. During this round requests from `HerkulexArray` are forwarded to data link layer (`HerkulexDriver`). So real-time and non real-time
    submodules can function simulteniously.

The start of next control cycle is triggered by `sync` port messages. Duration of frame are coosen via component properties.
`period_CM`, `period_RT_read`, `period_RT_JOG` declare time periods when command can be send to servo. Actual exchange may lasts longer.
`timeout` is maximal request duration (write and read). To pervent packet clases set

    min_control_cyle_duration > period_RT_JOG + (period_RT_read + timeout) + (period_CM + timeout)

                                                   RT read exchange period     CM exchange period

Servo array configuration can be loaded to `HerkulexArray` properties from OROCOS `*.cpf` file. Example configuration is provided.

### Limitations

1. `HerkulexDriver` `waitSendPacket()` operation does not function properly with USB-to-serial converters due to `tcdrain()` syscall returns prematurely.
    If actual `JOG` request execution time is larger then `period_RT_JOG` this can cause next read operation timeout.
2. `HerkulexArray` read `goals_in` port one time  per control cycle. This can cause unwanted behaviour in presence of multiple writers or port bufferization.

### Usage

To test **configuration without `HerkulexSched`** run:

    $ deployer -s `rospack find sweetie_bot_hardware_herkulex`/scripts/test_herkulex_array.ops

Test script loads two components `herk_driver` (`HerkulexDriver`) и `herk_array` (HerkulexArray). 
Configuration file `test_servos.cpf` contains description of rwo servo with ID=0 and ID=1.
Your can call various opertions from deployer:

    > herk_array.setGoal("test_servo", a.JOG_SPEED, 3.14 / 4, 0) 
    > herk_array.setGoal("test_servo", a.JOG_POSITION, 0, 0.5)
    > herk_array.setRegisterRAM("test_servo", "led_control", 2)
    > herk_array.printAllRegistersRAM("test_servo")

Servo setting can be modifed via properties of `herk_array`. After modification reconfuguration is necessary.

To test **real-time configuration** run:

    $ deployer -s `rospack find sweetie_bot_hardware_herkulex`/scripts/test_herkulex_sched.ops

Test script loads following components 
1. `sweetie_bot_hardware_herkulex` components: `herk_driver` (`HerkulexDriver`), `herk_array` (HerkulexArray) and `herk_sched` (`HerkulexSched`).
2. `herk_timer` (`OCL::TimerComponent`) generates sync messages for `herk_sched`.
2. `reporter_herk_state` and `reporter_herk_stat` (`OCL::TimerComponent`) store servo positions and communication statistics to files.

Configuration file `test_servos.cpf` contains description of rwo servo with ID=0 and ID=1.
Script sets sequentallty different goal positions for `servo1`.

Rounds setting can be modifed via properties of `herk_sched`. After modification reconfuguration is necessary.


