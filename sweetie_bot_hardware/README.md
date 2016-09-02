SweetieBot hardware interface
=============================

Package `sweetie_bot_hardware_herkulex`
--------------------------------------

Package contains `HerkulexDriver`, `HerkulexArray` and `HerkulexSched` components implementation: [link](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-herkulex-alt).

`HerkulexDriver` implemets data link layer (interacts with serial port, check packet checksums, translate data frames in `sweetie_bot_hardware_herkulex_msgs::HerkulexPacket` and backward).

`HerkulexArray` represents high-level configuration and monitoring interface. It properties coontains servo descriprions and its registers configurations. 
It provides convinent operation for access registers, read statuses, set individual goal. This component is not meant to work in real-time cycle.

`HerkulexSched` represents real-time interface. Control cycle (discretization period) is divided into three rounds:
1. `RT_JOG` During this round set goal (Herkulex `SJOG`) command is being send to servos. Servo goals are read from `goals` port.
1. `RT_read` During this round states of servos from `poll_list` are acquared. Stete includes position, velocity and etc.
1. `CM` During this round request from `HerkulexArray` are forward to data link layer (`HerkulexDriver`). So real-time and non real-time
    submodules can function simulteniously.
The start of next control cycle is triggered by `sync` port messages.

Servo array configuration can be loaded to `HerkulexArray` properties from OROCOS `*.cpf` file.

### Usage

To test configuration without `HerkulexSched` run:

    $ deployer -s `rospack find sweetie_bot_hardware_herkulex`/scripts/test_herkulex_array.ops

Test script loads two components `herk_driver` (`HerkulexDriver`) и `herk_array` (HerkulexArray). 
Configuration file `test_servos.cpf` contains description of rwo servo with ID=0 and ID=1.
Your can call various opertions from deployer:

    > herk_array.setGoal("test_servo", a.JOG_SPEED, 3.14 / 4, 0) 
    > herk_array.setGoal("test_servo", a.JOG_POSITION, 0, 0.5)
    > herk_array.setRegisterRAM("test_servo", "led_control", 2)
    > herk_array.printAllRegistersRAM("test_servo")

Servo setting can be modifed via properties of `herk_array`. After modification reconfuguration is necessary.

To test real-time configuration run:

    $ deployer -s `rospack find sweetie_bot_hardware_herkulex`/scripts/test_herkulex_sched.ops

Test script loads following components 
1. `sweetie_bot_hardware_herkulex` components: `herk_driver` (`HerkulexDriver`), `herk_array` (HerkulexArray) and `herk_sched` (`HerkulexSched`).
2. `herk_timer` (`OCL::TimerComponent`) generates sync messages for `herk_sched`.
2. `reporter_herk_state` and `reporter_herk_stat` (`OCL::TimerComponent`) store servo positions and communication statistics to files.

Configuration file `test_servos.cpf` contains description of rwo servo with ID=0 and ID=1.
Script sets sequentallty different goal positions for `servo1`.

Rounds setting can be modifed via properties of `herk_sched`. After modification reconfuguration is necessary.


