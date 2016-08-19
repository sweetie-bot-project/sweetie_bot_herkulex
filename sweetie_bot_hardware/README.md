SweetieBot hardware interface
=============================

sweetie_bot_hardware_herkulex
-----------------------------
Package contains `HerkulexDriver` and `HerkulexArray` components implementation: [](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-herkulex-alt).

To test it run `test_start.ops` script, which contains basic tests.

    $ deployer -s `rospack find sweetie_bot_hardware_herkulex`/scripts/test_start.ops

Test script loads two components `d` (`HerkulexDriver`) и `a` (HerkulexArray). Configuration file `test_servos.cpf` contains description of one servo with ID=0.
    
Now configure `HerkulexDriver`

    > d.port_name = "/dev/ttyS0"
    > d.configure

and `HerkulexArray`

    > a.configure

Check if staus led is green (Color can be set via corresponding property in `test_servos.cpf`) and try different commands:

    > a.setGoal("test_servo", a.JOG_SPEED, 3.14 / 4, 0) 
    > a.setGoal("test_servo", a.JOG_POSITION, 0, 0.5)
    > a.setRegisterRAM("test_servo", "led_control", 2)
    > a.printAllRegistersRAM("test_servo")
