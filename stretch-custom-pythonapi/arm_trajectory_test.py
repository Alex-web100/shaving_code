import time
import stretch_body.arm
import stretch_body.lift
from stretch_body.hello_utils import ThreadServiceExit

a = stretch_body.arm.Arm()
l = stretch_body.lift.Lift()
a.startup()
l.startup()
try:
    a.motor.disable_sync_mode()
    l.motor.disable_sync_mode()
    if not a.startup():
        exit() # failed to start arm!
    
    starting_position = a.status['pos']
    starting_position_l = l.status['pos']
    a.move_to(starting_position-0.3)
    a.push_command()
    a.motor.wait_until_at_setpoint()
    l.move_to(starting_position_l-0.3)
    l.push_comamand()
    l.motor.wait_until_at_setpoint()
    l.move_to(0.7)
    l.push_command()
    l.motor.wait_until_at_setpoint()
    temp = a.status['pos']
    a.move_by(0.4)
    a.push_command()
    a.motor.wait_until_at_setpoint()
    l.move_to(0.65)
    l.push_command()
    l.motor.wait_until_at_setpoint()
    starting_position = a.status['pos']
    a.trajectory.add(t_s=0,x_m=starting_position)
    a.trajectory.add(t_s=3, x_m = (starting_position-temp)/2)
    a.trajectory.add(t_s=10, x_m=temp)

    a.follow_trajectory()
    time.sleep(10)

except: (KeyboardInterrupt, SystemExit, ThreadServiceExit) 
pass

l.stop()
a.stop()
