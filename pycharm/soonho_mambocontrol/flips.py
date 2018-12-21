from pyparrot.Minidrone import Mambo
from pprint import pprint

bt_mac_01 = "d0:3a:de:8a:e6:37"#"90:3a:e6:21:82:0a"
bt_mac_02 = "d0:3a:82:0a:e6:21"#"90:3a:e6:21:82:0a"

drone = Mambo(address=bt_mac_01, use_wifi=False)


print("trying to connect")
success_01 = drone.connect(num_retries=3)
print("drone_01 connected: %s " % success_01)

if (success_01):
    
    print("taking off!")
    drone.safe_takeoff(5)

    if (drone.sensors.flying_state != "emergency"):
        print("flying state is %s" % drone.sensors.flying_state)
        print("Flying direct: going up")
        drone.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=20, duration=1)

        print("flip left")
        print("flying state is %s" % drone.sensors.flying_state)
        success = drone.flip(direction="left")
        print("mambo flip result %s" % success)
        drone.smart_sleep(2)

        print("flip right")
        print("flying state is %s" % drone.sensors.flying_state)
        success = drone.flip(direction="right")
        print("mambo flip result %s" % success)
        drone.smart_sleep(2)

        print("flip front")
        print("flying state is %s" % drone.sensors.flying_state)
        success = drone.flip(direction="front")
        print("mambo flip result %s" % success)
        drone.smart_sleep(2)

        print("flip back")
        print("flying state is %s" % drone.sensors.flying_state)
        success = drone.flip(direction="back")
        print("mambo flip result %s" % success)
        drone.smart_sleep(2)

        print("landing")
        print("flying state is %s" % drone.sensors.flying_state)
        drone.safe_land(5)
        drone.smart_sleep(2)

    print("disconnect")
    drone.disconnect()