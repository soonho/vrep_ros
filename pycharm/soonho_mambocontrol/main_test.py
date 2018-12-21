from pyparrot.Minidrone import Mambo
from pprint import pprint

bt_mac_01 = "d0:3a:de:8a:e6:37"#"90:3a:e6:21:82:0a"
bt_mac_02 = "d0:3a:82:0a:e6:21"#"90:3a:e6:21:82:0a"

drone_01 = Mambo(address=bt_mac_01, use_wifi=False)
drone_02 = Mambo(address=bt_mac_02, use_wifi=False)


print("trying to connect")
success_01 = drone_01.connect(num_retries=3)
success_02 = drone_02.connect(num_retries=3)
print("drone_01 connected: %s " % success_01)
print("drone_02 connected: %s " % success_02)

if (success_01) & success_02:
    # get the state information
    print("test stats")
    drone_01.ask_for_state_update()
    drone_02.ask_for_state_update()
    pprint(vars(drone_01.sensors))
    pprint(vars(drone_02.sensors))

    drone_01.disconnect()
    drone_02.disconnect()

    # print("taking off!")
    # drone.safe_takeoff(5)
    # for i in range(1000) :
    #     drone.ask_for_state_update()
    #     pprint(vars(drone.sensors))
    #     drone.smart_sleep(0.01)
