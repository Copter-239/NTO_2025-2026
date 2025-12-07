import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover import long_callback
from mavros_msgs.srv import CommandBool
from std_msgs.msg import UInt16MultiArray, Bool
from multiprocessing import Process
from subprocess import run
Process(target=lambda: run(["python3", "app/app.py"], capture_output=True)).start()
rospy.init_node('main_node')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
is_starting_process_running_flight = False
process_running_flight = Process(target=lambda: run(["python3", "flight.py"], capture_output=True))
def bool_start_or_stop_callback(msg):
    global process_running_flight, is_starting_process_running_flight
    if msg.data:
        if not is_starting_process_running_flight:
            is_starting_process_running_flight = True
            process_running_flight.start()
    else:
        if is_starting_process_running_flight:
            is_starting_process_running_flight = False
            process_running_flight.terminate()
            process_running_flight.join()
rospy.Subscriber('bool_start', Bool, bool_start_or_stop_callback)
rospy.spin()


