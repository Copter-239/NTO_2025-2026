import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover import long_callback
from mavros_msgs.srv import CommandBool
from std_msgs.msg import UInt16MultiArray, Bool
from multiprocessing import Process
from subprocess import run
import setting
Process(target=lambda: run(["python3", "rand_world.py"], capture_output=True)).start()
Process(target=lambda: run(["python3", "app/app.py"])).start()
rospy.init_node('main_node')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
rospy.spin()




