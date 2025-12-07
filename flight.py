import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from mavros_msgs.srv import CommandBool
from std_msgs.msg import UInt16MultiArray, Bool
rospy.init_node('flight')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

navigate(x=0, y=0, z=2, frame_id='aruco_map', auto_arm=True)
navigate(x=4.5, y=4.5, z=0, frame_id='aruco_map')
navigate(x=0, y=0, z=2, frame_id='aruco_map')


