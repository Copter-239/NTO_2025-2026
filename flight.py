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
bridge = CvBridge()
# Yavnoe ozhidanie servisov
rospy.wait_for_service('get_telemetry')
rospy.wait_for_service('navigate')
rospy.wait_for_service('set_position')
rospy.wait_for_service('land')

# Sozdanie proksi servisov
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Dopolnitelnoe ozhidanie dlya inicializacii
rospy.sleep(5)

print("Nachinayu vzlet...")

# Vzlet na 2 metra s ispolzovaniem freyma 'body'
# Parametr auto_arm=True dlya avtomaticheskogo arminga
navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True, speed=0.5)

# Ozhidanie zaversheniya vzleta (nuzhno bolshe vremeni dlya vzleta)
rospy.sleep(8)

# Proverka telemetrii posle vzleta
telem = get_telemetry(frame_id='navigate_target')
print(f"Tekushchaya poziciya: x={telem.x:.2f}, y={telem.y:.2f}, z={telem.z:.2f}")

# Dvizhenie vpered i vpravo na 4.5 metra po kazhdoy osi
# Obratite vnimanie: z=0 oznachaet "ne menyat vysotu" v sisteme 'body'
print("Dvizhenie vpered i vpravo...")
navigate(x=4.5, y=4.5, z=2, frame_id='body', speed=0.5)
rospy.sleep(10)

msg = rospy.wait_for_message('main_camera/image_raw', Image, timeout=5.0)
image = bridge.imgmsg_to_cv2(msg, 'bgr8')
cv2.imwrite('output.jpg', image)
rospy.sleep(10)
# Vozvrat v iskhodnuyu tochku (v sisteme 'body' eto otnositelnye koordinaty)
print("Vozvrat v iskhodnuyu tochku...")
navigate(x=-4.5, y=-4.5, z=-2, frame_id='body', speed=0.5)
rospy.sleep(10)

# Posadka
print("Posadka...")
land()
rospy.sleep(5)

print("Missiya zavershena")