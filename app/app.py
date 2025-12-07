import rospy
from clover import srv, long_callback
from mavros_msgs.srv import CommandBool
from std_msgs.msg import UInt16MultiArray, Bool
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import base64
import cv2
from subprocess import run
from multiprocessing import Process
from flask import Flask, render_template

bridge = CvBridge()
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land_dron = rospy.ServiceProxy('land', Trigger)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
start_bool = rospy.Publisher('bool_start', Bool, queue_size=1)
rospy.init_node('server_web_ui')
app = Flask(__name__)


@long_callback
def image_callback(data):
    global img_now_down_cam
    img_now_down_cam = bridge.imgmsg_to_cv2(data, 'bgr8')
# cap = cv2.VideoCapture(0)
# def get_data_img_cam_PC():
#     global img_now_down_cam
#     while True:
#         _, img_now_down_cam = cap.read()
#         cv2.waitKey(10)
def img_to_base64_rgb(img):
    # Конвертируем BGR → RGB
    # img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Кодируем
    _, buffer = cv2.imencode('.jpg', img)
    base64_str = base64.b64encode(buffer).decode('utf-8')
    return f"data:image/jpeg;base64,{base64_str}"


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/start')
def start():
    bool_start_or_stop_callback(True)
    return 'ok', 200


@app.route('/stop')
def stop():
    bool_start_or_stop_callback(False)
    navigate(x=0, y=0, z=0, frame_id='body')
    land_dron()
    return 'ok', 200


@app.route('/kill')
def kill():
    navigate(x=0, y=0, z=0, frame_id='body')
    bool_start_or_stop_callback(False)
    arming(False)
    return 'ok', 200


@app.route('/img_down_cam')
def img_down_cam():
    return img_to_base64_rgb(img_now_down_cam)
    return "NOne"


rospy.Subscriber('main_camera/image_raw', Image, image_callback)
# f = Thread(target=get_data_img_cam_PC)
# f.start()
is_starting_process_running_flight = False



def start_flight():
    return run(["python3", r"flight.py"])


process_running_flight = Process(target=start_flight)

def bool_start_or_stop_callback(msg):
    global process_running_flight, is_starting_process_running_flight
    if msg:
        if not is_starting_process_running_flight:
            is_starting_process_running_flight = True
            print('start')
            process_running_flight.start()
    else:
        if is_starting_process_running_flight:
            is_starting_process_running_flight = False
            print(stop)
            process_running_flight.terminate()
            process_running_flight.join()


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)



