# import rospy
# from clover import srv, long_callback
# from mavros_msgs.srv import CommandBool
# from std_msgs.msg import UInt16MultiArray, Bool
# from std_srvs.srv import Trigger
from flask import Flask, request, render_template
import base64, cv2
from threading import Thread
# arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
# land_dron = rospy.ServiceProxy('land', Trigger)
# navigate = rospy.ServiceProxy('navigate', srv.Navigate)
# start_bool = rospy.Publisher('bool_start', Bool, queue_size=1)
# rospy.init_node('server_web_ui')
app = Flask(__name__)
def msg_convert(data, obj):
    msg = obj()
    msg.data = data
    return msg
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# bridge = CvBridge()
# @long_callback
# def image_callback(data):
#     global img_now_down_cam
#     img_now_down_cam = bridge.imgmsg_to_cv2(data, 'bgr8')\
cap = cv2.VideoCapture(0)
def get_data_img_cam_PC():
    global img_now_down_cam
    while True:
        _, img_now_down_cam = cap.read()
        cv2.waitKey(10)
def img_to_base64_rgb(img):
    # Конвертируем BGR → RGB
    #img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Кодируем
    _, buffer = cv2.imencode('.jpg', img)
    base64_str = base64.b64encode(buffer).decode('utf-8')
    return f"data:image/jpeg;base64,{base64_str}"
@app.route('/')
def index():
    return render_template('index.html')
@app.route('/start')
def start():
    # start_bool.publish(msg_convert(True, Bool))
    return 'ok', 200
@app.route('/stop')
def stop():
    # start_bool.publish(msg_convert(False, Bool))
    # navigate(x=0, y=0, z=0, frame_id='body')
    # land_dron()
    return 'ok', 200
@app.route('/kill')
def kill():
    # navigate(x=0, y=0, z=0, frame_id='body')
    # start_bool.publish(msg_convert(False, Bool))
    # arming(False)
    return 'ok', 200
@app.route('/img_down_cam')
def img_down_cam():
    return img_to_base64_rgb(img_now_down_cam)
# rospy.Subscriber('main_camera/image_raw', Image, image_callback)
f = Thread(target=get_data_img_cam_PC)
f.start()


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)


