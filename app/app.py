import rospy
from clover import srv
from mavros_msgs.srv import CommandBool
from std_msgs.msg import UInt16MultiArray, Bool
from std_srvs.srv import Trigger
from flask import Flask, request, render_template
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land_dron = rospy.ServiceProxy('land', Trigger)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
start_bool = rospy.Publisher('bool_start', Bool, queue_size=1)
rospy.init_node('flight')
app = Flask(__name__)
def msg_convert(data, obj):
    msg = obj()
    msg.data = data
    return msg
@app.route('/')
def index():
    return render_template('index.html')
@app.route('/start')
def start():
    start_bool.publish(msg_convert(True, Bool))
    return 'ok', 200
@app.route('/stop')
def stop():
    start_bool.publish(msg_convert(False, Bool))
    navigate(x=0, y=0, z=0, frame_id='body')
    land_dron()
    return 'ok', 200
@app.route('/kill')
def kill():
    navigate(x=0, y=0, z=0, frame_id='body')
    start_bool.publish(msg_convert(False, Bool))
    arming(False)
    return 'ok', 200
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

