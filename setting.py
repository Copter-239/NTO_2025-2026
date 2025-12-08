import os
open('/home/clover/catkin_ws/src/clover/clover/launch/aruco.launch','w').write(open('launch/aruco.launch','r').read())
open('/home/clover/catkin_ws/src/clover/clover/launch/clover.launch','w').write(open('launch/clover.launch','r').read())
open('/home/clover/catkin_ws/src/clover/clover_simulation/models/camera/camera.sdf','w').write(open('camera.sdf','r').read())
os.system('pip install --upgrade flask')





