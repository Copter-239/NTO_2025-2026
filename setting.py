import os
open('/home/clover/catkin_ws/src/clover/clover/launch/aruco.launch','w').write(open('launch/aruco.launch','r').read())
open('/home/clover/catkin_ws/src/clover/clover/launch/clover.launch','w').write(open('launch/clover.launch','r').read())
os.system('mv data/model/* /home/clover/catkin_ws/src/clover/clover_simulation/models/')
os.system('pip install --upgrade flask')




