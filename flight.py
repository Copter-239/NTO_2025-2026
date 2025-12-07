import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

# Wait for services to become available
rospy.wait_for_service('get_telemetry')
rospy.wait_for_service('navigate')
rospy.wait_for_service('land')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def wait_arrival(tolerance=0.2, timeout=rospy.Duration(30)):
    """Wait for drone to reach target position"""
    start = rospy.Time.now()
    while rospy.Time.now() - start < timeout:
        telem = get_telemetry(frame_id='navigate_target')
        if abs(telem.x) < tolerance and abs(telem.y) < tolerance and abs(telem.z) < tolerance:
            return True
        rospy.sleep(0.2)
    return False

# Mission start
print("[INFO] Starting mission...")

# Takeoff to 2 meters
print("[INFO] Taking off to 2 meters...")
res = navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True)
if not res.success:
    print("[ERROR] Takeoff failed!")
    exit()

if wait_arrival():
    print("[INFO] Reached target altitude: 2m")
else:
    print("[WARNING] Timeout while waiting to reach altitude!")

# Navigate to point (4.5, 4.5, 0) in aruco_map frame
print("[INFO] Navigating to point (4.5, 4.5, 0)...")
res = navigate(x=4.5, y=4.5, z=0, frame_id='aruco_map')
if res.success:
    if wait_arrival():
        print("[INFO] Arrived at target position (4.5, 4.5, 0)")
    else:
        print("[WARNING] Timeout while navigating to (4.5, 4.5, 0)")
else:
    print("[ERROR] Navigation command failed!")

# Return to home position at (0, 0, 2)
print("[INFO] Returning to home position (0, 0, 2)...")
res = navigate(x=0, y=0, z=2, frame_id='aruco_map')
if res.success:
    if wait_arrival():
        print("[INFO] Arrived at home position (0, 0, 2)")
    else:
        print("[WARNING] Timeout while returning home")
else:
    print("[ERROR] Return command failed!")

print("[INFO] Mission completed successfully!")
