import rospy
from clover import srv
from std_srvs.srv import Trigger
import math

rospy.init_node('flight')

# Wait for services to become available
rospy.loginfo("Waiting for ROS services...")
rospy.wait_for_service('get_telemetry')
rospy.wait_for_service('navigate')
rospy.wait_for_service('land')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def wait_arrival(tolerance=0.2, timeout=30.0):
    """Wait for drone to reach target position with progress logging"""
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:
        telem = get_telemetry(frame_id='navigate_target')
        
        # Calculate distance to target
        distance = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
        
        # Log progress every 2 seconds
        if int(rospy.Time.now().to_sec() - start_time) % 2 == 0:
            rospy.loginfo(f"Distance to target: {distance:.2f}m | Position: ({telem.x:.2f}, {telem.y:.2f}, {telem.z:.2f})")
        
        if abs(telem.x) < tolerance and abs(telem.y) < tolerance and abs(telem.z) < tolerance:
            rospy.loginfo("Target reached within tolerance")
            return True
        
        rospy.sleep(0.2)
    
    rospy.logwarn(f"Timeout reached after {timeout} seconds")
    return False

# Mission start
rospy.loginfo("=" * 50)
rospy.loginfo("Starting autonomous mission")
rospy.loginfo("=" * 50)

# Takeoff to 2 meters
rospy.loginfo("Phase 1: Takeoff")
rospy.loginfo("Arming motors and taking off to 2m altitude...")
res = navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True)

if not res.success:
    rospy.logerr("Takeoff command failed! Check motors and safety switches.")
    exit()

rospy.loginfo("Takeoff command accepted. Waiting to reach altitude...")

if wait_arrival():
    rospy.loginfo("Takeoff successful. Current altitude: 2m")
else:
    rospy.logwarn("Takeoff timeout, but continuing mission...")

# Navigate to point (4.5, 4.5, 0)
rospy.loginfo("\nPhase 2: Navigation to target")
rospy.loginfo("Flying to coordinates (4.5, 4.5, 0) in aruco_map frame...")
res = navigate(x=4.5, y=4.5, z=0, frame_id='aruco_map')

if res.success:
    if wait_arrival(tolerance=0.3, timeout=45.0):
        rospy.loginfo("Target position reached successfully")
    else:
        rospy.logwarn("Navigation timeout. Proceeding to next waypoint.")
else:
    rospy.logerr("Navigation command rejected!")

# Return to home
rospy.loginfo("\nPhase 3: Return to home")
rospy.loginfo("Returning to home position (0, 0, 2)...")
res = navigate(x=0, y=0, z=2, frame_id='aruco_map')

if res.success:
    if wait_arrival():
        rospy.loginfo("Successfully returned to home position")
    else:
        rospy.logwarn("Return timeout, but mission complete.")
else:
    rospy.logerr("Return command rejected!")

# Mission complete
rospy.loginfo("\n" + "=" * 50)
rospy.loginfo("Mission sequence completed")
rospy.loginfo("Ready for landing command or next instructions")
rospy.loginfo("=" * 50)

# Optional: Add automatic landing
# rospy.loginfo("Initiating automatic landing...")
# land_result = land()
# if land_result.success:
#     rospy.loginfo("✓ Landing sequence started")
# else:
#     rospy.logerr("Landing command failed!")

