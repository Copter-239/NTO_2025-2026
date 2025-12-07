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

def wait_for_position_estimate(timeout=10.0):
    """Wait for drone to get position estimate in aruco_map frame"""
    rospy.loginfo("Waiting for position estimate in aruco_map...")
    start_time = rospy.Time.now().to_sec()
    
    while rospy.Time.now().to_sec() - start_time < timeout:
        telem = get_telemetry(frame_id='aruco_map')
        
        # Check if we have a valid position estimate (not all zeros or NaNs)
        position_valid = (
            not math.isnan(telem.x) and
            not math.isnan(telem.y) and
            not math.isnan(telem.z) and
            (abs(telem.x) > 0.01 or abs(telem.y) > 0.01 or abs(telem.z) > 0.01)
        )
        
        if position_valid:
            rospy.loginfo(f"Position estimate obtained: ({telem.x:.2f}, {telem.y:.2f}, {telem.z:.2f})")
            return True
        
        rospy.loginfo(f"Waiting for position estimate... Current: ({telem.x:.2f}, {telem.y:.2f}, {telem.z:.2f})")
        rospy.sleep(1.0)
    
    rospy.logerr(f"Failed to get position estimate within {timeout} seconds")
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

# Wait for position estimate in aruco_map frame
if not wait_for_position_estimate(timeout=15.0):
    rospy.logwarn("Proceeding with body frame navigation instead...")
    
    # If we can't get position estimate, use body frame for relative navigation
    rospy.loginfo("Phase 2: Navigation using body frame")
    rospy.loginfo("Flying forward and right by 4.5m each...")
    
    # Navigate relative to current position
    res = navigate(x=4.5, y=4.5, z=0, frame_id='body')
    
    if res.success:
        if wait_arrival(tolerance=0.5, timeout=45.0):
            rospy.loginfo("Relative navigation successful")
        else:
            rospy.logwarn("Navigation timeout")
    else:
        rospy.logerr("Body frame navigation command rejected!")
else:
    # Navigate to point (4.5, 4.5, 0) in aruco_map frame
    rospy.loginfo("\nPhase 2: Navigation to target in aruco_map")
    rospy.loginfo("Flying to coordinates (4.5, 4.5, 0) in aruco_map frame...")
    res = navigate(x=4.5, y=4.5, z=0, frame_id='aruco_map')

    if res.success:
        if wait_arrival(tolerance=0.3, timeout=45.0):
            rospy.loginfo("Target position reached successfully")
        else:
            rospy.logwarn("Navigation timeout. Proceeding to next waypoint.")
    else:
        rospy.logerr("Navigation command rejected! Trying body frame...")
        # Fallback to body frame
        res = navigate(x=4.5, y=4.5, z=0, frame_id='body')
        if res.success:
            wait_arrival(tolerance=0.5, timeout=45.0)

# Return to home
rospy.loginfo("\nPhase 3: Return to home")
rospy.loginfo("Returning to home position (0, 0, 2)...")

# Try to return using aruco_map if we have position, otherwise use body frame
telem = get_telemetry(frame_id='aruco_map')
position_valid = not math.isnan(telem.x) and not math.isnan(telem.y) and not math.isnan(telem.z)

if position_valid:
    res = navigate(x=0, y=0, z=2, frame_id='aruco_map')
    frame_used = "aruco_map"
else:
    res = navigate(x=0, y=0, z=2, frame_id='body')
    frame_used = "body"

rospy.loginfo(f"Return command using {frame_used} frame")

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

# Landing option
rospy.loginfo("Initiating automatic landing...")
land_result = land()
if land_result.success:
    rospy.loginfo("Landing sequence started")
    rospy.sleep(5)  # Wait for landing to complete
else:
    rospy.logerr("Landing command failed!")
