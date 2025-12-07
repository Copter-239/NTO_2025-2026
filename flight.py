import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

# Ожидание доступности сервисов
rospy.wait_for_service('get_telemetry')
rospy.wait_for_service('navigate')
rospy.wait_for_service('land')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def wait_arrival(tolerance=0.2, timeout=rospy.Duration(30)):
    """Ожидание достижения цели"""
    start = rospy.Time.now()
    while rospy.Time.now() - start < timeout:
        telem = get_telemetry(frame_id='navigate_target')
        if abs(telem.x) < tolerance and abs(telem.y) < tolerance and abs(telem.z) < tolerance:
            return True
        rospy.sleep(0.2)
    return False

# Взлёт на 2 метра
print("Взлёт на 2 метра...")
res = navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True)
if not res.success:
    print("Ошибка взлёта!")
    exit()

if wait_arrival():
    print("Достигнута высота 2 метра")
else:
    print("Таймаут достижения высоты!")

# Перемещение к точке (4.5, 4.5, 0)
print("Перемещение к (4.5, 4.5, 4)...")
res = navigate(x=4.5, y=4.5, z=0, frame_id='aruco_map')
if res.success:
    wait_arrival()

# Возврат к точке (0, 0, 2)
print("Возврат к (0, 0, 2)...")
res = navigate(x=0, y=0, z=2, frame_id='aruco_map')
if res.success:
    wait_arrival()

print("Миссия завершена")
