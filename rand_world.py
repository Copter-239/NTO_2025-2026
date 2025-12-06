import math
import os
import random
from typing import List, Tuple


def split_polyline_to_points(polyline: List[Tuple[float, float]],
                             step_size: float = 0.02,
                             include_vertices: bool = True) -> List[Tuple[float, float]]:
    """
    Разбивает ломаную линию из 3 точек на промежуточные точки с заданным шагом.

    Args:
        polyline: Список из 3 точек в формате [(x1, y1), (x2, y2), (x3, y3)]
        step_size: Расстояние между точками (по умолчанию 0.02)
        include_vertices: Включать ли исходные вершины в результат

    Returns:
        Список точек вдоль ломаной с заданным шагом
    """
    if len(polyline) != 3:
        raise ValueError("Ломаная должна содержать ровно 3 точки")

    if step_size <= 0:
        raise ValueError("Шаг должен быть положительным числом")

    result_points = []

    # Функция для вычисления расстояния между точками
    def distance(p1, p2):
        return math.dist(p1, p2)

    # Обрабатываем каждый отрезок ломаной
    for i in range(len(polyline) - 1):
        start = polyline[i]
        end = polyline[i + 1]

        # Добавляем начальную точку отрезка (кроме второго и третьего отрезков, если не включаем вершины)
        if (i == 0 and include_vertices) or not include_vertices:
            result_points.append(start)

        # Вычисляем параметры отрезка
        seg_length = distance(start, end)
        if seg_length == 0:
            continue  # Пропускаем нулевые отрезки

        # Направляющий вектор отрезка
        dx = end[0] - start[0]
        dy = end[1] - start[1]

        # Количество шагов на этом отрезке
        num_steps = int(math.ceil(seg_length / step_size))

        # Генерируем промежуточные точки
        for step in range(1, num_steps + 1):
            # Параметр t от 0 до 1
            t = min(step * step_size / seg_length, 1.0)

            # Вычисляем координаты точки
            x = start[0] + t * dx
            y = start[1] + t * dy

            # Добавляем точку (проверяем, не совпадает ли с конечной)
            if step == num_steps and include_vertices and i < len(polyline) - 1:
                # Для последней точки на отрезке (кроме последнего отрезка)
                # Проверяем, не совпадает ли с началом следующего отрезка
                if i + 1 < len(polyline):
                    next_start = polyline[i + 1]
                    if abs(x - next_start[0]) > 1e-9 or abs(y - next_start[1]) > 1e-9:
                        result_points.append((x, y))
            else:
                result_points.append((x, y))

    # Добавляем последнюю точку ломаной
    if include_vertices:
        result_points.append(polyline[-1])

    # Удаляем дубликаты (с учетом точности)
    unique_points = []
    for point in result_points:
        if not any(abs(point[0] - p[0]) < 1e-9 and abs(point[1] - p[1]) < 1e-9
                   for p in unique_points):
            unique_points.append(point)

    return unique_points


def angle_between_lines(p1, p2, p3):
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    angle = math.atan2(v2[1] * v1[0] - v2[0] * v1[1], v2[0] * v1[0] + v2[1] * v1[1])
    angle = math.degrees(angle)
    if angle < 0:
        angle += 360
    return angle


def find_triangle_angles(p1, p2):
    # Вычисляем длины катетов
    y1, x1 = p1
    y2, x2 = p2
    dx = x2 - x1
    dy = y2 - y1
    angle1 = math.atan2(dy, dx)
    return angle1


def find_midpoint(point1, point2):
    """
    Находит среднюю точку между двумя точками в 2D-пространстве

    Args:
        point1: кортеж (x1, y1) - координаты первой точки
        point2: кортеж (x2, y2) - координаты второй точки

    Returns:
        tuple: координаты средней точки (x, y)
    """
    y1, x1 = point1
    y2, x2 = point2

    # Вычисляем средние координаты
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2

    return (mid_x, mid_y)


while True:
    p1 = [random.randint(0, 89) / 10, random.randint(0, 89) / 10]
    p2 = [random.randint(0, 89) / 10, random.randint(0, 89) / 10]
    angle = angle_between_lines((1, 1), p1, p2)
    if (1 < abs(angle) < 30 or 330 < abs(angle) < 360) and 5 <= math.dist((1, 1), p1) + math.dist(p1, p2) <= 10:
        break
allPointLine = split_polyline_to_points([(1, 1), p1, p2], 0.05, False)
points2 = []
for _ in range(5):
    while True:
        flag = True
        pv = [random.randint(0, 89) / 10, random.randint(0, 89) / 10]
        p = min(allPointLine, key=lambda p: math.dist(p, pv))
        if 1 < math.dist(p, pv) < 2 and math.dist(p, (1, 1)) > 0.2 and math.dist(p, p1) > 0.2 and math.dist(p, p2) > 0.2:
            for i in points2:
                if 0.75 > math.dist(p, i[0]):
                    flag = False
            if flag:
                break
    points2.append((p, pv))
dist1, dist2, angle1, angle2, midpoint1, midpoint2 = math.dist((1, 1), p1.__reversed__()), math.dist(
    p1.__reversed__(), p2.__reversed__()), find_triangle_angles((1, 1), p1), find_triangle_angles(p1, p2), find_midpoint((1, 1), p1), find_midpoint(
    p1, p2)
print(angle, p1, p2, dist1, dist2, angle1, angle2, midpoint1, midpoint2)
print(' '.join(map(str, midpoint1)))
print(' '.join(map(str, midpoint2)))
print(pv, p)
if __name__ != '__main__':
    with open('clover_aruco.world') as f:
        world = f.read()
    world = world.replace('__POSE1__', ' '.join(map(str, midpoint1)))
    world = world.replace('__POSE2__', ' '.join(map(str, midpoint2)))
    world = world.replace('__ANGLE1__', str(angle1))
    world = world.replace('__ANGLE2__', str(angle2))
    # for i in range(len(points2)):
    #     world = world.replace(f'__POSE{i+3}__', ' '.join(map(str, find_midpoint(points2[i][0], points2[i][1]))))
    #     world = world.replace(f'__ANGLE{i+3}__', str(find_triangle_angles(points2[i][0], points2[i][1])))
    with open('/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world', 'w') as f:
        f.write(world)

    with open('main.sdf') as f:
        main = f.read()
    main = main.replace('__LENGTH__', str(dist1))
    with open('main/main.sdf', 'w') as f:
        f.write(main)
    os.system('rm -rf /home/clover/catkin_ws/src/clover/clover_simulation/models/main')
    os.system('cp -r main /home/clover/catkin_ws/src/clover/clover_simulation/models/main/')

    with open('main.sdf') as f:
        main = f.read()
    main = main.replace('__LENGTH__', str(dist2))
    with open('main/main.sdf', 'w') as f:
        f.write(main)
    os.system('rm -rf /home/clover/catkin_ws/src/clover/clover_simulation/models/main2')
    os.system('cp -r main /home/clover/catkin_ws/src/clover/clover_simulation/models/main2/')
    # for i in range(len(points2)):
    #     with open('sidebar.sdf') as f:
    #         sidebar = f.read()
    #     sidebar = sidebar.replace('__LENGTH__', str(math.dist(points2[i][0], points2[i][1])))
    #     with open('sidebars/sidebar.sdf', 'w') as f:
    #         f.write(sidebar)
    #     os.system(f'cp -r sidebars /home/clover/catkin_ws/src/clover/clover_simulation/models/sidebar{i+1}/')

else:
    import cv2
    import numpy as np

    m = 50
    img = np.zeros((10 * m, 10 * m, 3), np.uint8)
    img = cv2.line(img, (m, m), tuple(map(lambda x: int(x * m), p1)), (255, 0, 0), 2)
    img = cv2.line(img, tuple(map(lambda x: int(x * m), p1)), tuple(map(lambda x: int(x * m), p2)), (255, 0, 0), 2)
    for i in range(len(points2)):
        img = cv2.line(img, tuple(map(lambda x: int(x * m), points2[i][0])),
                       tuple(map(lambda x: int(x * m), points2[i][1])), (0, 255, 0), 2)
    cv2.imshow('img', img)
    cv2.waitKey(0)
