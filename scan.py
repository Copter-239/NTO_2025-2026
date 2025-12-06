import math
from typing import List, Tuple

import cv2
import numpy as np


def get_intermediate_points_count(p1: Tuple[float, float], p2: Tuple[float, float], num_points: int) -> List[
    Tuple[float, float]]:
    """
    Генерирует заданное количество промежуточных точек между p1 и p2 с использованием
    линейной интерполяции.

    Args:
        p1: Начальная точка (x1, y1).
        p2: Конечная точка (x2, y2).
        num_points: Общее количество промежуточных точек, которые нужно сгенерировать
                    (не включая p1 и p2).

    Returns:
        Список промежуточных точек.
    """
    if num_points <= 0:
        return []

    x1, y1 = p1
    x2, y2 = p2
    points = []

    # Мы делим путь на num_points + 1 сегментов, чтобы получить нужное количество точек между ними
    for i in range(1, num_points + 1):
        # Коэффициент интерполяции (t) меняется от 0 до 1
        t = i / (num_points + 1)

        # Линейная интерполяция по формуле: P(t) = P1 + t * (P2 - P1)
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)

        # Округляем до целых чисел, если работаем с пикселями
        points.append((int(round(x)), int(round(y))))
        # Если нужна точность float, используйте: points.append((x, y))

    return points


def rotate_points(points: List[Tuple[float, float]], transformation_matrix: np.ndarray) -> List[Tuple[float, float]]:
    """
    Поворачивает список 2D точек с использованием заданной матрицы аффинного преобразования.

    Args:
        points: Исходный список точек в формате [(x1, y1), (x2, y2), ...].
        transformation_matrix: Матрица преобразования OpenCV/NumPy (например,
                               полученная из cv2.getRotationMatrix2D()).

    Returns:
        Список повернутых точек в формате [(x1_rot, y1_rot), (x2_rot, y2_rot), ...].
    """
    if not points:
        return []

    # 1. Преобразовать входной список точек в массив NumPy типа float32.
    #    Формат: [N точек, 2 координаты (x, y)]
    points_np = np.float32(points)

    # 2. Изменить форму массива для совместимости с cv2.transform.
    #    Формат: [N точек, 1 (для совместимости), 2 координаты (x, y)]
    #    cv2.transform ожидает именно такой формат.
    points_reshaped = points_np.reshape(-1, 1, 2)

    # 3. Применить аффинное преобразование
    rotated_points_np = cv2.transform(points_reshaped, transformation_matrix)

    # 4. Изменить форму результата обратно в удобный формат [N точек, 2 координаты (x, y)]
    rotated_points_flat = rotated_points_np.reshape(-1, 2)

    # 5. Преобразовать массив NumPy обратно в стандартный Python list of tuples
    #    Мы используем .astype(int) если нужны целые пиксельные координаты
    #    или оставляем float если нужна точность.
    rotated_points_list = rotated_points_flat.astype(int).tolist()
    # ^^^ Замените .astype(int) на .tolist(), если хотите сохранить дробные координаты

    return rotated_points_list


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


def sum_point(args):
    point_sum = list(args[0])
    for point in args[1:]:
        point_sum[0] += point[0]
        point_sum[1] += point[1]
    return (int(point_sum[0] / len(args)), int(point_sum[1] / len(args)))


detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100),
                                   cv2.aruco.DetectorParameters())


def find_triangle_angles(cord1, cord2):
    # Вычисляем длины катетов
    dx = cord2[0] - cord1[0]
    dy = cord2[1] - cord1[1]
    angle1 = math.degrees(math.atan2(dy, dx))  # Угол при точке (x1, y1)
    return angle1


def detect_aruco_maps(frame, output_frame=False, is_id=None):
    itog = {'aruco_map': {}}
    corners, ids, rejected = detector.detectMarkers(frame)
    if ids is not None:
        for i in range(len(ids)):
            if is_id is None or int(ids[i][0]) in is_id:
                marker_corners = corners[i][0]
                center = tuple(map(int, marker_corners.mean(axis=0)))
                tc = tuple((
                    int((int(marker_corners[1][0]) + int(marker_corners[0][0])) / 2),
                    int(((int(marker_corners[1][1]) + int(marker_corners[0][1])) / 2))))
                itog['aruco_map'][int(ids[i][0])] = (
                    {'center_point': center, 'id': int(ids[i][0]), 'top_center_point': tc})
                if output_frame:
                    # cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.circle(frame, center, 5, (0, 255, 0), -1)
                    cv2.circle(frame, tc, 5, (255, 0, 0), -1)
    if output_frame:
        itog['frame'] = frame
    return itog


img = cv2.imread(r"C:\Users\user\Downloads\Telegram Desktop\photo_2025-12-04_16-44-05.jpg")
data = detect_aruco_maps(img.copy(), True, (
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
    0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
    9, 19, 29, 39, 49, 59, 69, 79, 89, 99))
img_aruco = data['frame']
a = []
for i in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9):
    if i in data['aruco_map'] and i + 90 in data['aruco_map']:
        a.append(
            find_triangle_angles(data['aruco_map'][i]['center_point'], data['aruco_map'][i + 90]['center_point']))
    else:
        pass
if len(a) > 0:
    a = sum(a) / len(a)
    # print(a)

    # print(len(data['aruco_map']))

    h, w = img.shape[:2]
    data = detect_aruco_maps(img.copy(), True, (
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9))
    data1 = detect_aruco_maps(img.copy(), False, (
        90, 91, 92, 93, 94, 95, 96, 97, 98, 99))
    data2 = detect_aruco_maps(img.copy(), False, (
        0, 10, 20, 30, 40, 50, 60, 70, 80, 90))
    data3 = detect_aruco_maps(img.copy(), False, (
        9, 19, 29, 39, 49, 59, 69, 79, 89, 99))
    data = list(map(lambda x: x['center_point'], data['aruco_map'].values()))
    data1 = list(map(lambda x: x['center_point'], data1['aruco_map'].values()))
    data2 = list(map(lambda x: x['center_point'], data2['aruco_map'].values()))
    data3 = list(map(lambda x: x['center_point'], data3['aruco_map'].values()))
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, a, 1.0)
    data = sum_point(rotate_points(data, M))
    data1 = sum_point(rotate_points(data1, M))
    data2 = sum_point(rotate_points(data2, M))
    data3 = sum_point(rotate_points(data3, M))

    # Применить аффинное преобразование
    img = cv2.warpAffine(img, M, (w, h))
    p = (data1[0] + data[0] + data3[1] + data2[1] / 4) * 0.025
    # img = cv2.circle(img, (data[0], data3[1]), 2, (0, 255, 255), -1)
    # img = cv2.circle(img, (data1[0], data2[1]), 2, (0, 255, 255), -1)
    # img = cv2.circle(img, center, 2, (0, 255, 0), -1)
    img_obr = img.copy()
    img_obr = img_obr[int(data3[1] - p):int(data2[1] + p), int(data[0] - p):int(data1[0] + p)]
    img_obr = cv2.resize(img_obr, (500, 500), interpolation=cv2.INTER_LINEAR)
    bw = cv2.inRange(img_obr, (0, 0, 161), (255, 120, 255))
    contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=cv2.contourArea)
    con = np.zeros(img_obr.shape, np.uint8)
    cv2.drawContours(con, contours, -1, (0, 255, 0), 1)
    perimeter = cv2.arcLength(contours[-1], True)
    epsilon = perimeter * 0.07
    epsilon2 = perimeter * 0.0249
    epsilon3 = perimeter * 0.0035
    approx = cv2.approxPolyDP(contours[-1], epsilon, True)
    approx2 = cv2.approxPolyDP(contours[-1], epsilon2, True)
    approx3 = cv2.approxPolyDP(contours[-1], epsilon3, True)
    n = np.zeros(img_obr.shape, np.uint8)
    bw = np.zeros(img_obr.shape, np.uint8)
    cv2.drawContours(n, [approx2], -1, (0, 255, 0), 1)
    cv2.drawContours(n, [approx], -1, (0, 0, 255), 1)
    cv2.drawContours(bw, [approx3], -1, (255, 255, 255), -1)
    cv2.imshow('contours', n)
    cv2.imshow('img_obr', img_obr)
    cv2.waitKey(0)
    dist = math.dist(approx[0][0].tolist(), approx[1][0].tolist())

    points = get_intermediate_points_count(approx[0][0].tolist(), approx[1][0].tolist(), int(dist / 0.02))

    total = approx2.tolist()
    for p in points:
        for i in range(len(total)).__reversed__():
            d = math.dist(total[i][0], p)
            if d < 40:
                total.pop(i)
    for i in range(len(approx3)).__reversed__():
        for t in total:
            if math.dist(approx3[i][0].tolist(), t[0]) < 5:
                # print(approx3[i][0].tolist())
                contour_list = [point[0].tolist() for point in approx3]
                contour_list.pop(i)
                approx3 = np.array(contour_list, dtype=np.int32).reshape(-1, 1, 2)

    temp = np.zeros(bw.shape, np.uint8)
    cv2.drawContours(temp, [approx3], -1, (255, 255, 255), -1)
    perimeter = cv2.arcLength(approx3, True)
    epsilon = perimeter * 0.015
    approx4 = cv2.approxPolyDP(approx3, epsilon, True)
    for i in range(len(approx4)).__reversed__():
        for t in approx:
            if math.dist(approx4[i][0].tolist(), t[0]) < 5:
                # print(approx3[i][0].tolist())
                contour_list = [point[0].tolist() for point in approx4]
                contour_list.pop(i)
                approx4 = np.array(contour_list, dtype=np.int32).reshape(-1, 1, 2)

    contour_list = [point[0].tolist() for point in approx]
    contour_list.append(sum_point([point[0].tolist() for point in approx4]))
    contour_list.append(contour_list.pop(1))
    line_points = split_polyline_to_points(contour_list ,step_size=0.3)
    # for i in range(len(line_points)).__reversed__():
    #     for t in contour_list:
    #         if math.dist(t, line_points[i]) < 14:
    #             line_points.pop(i)
    approx3 = np.array(contour_list, dtype=np.int32).reshape(-1, 1, 2)
    line_points_total = []
    for p in total:
        n = cv2.circle(n, p[0], 5, (255, 0, 0), -1)
        img_obr = cv2.circle(img_obr, p[0], 5, (255, 0, 0), -1)
        line = min(line_points, key=lambda pv: math.dist(p[0], pv))
        img_obr = cv2.line(img_obr, p[0], (int(line[0]), int(line[1])), (255, 0, 0), 2)
        line_points_total.append((int(line[0]), int(line[1])))
        cv2.drawContours(img_obr, [approx3], -1, (0, 255, 0), 2)
    cv2.imshow('contours', n)
    cv2.imshow('img_obr', img_obr)
    cv2.waitKey(0)
    # p = min(line_points, key=lambda pv: math.dist(p, pv))
    cv2.imshow('temp', temp)
    cv2.imshow('bw', bw)
    cv2.waitKey(0)
    bw = cv2.bitwise_xor(temp, bw)
    for a in approx3:
        # print(a[0])
        temp = cv2.circle(temp, a[0], 5, (255, 0, 0), -1)
    cv2.imshow('temp', temp)
    cv2.imshow('bw', bw)
    cv2.waitKey(0)
    print(line_points_total)
