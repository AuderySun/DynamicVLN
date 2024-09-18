import random

def is_intersection(waypoint):
    return len(waypoint.next(0.1)) > 1

import math

def is_turn(prev_wp, current_wp, next_wp, angle_threshold=10.0):
    # 计算前一个和当前路点之间的向量
    v1 = current_wp.transform.location - prev_wp.transform.location
    # 计算当前和下一个路点之间的向量
    v2 = next_wp.transform.location - current_wp.transform.location

    # 计算两个向量的角度（弧度）
    angle1 = math.atan2(v1.y, v1.x)
    angle2 = math.atan2(v2.y, v2.x)

    # 计算角度差（绝对值）
    angle_diff = abs(math.degrees(angle2 - angle1))

    # 将角度差调整到 [0, 360] 范围内
    if angle_diff > 180:
        angle_diff = 360 - angle_diff

    # 判断是否超过阈值
    return angle_diff >= angle_threshold


def generate_route_with_turns(start_wp, min_length, max_length, num_turns, angle_threshold=10.0):
    for _ in range(1000):  # 尝试最多 1000 次
        route = [start_wp]
        turn_count = 0
        current_wp = start_wp

        while len(route) < max_length:
            # 获取下一个路点
            next_wps = current_wp.next(2.0)
            if not next_wps:
                break  # 到达死路

            # 随机选择下一个路点
            current_wp = random.choice(next_wps)
            route.append(current_wp)

            # 如果有至少三个路点，检查是否发生转弯
            if len(route) >= 3:
                prev_wp = route[-3]
                curr_wp = route[-2]
                next_wp = route[-1]

                if is_turn(prev_wp, curr_wp, next_wp, angle_threshold):
                    turn_count += 1

            # 如果达到要求的转弯次数和最小长度，返回路线
            if len(route) >= min_length and turn_count >= num_turns:
                return route

        # 如果在 while 循环中未返回，检查是否满足条件
        if turn_count == num_turns and min_length <= len(route) <= max_length:
            return route  # 找到有效路线

    print("Could not find a route meeting the criteria.")
    return None



def spawn_obstacles_along_route(world, route, num_obstacles):
    obstacle_waypoints = random.sample(route[5:-5], num_obstacles)  # Avoid start and end points
    obstacles = []

    blueprint_library = world.get_blueprint_library()
    pedestrian_bps = list(blueprint_library.filter('walker.pedestrian.*'))
    vehicle_bps = list(blueprint_library.filter('vehicle.*.*'))
    obstacle_bps = pedestrian_bps + vehicle_bps

    for wp in obstacle_waypoints:
        obstacle_bp = random.choice(obstacle_bps)
        transform = wp.transform
        obstacle = world.try_spawn_actor(obstacle_bp, transform)
        if obstacle:
            obstacles.append(obstacle)
    return obstacles


