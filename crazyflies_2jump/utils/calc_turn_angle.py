def calc_turn_angle(robot_angle, target_angle):
    """
    根据当前角度和目标角度，计算最优旋转方向（左转/右转）及角度差。
    所有角度均以度为单位，范围[0, 360)
    """
    # 归一化
    robot_angle %= 360
    target_angle %= 360

    # 差值（目标 - 当前）
    diff = (target_angle - robot_angle) % 360

    # 如果差值在0~180，说明左转更近，否则右转更近
    if diff <= 180:
        direction = "left"
        delta = diff
    else:
        direction = "right"
        delta = 360 - diff

    return direction, delta
