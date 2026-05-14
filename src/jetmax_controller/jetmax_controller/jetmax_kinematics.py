import math

# Rangos permitidos (grados)
AngleRotateRange = (0, 240)
AngleLeftRange = (0, 180)
AngleRightRange = (-20, 160)

# Longitudes del robot (mm)
L0 = 84.4
L1 = 8.14
L2 = 128.41
L3 = 138.0


def forward_kinematics(angle):
    """
    Jetmax forward kinematics
    :param angle: active angles [rotate, left, right] en grados
    :return: lista de 9 ángulos articulares en grados
    """

    alpha1, alpha2, alpha3 = angle

    if not (AngleRotateRange[0] <= alpha1 <= AngleRotateRange[1]):
        return None
    if not (AngleLeftRange[0] <= alpha2 <= AngleLeftRange[1]):
        return None
    if not (AngleRightRange[0] <= alpha3 <= AngleRightRange[1]):
        return None

    alpha3 = -alpha3

    joint_angle = [0.0] * 9

    # 3 joints activos
    joint_angle[0] = alpha1 - 90        # Joint1
    joint_angle[1] = 90 - alpha2        # Joint2
    joint_angle[5] = alpha3             # Joint6

    # 6 joints pasivos (visualización)
    joint_angle[2] = 90 - (alpha2 + alpha3)
    joint_angle[3] = 90 - alpha2
    joint_angle[4] = joint_angle[1]
    joint_angle[6] = joint_angle[2]
    joint_angle[7] = alpha3
    joint_angle[8] = alpha3

    return joint_angle


def inverse_kinematics(position):
    """
    Jetmax inverse kinematics
    :param position: (x, y, z) en mm
    :return: (pos1, pos2, pos3) en grados o None
    """

    x, y, z = position
    y = -y

    # Cálculo theta1
    if abs(y) < 1e-6:
        if x < 0:
            theta1 = -90
        elif x > 0:
            theta1 = 90
        else:
            return None
    else:
        theta1 = math.degrees(math.atan2(x, y))

    r = math.sqrt(x * x + y * y) - L1
    z = z - L0

    dist = math.sqrt(r * r + z * z)

    if dist > (L2 + L3) or dist < abs(L2 - L3):
        return None

    alpha = math.degrees(math.atan2(z, r))

    beta = math.degrees(
        math.acos((L2**2 + L3**2 - dist**2) / (2 * L2 * L3))
    )

    gamma = math.degrees(
        math.acos((L2**2 - L3**2 + dist**2) / (2 * L2 * dist))
    )

    pos1 = theta1 + 90

    theta2 = alpha + gamma
    pos2 = 180 - theta2

    theta3 = beta + theta2
    pos3 = 180 - theta3

    return pos1, pos2, pos3