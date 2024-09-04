import numpy as np

# convert quaternion matrix to rotation
def quaternion_to_rotation(quaternion):
    '''
    The input quaternion is of the format [w, x, y, z] and the output
    is a 3X3 rotation matrix corresponding to the input quaternion.
    '''
    rotation = np.zeros((3, 3))
    q_0 = quaternion[0]
    q_1 = quaternion[1]
    q_2 = quaternion[2]
    q_3 = quaternion[3]

    rotation[0, 0] = 1 - (2 * q_2**2) - (2 * q_3**2)
    rotation[0, 1] = (2 * q_1 * q_2) - (2 * q_0 * q_3)
    rotation[0, 2] = (2 * q_1 * q_3) + (2 * q_0 * q_2)
    rotation[1, 0] = (2 * q_1 * q_2) + (2 * q_0 * q_3)
    rotation[1, 1] = 1 - (2 * q_1**2) - (2 * q_3**2)
    rotation[1, 2] = (2 * q_2 * q_3) - (2 * q_0 * q_1)
    rotation[2, 0] = (2 * q_1 * q_3) - (2 * q_0 * q_2)
    rotation[2, 1] = (2 * q_2 * q_3) + (2 * q_0 * q_1)
    rotation[2, 2] = 1 - (2 * q_1**2) - (2 * q_2**2)

    return rotation

# operator for quaternion multiplication
def quaternion_multiply(quaternion1, quaternion2):
    '''
    Input are two quaternions of the format [w, x, y, z] and the output
    is the multiplication of the two quats and output is also of the same
    format as [w, x, y, z]
    '''
    w1, x1, y1, z1 = quaternion1
    w2, x2, y2, z2 = quaternion2

    v1 = np.array([x1, y1, z1])
    v2 = np.array([x2, y2, z2])

    v3 = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    w3 = w1 * w2 - np.dot(v1, v2)
    
    result = np.array([w3, v3[0], v3[1], v3[2]])

    return result

# normalize the quaternion
def normalize_quaternion(quaternion):
    '''
    Input is quaternion of format [w, x, y, z]
    Output is normalized quaternion of the format [w, x, y, z]
    '''
    quaternion = quaternion.flatten()
    mod_q = round(np.sqrt(quaternion[0]**2 + quaternion[1]**2 + quaternion[2]**2 + quaternion[3]**2), 2)
    
    if mod_q == 0:
        return quaternion 
    else: 
        return quaternion / mod_q
