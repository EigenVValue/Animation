import numpy as np

def buildQuaternionRotation(angle: float, x: float, y: float, z: float):
    '''return a quaternion with a rotaion: angle='angle', axis=(x,y,z)'''
    
    degreetorad = np.pi / 180.0
    c = np.cos((angle / 2.0) * degreetorad)    # cosine
    s = np.sin((angle / 2.0) * degreetorad)      # sine

    rotationQuat = [s * x, s * y, s * z, c]
    return rotationQuat

def quaternionMultiplication(quat1, quat2):
    '''calculate quaternion quat1*quat2'''

    v1 = np.array([quat1[0], quat1[1], quat1[2]])
    v2 = np.array([quat2[0], quat2[1], quat2[2]])

    w1 = quat1[3]
    w2 = quat2[3]

    v3 = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    w3 = w1 * w2 - np.dot(v1, v2)
    
    result = np.array([v3[0], v3[1], v3[2], w3])

    return result