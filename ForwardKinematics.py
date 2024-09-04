import numpy as np
from BvhParser import Bvh, BvhNode
from quaternions import buildQuaternionRotation, quaternionMultiplication
from utils import _rotation_order

# array to store the bones for rendering
bones = np.zeros((24, 2, 4))
bone_counter = 0
# index for current frame data
pFrame =  0
# frame data
frame_data = []

def setJointInformationRecursively(joint: BvhNode, mocap: Bvh):
    # get joint offset
    joint.joint_offset = np.array(mocap.joint_offset(joint.name))
    # set joint offset as the initial local position
    joint.local_translation = np.append(joint.joint_offset, 0.)
    # get the channels for the joint (rotation and translation channels)
    joint.saved_channels = mocap.joint_channels(joint.name)
    # set rotation order for the joints
    joint.rotation_order = _rotation_order(joint.saved_channels)
    # set the info on the joint parent
    joint.parent = mocap.joint_parent(joint.name)

    # iterate on the joints of the children recursively
    for child in mocap.joint_direct_children(joint.name):
        setJointInformationRecursively(child, mocap)

# method to read a .bvh file 
def processBVH(filename):
    # read the mocap data
    with open(filename) as f:
        mocap = Bvh(f.read())

    #this contains all of the frames data.
    frames = np.array(mocap.frames).astype('float32')

    # recursively iterate over the joints and add relevant information
    setJointInformationRecursively(mocap.root.children[1], mocap)

    # return the root joint and the mocap data 
    return [mocap.root.children[1], mocap, frames]

# compute local rotation quaternion with specific rotation angles (angle1, angle2, angle3) and a euler rotation order
def computeLocalQuaternion(joint: BvhNode):
    
    # add/edit your code here for part 1
    
    # default return
    return np.array([0, 0, 0, 1])
    
# compute global rotation quaternion accumulated from root joint for a joint
def computeGlobalQuaternion(joint: BvhNode, local_quat: np.ndarray):

    # add/edit your code here for part 2

    # default return
    return local_quat

# based on global quaternion and local position, compute global position for a joint
def computeGlobalPosition(joint: BvhNode):

    # add/edit your code here for part 3

    # default return
    joint.global_position = joint.local_translation

    return joint.local_translation

# recursively calculate each joint's global position from the root (in-order traversal)
def calculateJointPosRecursivelyWithQuaternion(joint: BvhNode, mocap: Bvh, frame_no: int):
    global pFrame, frame_data, bones, bone_counter

    # check if joint is root. If yes, set the first three elements as root joint's global translation
    if (joint.parent == None):
        joint.local_translation = frame_data[:3]
        joint.local_translation = np.append(joint.local_translation, 0.)
        pFrame += 3 # pFrame points to joints' Euler angles by adding by 3.
    
    # ------------------------------coding part start------------------------------------------
	# Please modify the code accordingly inside these sub-functions                            
	# Coding Part: 1) calculate local rotation in quaternion from euler angles for current node
    local_quat = computeLocalQuaternion(joint)

	# Coding Part: 2) calculate global rotation quaternion for child nodes
    joint.global_quaternion = computeGlobalQuaternion(joint, local_quat)

	# Coding Part: 3) calculate current node's global position
    GlobalPosition = computeGlobalPosition(joint)
	# ------------------------------coding part end--------------------------------------------

    # calculate and store the bones into the global array
    if (joint.parent != None):
        bone = np.zeros((2, 4))
        start = joint.parent.global_position
        end = GlobalPosition
        bone[0] = start
        bone[1] = end
        bones[bone_counter] = bone
        bone_counter += 1
    
    # recursively call self for position computation
    children = mocap.joint_direct_children(joint.name)
    for child in children:
        calculateJointPosRecursivelyWithQuaternion(child, mocap, frame_no)

# clear global arrays and call calculateJointPosRecursivelyWithQuaternion
def calculateJointPos(skeleton, frame_no):
    global bones, pFrame, frame_data, bone_counter

    # clear global values
    bones = np.zeros((24, 2, 4))
    pFrame = 0
    bone_counter = 0
    
    # set frame data for current frame
    frame_data = skeleton[2][frame_no, :]

    # traverse down the entire skeleton tree and recursively calculate each joint's global position
    calculateJointPosRecursivelyWithQuaternion(skeleton[0], skeleton[1], frame_no)

    # return list of bones to render
    return bones
