'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy import sin, cos, pi, deg2rad
from numpy.matlib import matrix, identity

from recognize_posture import PostureRecognitionAgent

def Rx(angle_rad):
    c = cos(angle_rad)
    s = sin(angle_rad)
    return matrix([[1, 0, 0, 0],
                   [0, c, -s, 0],
                   [0, s,  c, 0],
                   [0, 0,  0, 1]], dtype=float)

def Ry(angle_rad):
    c = cos(angle_rad)
    s = sin(angle_rad)
    return matrix([[c, 0, s, 0],
                   [0, 1, 0, 0],
                   [-s,0, c, 0],
                   [0, 0, 0, 1]], dtype=float)

def Rz(angle_rad):
    c = cos(angle_rad)
    s = sin(angle_rad)
    return matrix([[c, -s,0, 0],
                   [s,  c, 0, 0],
                   [0, 0,  1, 0],
                   [0, 0,  0, 1]], dtype=float)

def Trans(x, y, z):
    return matrix([[1, 0, 0, x],
                   [0, 1, 0, y],
                   [0, 0, 1, z],
                   [0, 0, 0, 1]], dtype=float)

NECK_OFFSET_Z = 0.1265
SHOULDER_OFFSET_Y = 0.098
SHOULDER_OFFSET_Z = 0.100
UPPER_ARM_LENGTH = 0.105
ELBOW_OFFSET_Y = 0.015
LOWER_ARM_LENGTH = 0.05595
HAND_OFFSET_X = 0.05775
HAND_OFFSET_Z = 0.01231

HIP_OFFSET_Y = 0.050
HIP_OFFSET_Z = 0.085
THIGH_LENGTH = 0.100
TIBIA_LENGTH = 0.10290
FOOT_HEIGHT = 0.04519


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        
        self.joint_names.extend(['LFoot', 'RFoot', 'LArmEffector', 'RArmEffector'])
        self.transforms = {n: identity(4) for n in self.joint_names}

        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LArmEffector'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RArmEffector'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'LFoot'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RFoot']
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        T = identity(4)
        if joint_name == 'HeadYaw':
            T = Trans(0, 0, NECK_OFFSET_Z) * Rz(joint_angle)
        elif joint_name == 'HeadPitch':
            T = Ry(joint_angle)
        elif joint_name == 'LShoulderPitch':
            T = Trans(0, SHOULDER_OFFSET_Y, SHOULDER_OFFSET_Z) * Ry(joint_angle)
        elif joint_name == 'LShoulderRoll':
            T = Rz(joint_angle)
        elif joint_name == 'LElbowYaw':
            T = Trans(UPPER_ARM_LENGTH, ELBOW_OFFSET_Y, 0) * Rx(joint_angle)
        elif joint_name == 'LElbowRoll':
            T = Rz(joint_angle)
        elif joint_name == 'LWristYaw':
            T = Trans(LOWER_ARM_LENGTH, 0, 0) * Rx(joint_angle)
        elif joint_name == 'LArmEffector':
             T = Trans(HAND_OFFSET_X, 0, -HAND_OFFSET_Z)
        elif joint_name == 'RShoulderPitch':
            T = Trans(0, -SHOULDER_OFFSET_Y, SHOULDER_OFFSET_Z) * Ry(joint_angle)
        elif joint_name == 'RShoulderRoll':
            T = Rz(joint_angle)
        elif joint_name == 'RElbowYaw':
            T = Trans(UPPER_ARM_LENGTH, -ELBOW_OFFSET_Y, 0) * Rx(joint_angle)
        elif joint_name == 'RElbowRoll':
            T = Rz(joint_angle)
        elif joint_name == 'RWristYaw':
            T = Trans(LOWER_ARM_LENGTH, 0, 0) * Rx(joint_angle)
        elif joint_name == 'RArmEffector':
             T = Trans(HAND_OFFSET_X, 0, -HAND_OFFSET_Z)
        elif joint_name == 'LHipYawPitch':
            T = Trans(0, HIP_OFFSET_Y, -HIP_OFFSET_Z) * Rz(joint_angle)
        elif joint_name == 'LHipRoll':
            T = Rx(joint_angle)
        elif joint_name == 'LHipPitch':
            T = Ry(joint_angle)
        elif joint_name == 'LKneePitch':
            T = Trans(0, 0, -THIGH_LENGTH) * Ry(joint_angle)
        elif joint_name == 'LAnklePitch':
            T = Trans(0, 0, -TIBIA_LENGTH) * Ry(joint_angle)
        elif joint_name == 'LAnkleRoll':
            T = Rx(joint_angle)
        elif joint_name == 'LFoot':
            T = Trans(0, 0, -FOOT_HEIGHT)
        elif joint_name == 'RHipYawPitch':
            T = Trans(0, -HIP_OFFSET_Y, -HIP_OFFSET_Z) * Rz(joint_angle)
        elif joint_name == 'RHipRoll':
            T = Rx(joint_angle)
        elif joint_name == 'RHipPitch':
            T = Ry(joint_angle)
        elif joint_name == 'RKneePitch':
            T = Trans(0, 0, -THIGH_LENGTH) * Ry(joint_angle)
        elif joint_name == 'RAnklePitch':
            T = Trans(0, 0, -TIBIA_LENGTH) * Ry(joint_angle)
        elif joint_name == 'RAnkleRoll':
            T = Rx(joint_angle)
        elif joint_name == 'RFoot':
            T = Trans(0, 0, -FOOT_HEIGHT)
        return T

    def forward_kinematics(self, joints):
        for chain_name, chain_joints in self.chains.items():
            T_global = identity(4)
            for joint in chain_joints:
                angle = joints.get(joint, 0.0)
                Tl = self.local_trans(joint, angle)
                T_global = T_global * Tl
                self.transforms[joint] = T_global