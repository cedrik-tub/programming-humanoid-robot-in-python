'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


import numpy as np
from math import acos, asin, atan2, sqrt, pi, degrees

from forward_kinematics import ForwardKinematicsAgent, Trans, Rx, Ry, Rz
from forward_kinematics import THIGH_LENGTH, TIBIA_LENGTH, HIP_OFFSET_Y, HIP_OFFSET_Z, FOOT_HEIGHT

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform_torso_to_foot):
        joint_angles_rad = [0.0] * 6

        if effector_name != 'LLeg' and effector_name != 'RLeg':
            print(f"Inverse Kinematics not implemented for {effector_name}")
            return joint_angles_rad

        l_upper_leg = THIGH_LENGTH
        l_lower_leg = TIBIA_LENGTH
        
        current_hip_offset_y = HIP_OFFSET_Y
        if effector_name == 'RLeg':
            current_hip_offset_y = -HIP_OFFSET_Y

        T_torso_to_hip_yaw_pitch_origin = Trans(0, current_hip_offset_y, -HIP_OFFSET_Z)
        T_hip_yaw_pitch_origin_to_foot = np.linalg.inv(T_torso_to_hip_yaw_pitch_origin) * transform_torso_to_foot
        
        p_AR_H0 = np.array(T_hip_yaw_pitch_origin_to_foot[0:3, 3]).flatten()

        rot_angle_hip_ortho = pi/4
        if effector_name == 'RLeg':
            rot_angle_hip_ortho = -pi/4
        
        Rx_hip_ortho_mat = np.array(Rx(rot_angle_hip_ortho)[0:3, 0:3])

        p_AR_H_ortho = Rx_hip_ortho_mat @ p_AR_H0
        x, y, z = p_AR_H_ortho[0], p_AR_H_ortho[1], p_AR_H_ortho[2]

        l_trans_sq = x*x + y*y + z*z
        l_trans = sqrt(l_trans_sq)

        cos_gamma_knee_arg = (l_upper_leg**2 + l_lower_leg**2 - l_trans_sq) / (2 * l_upper_leg * l_lower_leg)
        if cos_gamma_knee_arg > 1.0: cos_gamma_knee_arg = 1.0
        if cos_gamma_knee_arg < -1.0: cos_gamma_knee_arg = -1.0
        gamma_knee = acos(cos_gamma_knee_arg)
        delta_knee = gamma_knee # Aldebaran Doku: positiver Winkel ist Beugung
        joint_angles_rad[3] = delta_knee

        val_acos_foot1_arg = (l_lower_leg**2 + l_trans_sq - l_upper_leg**2) / (2 * l_lower_leg * l_trans)
        if l_trans == 0: # Vermeide Division durch Null, falls Fuß am Hüft-Ursprung
             val_acos_foot1_arg = 0 # Führt zu pi/2, was wahrscheinlich nicht ideal ist, aber Singularität vermeidet
        elif val_acos_foot1_arg > 1.0: val_acos_foot1_arg = 1.0
        elif val_acos_foot1_arg < -1.0: val_acos_foot1_arg = -1.0
        delta_foot_pitch1 = acos(val_acos_foot1_arg)

        delta_foot_pitch2 = atan2(x, sqrt(y**2 + z**2) if (y**2 + z**2) > 1e-9 else 1e-9) # Vermeide sqrt(0)
        delta_foot_roll = atan2(y, z if abs(z) > 1e-9 else 1e-9 * np.sign(z) if z!=0 else 1e-9 ) #Vermeide atan2(y,0) für z=0

        joint_angles_rad[4] = -(delta_foot_pitch1 + delta_foot_pitch2) # NAO Konvention: negativer Pitch neigt Fuß nach unten
        joint_angles_rad[5] = delta_foot_roll                    
        
        R_H0_AR_original = np.array(T_hip_yaw_pitch_origin_to_foot[0:3, 0:3])
        R_H_ortho_AR = Rx_hip_ortho_mat @ R_H0_AR_original
        
        R_Thigh_Knee = np.array(Ry(delta_knee)[0:3,0:3])
        R_Knee_AnkleP = np.array(Ry(joint_angles_rad[4])[0:3,0:3])
        R_AnkleP_AnkleR = np.array(Rx(joint_angles_rad[5])[0:3,0:3])
        
        R_Thigh_AR = R_Thigh_Knee @ R_Knee_AnkleP @ R_AnkleP_AnkleR

        R_HT = R_H_ortho_AR @ np.linalg.inv(R_Thigh_AR)

        # Euler-Winkel-Extraktion aus R_HT (Z-X-Y Reihenfolge für HipYawPitch, HipRoll', HipPitch')
        # R_HT = Rz(delta_hip_yaw) * Rx(delta_hip_roll_prime) * Ry(delta_hip_pitch)
        
        # sin(roll') = R_HT[2,1]
        sin_delta_hip_roll_prime_val = R_HT[2,1]
        if sin_delta_hip_roll_prime_val > 1.0: sin_delta_hip_roll_prime_val = 1.0
        if sin_delta_hip_roll_prime_val < -1.0: sin_delta_hip_roll_prime_val = -1.0
        delta_hip_roll_prime = asin(sin_delta_hip_roll_prime_val)

        cos_delta_hip_roll_prime = cos(delta_hip_roll_prime)

        if abs(cos_delta_hip_roll_prime) < 1e-6: # Gimbal Lock Fall
            delta_hip_yaw = 0 # Kann nicht eindeutig bestimmt werden, setze zu 0
            delta_hip_pitch = atan2(R_HT[0,2], R_HT[0,0]) # Oder eine andere Konvention
        else:
            delta_hip_pitch = atan2(-R_HT[2,0] / cos_delta_hip_roll_prime, R_HT[2,2] / cos_delta_hip_roll_prime)
            delta_hip_yaw = atan2(-R_HT[0,1] / cos_delta_hip_roll_prime, R_HT[1,1] / cos_delta_hip_roll_prime)

        joint_angles_rad[0] = delta_hip_yaw     
        joint_angles_rad[1] = delta_hip_roll_prime - rot_angle_hip_ortho 
        joint_angles_rad[2] = delta_hip_pitch  

        if effector_name == 'RLeg':
            joint_angles_rad[0] *= -1 
            joint_angles_rad[1] *= -1 
            joint_angles_rad[5] *= -1 
            
        return joint_angles_rad

    def set_transforms(self, effector_name, transform):
        joint_angles_rad = self.inverse_kinematics(effector_name, transform)
        
        joint_angles_deg = [degrees(angle) for angle in joint_angles_rad]
        print(f"Calculated angles for {effector_name} (deg): {joint_angles_deg}")

        if effector_name == 'LLeg':
            leg_joint_names = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        elif effector_name == 'RLeg':
            leg_joint_names = ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        else:
            return

        if joint_angles_rad:
            values_for_keyframes = [joint_angles_rad]
            times_for_keyframes = [[1.0] * len(joint_angles_rad)] 
            self.keyframes = (leg_joint_names, values_for_keyframes, times_for_keyframes)


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    
    target_x_torso = 0.0
    target_y_torso_left_foot = HIP_OFFSET_Y + 0.05
    target_z_torso = -0.26 

    T_LLeg_target = np.array(Trans(target_x_torso, target_y_torso_left_foot, target_z_torso))
    T_LLeg_target[0:3, 0:3] = np.identity(3)

    print("Ziel-Transformation für LLeg (Torso zu Fuß):")
    print(T_LLeg_target)
    agent.set_transforms('LLeg', T_LLeg_target)
    
    agent.run()