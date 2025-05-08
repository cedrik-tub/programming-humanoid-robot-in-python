from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        interpolated_joints_from_keyframes = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(interpolated_joints_from_keyframes)

        if 'LHipYawPitch' in self.joint_names and 'RHipYawPitch' in self.joint_names:
            self.target_joints['RHipYawPitch'] = self.target_joints['LHipYawPitch']
        
        return super(AngleInterpolationAgent, self).think(perception)

    def _cubic_bezier(self, p0, p1, p2, p3, u):
        return ((1-u)**3 * p0 +
                3 * (1-u)**2 * u * p1 +
                3 * (1-u) * u**2 * p2 +
                u**3 * p3)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        names, times_data, keys_data = keyframes
        current_time = perception.time

        if not names or not times_data or not keys_data:
            return target_joints

        for joint_idx, joint_name in enumerate(names):
            if joint_idx >= len(times_data) or joint_idx >= len(keys_data):
                continue

            joint_times = times_data[joint_idx]
            joint_key_params = keys_data[joint_idx]

            if not joint_times or not joint_key_params:
                continue
            
            if current_time < joint_times[0]:
                if isinstance(joint_key_params[0], (list, tuple)):
                    target_joints[joint_name] = joint_key_params[0][0]
                else: 
                    target_joints[joint_name] = joint_key_params[0]
                continue

            if current_time >= joint_times[-1]:
                if isinstance(joint_key_params[-1], (list, tuple)):
                    target_joints[joint_name] = joint_key_params[-1][0]
                else:
                    target_joints[joint_name] = joint_key_params[-1]
                continue

            segment_idx = -1
            for i in range(len(joint_times) - 1):
                if joint_times[i] <= current_time < joint_times[i+1]:
                    segment_idx = i
                    break
            
            if segment_idx == -1:
                if isinstance(joint_key_params[-1], (list, tuple)):
                    target_joints[joint_name] = joint_key_params[-1][0]
                else:
                    target_joints[joint_name] = joint_key_params[-1]
                continue

            t0 = joint_times[segment_idx]
            t1 = joint_times[segment_idx+1]

            if t1 <= t0:
                if isinstance(joint_key_params[segment_idx], (list, tuple)):
                    target_joints[joint_name] = joint_key_params[segment_idx][0]
                else:
                    target_joints[joint_name] = joint_key_params[segment_idx]
                continue
            
            u = (current_time - t0) / (t1 - t0)

            params_key0 = joint_key_params[segment_idx]
            params_key1 = joint_key_params[segment_idx+1]
            
            if not (isinstance(params_key0, (list, tuple)) and len(params_key0) == 3 and
                    isinstance(params_key1, (list, tuple)) and len(params_key1) == 3):
                if isinstance(params_key0, (list, tuple)):
                    target_joints[joint_name] = params_key0[0]
                continue

            p0_angle = params_key0[0]
            p3_angle = params_key1[0]
            
            if not (isinstance(params_key0[2], (list, tuple)) and len(params_key0[2]) == 3 and
                    isinstance(params_key1[1], (list, tuple)) and len(params_key1[1]) == 3):
                target_joints[joint_name] = p0_angle
                continue

            handle0_out_dangle = params_key0[2][2] 
            p1_angle = p0_angle + handle0_out_dangle

            handle1_in_dangle = params_key1[1][2]
            p2_angle = p3_angle + handle1_in_dangle
            
            interpolated_angle = self._cubic_bezier(p0_angle, p1_angle, p2_angle, p3_angle, u)
            target_joints[joint_name] = interpolated_angle
            
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()
    agent.run()