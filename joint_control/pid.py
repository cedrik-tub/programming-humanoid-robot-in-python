import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'software_installation'))

import numpy as np
from collections import deque
from spark_agent import SparkAgent, JOINT_CMD_NAMES

class PIDController(object):
    def __init__(self, dt, size):
        self.dt = dt
        self.size = size
        self.u = np.zeros(size)
        self.e1 = np.zeros(size)
        self.e2 = np.zeros(size)

        # HIER DIE BESTEN PARAMETER NACH DEM TUNING EINTRAGEN
        self.Kp = 20.0
        self.Ki = 0.2
        self.Kd = 1.0

        self._integral = np.zeros(size) # Wird für Velocity Form evtl. nicht direkt benötigt, aber im delta_u enthalten

        delay = 0
        self.y_buffer = deque(maxlen=delay + 1)
        for _ in range(delay + 1):
            self.y_buffer.appendleft(np.zeros(size))

    def set_delay(self, delay):
        current_elements = list(self.y_buffer)
        self.y_buffer = deque(maxlen=delay + 1)
        for elem in reversed(current_elements):
             if len(self.y_buffer) < delay + 1:
                 self.y_buffer.appendleft(elem)
             else:
                 break
        while len(self.y_buffer) < delay + 1:
            self.y_buffer.appendleft(np.zeros(self.size))

    def control(self, target, sensor):
        delay = self.y_buffer.maxlen - 1
        y_pred = sensor.copy()

        if delay > 0 and self.dt > 0:
             num_past_u = len(self.y_buffer)
             for k in range(min(delay, num_past_u)):
                 if k < len(self.y_buffer):
                     y_pred += self.y_buffer[k] * self.dt

        e0 = target - y_pred

        # Velocity Form Calculation
        delta_u = self.Kp * (e0 - self.e1)

        if self.dt > 0:
            delta_u += self.Ki * e0 * self.dt
            # Check if dt is sufficiently large to avoid division by a very small number if Kd is large
            if self.Kd != 0: # Avoid calculation if Kd is zero
                 delta_u += self.Kd * (e0 - 2*self.e1 + self.e2) / self.dt
        
        self.u += delta_u

        self.e2 = self.e1.copy()
        self.e1 = e0.copy()

        self.y_buffer.appendleft(self.u.copy())

        return self.u


class PIDAgent(SparkAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PIDAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.joint_names = list(JOINT_CMD_NAMES.keys()) # Ensure list for consistent order
        number_of_joints = len(self.joint_names)
        self.joint_controller = PIDController(dt=0.02, size=number_of_joints)
        self.target_joints = {k: 0.0 for k in self.joint_names} # Use float for consistency
        # Optional: self.joint_controller.set_delay(1) # Example


    def think(self, perception):
        action = super(PIDAgent, self).think(perception)

        joint_ids_ordered = self.joint_names # Use the stored ordered list

        joint_angles = np.nan_to_num(np.array(
            [perception.joint.get(joint_id, 0.0) for joint_id in joint_ids_ordered], dtype=float
        ))

        target_angles = np.array([self.target_joints.get(joint_id,
            perception.joint.get(joint_id, 0.0)) for joint_id in joint_ids_ordered], dtype=float)

        u = self.joint_controller.control(target_angles, joint_angles)

        action.speed = dict(zip(joint_ids_ordered, u))
        return action


if __name__ == '__main__':
    agent = PIDAgent()
    agent.target_joints['HeadYaw'] = 0.5
    # agent.target_joints['LShoulderPitch'] = -0.5
    agent.run()