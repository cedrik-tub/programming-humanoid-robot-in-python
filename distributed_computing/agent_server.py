import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

try:
    from inverse_kinematics import InverseKinematicsAgent
except ImportError:
    print("WARNUNG: inverse_kinematics.InverseKinematicsAgent nicht gefunden. Verwende Dummy-Implementierung.")
    class InverseKinematicsAgent(object):
        def __init__(self):
            self.angles = {"joint1": 0.0, "joint2": 0.0}
            self.posture = "standing"
            self.transforms = {"effector1": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]}
            print("Dummy InverseKinematicsAgent initialisiert.")

        def get_angle(self, joint_name):
            print(f"Server: get_angle für {joint_name} aufgerufen.")
            return self.angles.get(joint_name, "Unbekanntes Gelenk")

        def set_angle(self, joint_name, angle):
            print(f"Server: set_angle für {joint_name} auf {angle} aufgerufen.")
            if joint_name in self.angles:
                self.angles[joint_name] = angle
                return True
            return False

        def get_posture(self):
            print("Server: get_posture aufgerufen.")
            return self.posture

        def execute_keyframes(self, keyframes):
            print(f"Server: execute_keyframes mit {len(keyframes)} Keyframes aufgerufen. Blockierend...")
            import time
            time.sleep(2) 
            self.posture = keyframes[-1].get("posture", self.posture) if keyframes else self.posture
            print("Server: execute_keyframes abgeschlossen.")
            return "Keyframes ausgeführt"

        def get_transform(self, name):
            print(f"Server: get_transform für {name} aufgerufen.")
            return self.transforms.get(name, "Unbekannter Transform")

        def set_transform(self, effector_name, transform):
            print(f"Server: set_transform für {effector_name} mit {transform} aufgerufen. Blockierend...")
            import time
            time.sleep(2) 
            self.transforms[effector_name] = transform
            print("Server: set_transform abgeschlossen.")
            return True

import rpyc
from rpyc.utils.server import ThreadedServer

class ServerAgent(InverseKinematicsAgent, rpyc.Service):
    '''ServerAgent provides RPC service
    '''
    ALIASES = ["AGENT_SERVER"]

    def __init__(self):
        super().__init__()
        print("ServerAgent RPC Service initialisiert.")

    def exposed_get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return super().get_angle(joint_name)

    def exposed_set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        return super().set_angle(joint_name, angle)

    def exposed_get_posture(self):
        '''return current posture of robot'''
        return super().get_posture()

    def exposed_execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        return super().execute_keyframes(keyframes)

    def exposed_get_transform(self, name):
        '''get transform with given name
        '''
        return super().get_transform(name)

    def exposed_set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        return super().set_transform(effector_name, transform)

    def run(self, host='localhost', port=18861):
        """Startet den RPC-Server."""
        print(f"Starte ServerAgent RPC Service auf {host}:{port}...")
        server = ThreadedServer(self, hostname=host, port=port, protocol_config={"allow_public_attrs": True})
        server.start()

if __name__ == '__main__':
    agent_service = ServerAgent()
    print("Server wird gestartet. Drücke Strg+C zum Beenden.")
    agent_service.run()