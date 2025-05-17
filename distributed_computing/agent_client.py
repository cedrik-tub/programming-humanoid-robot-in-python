import rpyc
import weakref
import threading
import time

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, client_agent_instance):
        self.client_agent_proxy = weakref.proxy(client_agent_instance)
        self._async_results = {}

    def _execute_async(self, method_name, *args, **kwargs):
        """
        Interne Hilfsmethode, um eine beliebige Methode des ClientAgent.service asynchron auszuführen.
        Gibt ein RPyC AsyncResult Objekt zurück.
        """
        if not self.client_agent_proxy.service:
            raise ConnectionError("RPyC-Dienst nicht verbunden.")
        
        remote_method = getattr(self.client_agent_proxy.service, method_name)
        async_callable = rpyc.async_(remote_method)
        
        async_result = async_callable(*args, **kwargs)
        return async_result

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        print("Client (PostHandler): execute_keyframes asynchron angefordert.")
        return self._execute_async('exposed_execute_keyframes', keyframes)

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        print("Client (PostHandler): set_transform asynchron angefordert.")
        return self._execute_async('exposed_set_transform', effector_name, transform)


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    def __init__(self, host='localhost', port=18861):
        self.host = host
        self.port = port
        self.conn = None
        self.service = None
        self.post = PostHandler(self)
        self._connect()

    def _connect(self):
        try:
            self.conn = rpyc.connect(self.host, self.port, config={"allow_public_attrs": True})
            self.service = self.conn.root 
            print(f"Client: Verbunden mit Server auf {self.host}:{self.port}")
        except ConnectionRefusedError:
            print(f"Client: Verbindung zu {self.host}:{self.port} fehlgeschlagen. Läuft der Server?")
            self.service = None

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        if not self.service: return "Fehler: Nicht verbunden"
        print(f"Client: get_angle für {joint_name} angefordert.")
        return self.service.exposed_get_angle(joint_name)

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        if not self.service: return "Fehler: Nicht verbunden"
        print(f"Client: set_angle für {joint_name} auf {angle} angefordert.")
        return self.service.exposed_set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        if not self.service: return "Fehler: Nicht verbunden"
        print("Client: get_posture angefordert.")
        return self.service.exposed_get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        if not self.service: return "Fehler: Nicht verbunden"
        print(f"Client: execute_keyframes mit {len(keyframes)} Keyframes angefordert (blockierend).")
        result = self.service.exposed_execute_keyframes(keyframes)
        print("Client: execute_keyframes (blockierend) abgeschlossen.")
        return result

    def get_transform(self, name):
        '''get transform with given name
        '''
        if not self.service: return "Fehler: Nicht verbunden"
        print(f"Client: get_transform für {name} angefordert.")
        return self.service.exposed_get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        if not self.service: return "Fehler: Nicht verbunden"
        print(f"Client: set_transform für {effector_name} angefordert (blockierend).")
        result = self.service.exposed_set_transform(effector_name, transform)
        print("Client: set_transform (blockierend) abgeschlossen.")
        return result

    def close(self):
        if self.conn:
            print("Client: Verbindung wird geschlossen.")
            self.conn.close()
            self.conn = None
            self.service = None

    def __del__(self):
        self.close()


if __name__ == '__main__':
    agent = ClientAgent()

    if agent.service: 
        print("\n--- Teste blockierende Aufrufe ---")
        print("Posture:", agent.get_posture())
        print("Angle joint1:", agent.get_angle("joint1"))
        agent.set_angle("joint1", 1.23)
        print("Angle joint1 nach set:", agent.get_angle("joint1"))

        keyframes_data = [
            {"type": "pose", "time": 1.0, "posture": "waving"},
            {"type": "pose", "time": 2.0, "posture": "thinking"}
        ]
        print("Führe Keyframes aus (blockierend)...")
        response = agent.execute_keyframes(keyframes_data)
        print("Antwort von execute_keyframes:", response)
        print("Posture nach Keyframes:", agent.get_posture())

        print("\n--- Teste nicht-blockierende Aufrufe ---")
        
        print("Fordere execute_keyframes (nicht-blockierend) an...")
        async_result_keyframes = agent.post.execute_keyframes([{"posture": "dancing"}])
        print(f"Sofortige Rückgabe von post.execute_keyframes: {async_result_keyframes}")

        new_transform_data = [0,0,0,1, 0,0,1,0, 0,1,0,0, 1,0,0,0]
        print("Fordere set_transform (nicht-blockierend) an...")
        async_result_transform = agent.post.set_transform("effector1", new_transform_data)
        print(f"Sofortige Rückgabe von post.set_transform: {async_result_transform}")

        print("Mache andere Dinge, während die nicht-blockierenden Aufrufe laufen...")
        time.sleep(0.5) 
        print("Noch mehr andere Dinge...")

        print("\nWarte auf nicht-blockierende Ergebnisse...")
        
        while not async_result_keyframes.ready:
            print("Keyframes noch nicht fertig...")
            time.sleep(0.5)
        if async_result_keyframes.error:
            print("Fehler bei async execute_keyframes:", async_result_keyframes.value)
        else:
            print("Ergebnis von async execute_keyframes:", async_result_keyframes.value)
        print("Posture nach async Keyframes:", agent.get_posture())

        while not async_result_transform.ready:
            print("Transform noch nicht fertig...")
            time.sleep(0.5)
        if async_result_transform.error:
            print("Fehler bei async set_transform:", async_result_transform.value)
        else:
            print("Ergebnis von async set_transform:", async_result_transform.value)
        print("Transform nach async set_transform:", agent.get_transform("effector1"))

        agent.close()
    else:
        print("Client konnte keine Verbindung zum Server herstellen. Tests werden übersprungen.")