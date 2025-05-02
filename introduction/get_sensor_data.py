'''
In this exercise you need to know how to get sensor data.

* Task: get the current joint angle and temperature of joint HeadYaw

* Hint: The current sensor data of robot is stored in perception (class Perception in spark_agent.py)
  Confirmed structure: perception.hinge_joints['hj1']['ax'] (angle) and perception.hinge_joints['hj1']['tp'] (temperature)

'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'software_installation'))

# Versuche, SparkAgent zu importieren. Gib eine Hilfestellung, wenn es fehlschlägt.
try:
    from spark_agent import SparkAgent, Perception
except ImportError:
    print("Fehler: Konnte 'spark_agent' nicht importieren.")
    print("Stelle sicher, dass der PYTHONPATH korrekt gesetzt ist und die Datei 'spark_agent.py' im Verzeichnis '../software_installation' existiert.")
    sys.exit(1)


class MyAgent(SparkAgent):
    def think(self, perception: Perception):
        angle = None
        temperature = None

        # YOUR CODE HERE (Now using the confirmed names)
        try:
            if perception and hasattr(perception, 'hinge_joints'):
                all_joints_data = perception.hinge_joints
                if all_joints_data and 'hj1' in all_joints_data:
                    head_yaw_data = all_joints_data['hj1']
                    if 'ax' in head_yaw_data:
                        angle = head_yaw_data['ax']
                    if 'tp' in head_yaw_data:
                        temperature = head_yaw_data['tp']
        except Exception as e:
            print(f"Unerwarteter Fehler beim Zugriff auf Gelenkdaten: {e}")

        print(f'HeadYaw angle: {angle} temperature: {temperature}')

        return super(MyAgent, self).think(perception)

if '__main__' == __name__:
    agent = MyAgent()
    agent.run()