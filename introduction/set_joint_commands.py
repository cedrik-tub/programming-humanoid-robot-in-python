'''
In this exercise you need to know how to set joint commands.

* Tasks:
    1. set stiffness of LShoulderPitch (likely 'laj2') to 0
    2. set speed of HeadYaw ('hj1') to 0.1

* Hint: The commands are stored in action (class Action in spark_agent.py)
  Likely structure: action.hj['joint_name']['stf'] = value / action.hj['joint_name']['spd'] = value

'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'software_installation'))

try:
    from spark_agent import SparkAgent, Perception, Action
except ImportError:
    print("Fehler: Konnte 'spark_agent' nicht importieren.")
    print("Stelle sicher, dass der PYTHONPATH korrekt gesetzt ist und die Datei 'spark_agent.py' im Verzeichnis '../software_installation' existiert.")
    sys.exit(1)


class MyAgent(SparkAgent):
    def think(self, perception: Perception) -> Action:
        action = super(MyAgent, self).think(perception)

        if not hasattr(action, 'hj'):
             action.hj = {} 

        # YOUR CODE HERE
        lsp_joint_name = 'laj2'
        if lsp_joint_name not in action.hj:
            action.hj[lsp_joint_name] = {}

        action.hj[lsp_joint_name]['stf'] = 0.0
        print(f"Setting stiffness for {lsp_joint_name} to 0.0")


        hy_joint_name = 'hj1'

        if hy_joint_name not in action.hj:
            action.hj[hy_joint_name] = {}

        action.hj[hy_joint_name]['spd'] = 0.1
        print(f"Setting speed for {hy_joint_name} to 0.1")

        return action

if '__main__' == __name__:
    agent = MyAgent()
    agent.run()