# recognize_posture.py
'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''

import pickle
import numpy as np
from os import path

# AngleInterpolationAgent wird als Basisklasse für Keyframe-Ausführung benötigt
from angle_interpolation import AngleInterpolationAgent 
# keyframes.hello wird nur für das __main__ Beispiel unten benötigt
from keyframes import hello 


# Der Pfad zur Klassifikator-Datei, relativ zum Skript
# Stelle sicher, dass robot_pose.pkl im selben Verzeichnis wie dieses Skript liegt
# oder passe den Pfad entsprechend an.
ROBOT_POSE_CLF_FILE = 'robot_pose.pkl'

# Die Namen der Haltungen in der exakten Reihenfolge, wie sie beim Training verwendet wurden.
# Diese Liste ist KRITISCH für die korrekte Interpretation der Klassifikator-Ausgabe.
# Hole diese aus deinem learn_posture.ipynb, z.B. aus der Ausgabe von "print classes".
# Die Reihenfolge hier MUSS mit der Reihenfolge der Daten beim Training des Klassifikators übereinstimmen.
# Beispiel (basierend auf typischen Ausgaben, BITTE ÜBERPRÜFE UND KORRIGIERE DIESE FÜR DEIN PROJEKT):
# classes = ['Frog', 'HeadBack', 'Left', 'Knee', 'Crouch', 'Back', 'Belly', 'Right', 'Sit', 'Stand', 'StandInit']
# Wenn dein print classes z.B. war: ['Back', 'Belly', 'Crouch', 'Frog', 'HeadBack', 'Knee', 'Left', 'Right', 'Sit', 'Stand', 'StandInit']
# dann nutze diese Reihenfolge!
# Für dieses Beispiel nehme ich eine häufig gesehene Reihenfolge an:
POSTURE_NAMES = ['Back', 'Belly', 'Crouch', 'Frog', 'HeadBack', 'Knee', 'Left', 'Right', 'Sit', 'Stand', 'StandInit']


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown_init' # Initialer Status
        self.posture_classifier = None
        self.posture_names_ordered = POSTURE_NAMES # Speichere die geordnete Liste

        # Lade den trainierten Klassifikator
        try:
            # Stelle sicher, dass der Pfad korrekt ist, falls sich die Datei nicht im selben Verzeichnis befindet
            script_dir = path.dirname(path.abspath(__file__))
            classifier_path = path.join(script_dir, ROBOT_POSE_CLF_FILE)
            with open(classifier_path, 'rb') as f:
                self.posture_classifier = pickle.load(f)
            print(f"INFO: Haltungserkennungs-Klassifikator '{ROBOT_POSE_CLF_FILE}' erfolgreich geladen.")
        except FileNotFoundError:
            print(f"FEHLER: Klassifikator-Datei '{ROBOT_POSE_CLF_FILE}' nicht unter '{classifier_path}' gefunden.")
        except Exception as e:
            print(f"FEHLER beim Laden des Klassifikators '{ROBOT_POSE_CLF_FILE}': {e}")

    def think(self, perception):
        # Erkenne zuerst die aktuelle Haltung
        current_recognized_posture = self.recognize_posture(perception)
        
        # Optional: Gib nur aus, wenn sich die Haltung ändert, um die Konsole nicht zu fluten
        if current_recognized_posture != self.posture:
             # print(f"DEBUG: Haltung geändert von '{self.posture}' zu '{current_recognized_posture}'")
             pass
        
        self.posture = current_recognized_posture
        
        # Rufe die think-Methode der Elternklasse auf, die sich um die Keyframe-Ausführung kümmert
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        # Wenn der Klassifikator nicht geladen werden konnte, gib einen Fehlerstatus zurück
        if self.posture_classifier is None:
            return 'unknown_clf_not_loaded'

        # Feature-Reihenfolge muss exakt der beim Training entsprechen:
        # ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 
        #  'RHipYawPitch', 'LHipRoll', 'LHipPitch', 'RKneePitch',  <- Korrektur: RHipRoll, RHipPitch
        #  'AngleX', 'AngleY']
        # Die Aufgabenbeschreibung sagt: "RHipYawPitch als LHipYawPitch"
        # Die restlichen rechten Beingelenke sollten ihre eigenen Werte haben.

        try:
            # Extrahiere Gelenkwinkel aus der Wahrnehmung (Perception)
            # Verwende .get(key, default_value) um Fehler zu vermeiden, falls ein Gelenk fehlt
            l_hip_yaw_pitch = perception.joint.get('LHipYawPitch', 0.0)
            l_hip_roll      = perception.joint.get('LHipRoll', 0.0)
            l_hip_pitch     = perception.joint.get('LHipPitch', 0.0)
            l_knee_pitch    = perception.joint.get('LKneePitch', 0.0)
            
            r_hip_yaw_pitch = l_hip_yaw_pitch # Gemäß Anweisung
            r_hip_roll      = perception.joint.get('RHipRoll', 0.0)
            r_hip_pitch     = perception.joint.get('RHipPitch', 0.0)
            r_knee_pitch    = perception.joint.get('RKneePitch', 0.0)
            
            # Extrahiere Körperwinkel (IMU-Daten)
            # Die genaue Struktur von perception.imu muss bekannt sein.
            # Annahme: es ist ein Objekt mit 'angleX' und 'angleY' Attributen oder ein Dict.
            angle_x = 0.0
            angle_y = 0.0
            if hasattr(perception, 'imu') and perception.imu is not None:
                if isinstance(perception.imu, dict):
                    angle_x = perception.imu.get('angleX', 0.0)
                    angle_y = perception.imu.get('angleY', 0.0)
                else: # Annahme: es ist ein Objekt mit Attributen
                    angle_x = getattr(perception.imu, 'angleX', 0.0)
                    angle_y = getattr(perception.imu, 'angleY', 0.0)
            
            # Stelle die Features als NumPy-Array in der korrekten Reihenfolge zusammen
            features_array = np.array([
                l_hip_yaw_pitch, l_hip_roll, l_hip_pitch, l_knee_pitch,
                r_hip_yaw_pitch, r_hip_roll, r_hip_pitch, r_knee_pitch,
                angle_x, angle_y
            ], dtype=np.float64) # Stelle sicher, dass es Float64 ist, wie von Scikit-learn erwartet

            # Das Modell erwartet eine 2D-Array Eingabe (auch für ein einzelnes Sample)
            features_reshaped = features_array.reshape(1, -1)
            
            # Mache eine Vorhersage mit dem geladenen Klassifikator
            predicted_label_index_array = self.posture_classifier.predict(features_reshaped)
            predicted_label_index = predicted_label_index_array[0] # predict gibt ein Array zurück
            
            # Konvertiere den vorhergesagten Index zurück in den Namen der Haltung
            if 0 <= predicted_label_index < len(self.posture_names_ordered):
                recognized_posture_name = self.posture_names_ordered[predicted_label_index]
            else:
                # print(f"WARNUNG: Ungültiger vorhergesagter Index {predicted_label_index} vom Klassifikator.")
                recognized_posture_name = 'unknown_invalid_idx'
                
            return recognized_posture_name

        except Exception as e:
            # print(f"FEHLER während der Haltungserkennung: {e}")
            # import traceback
            # traceback.print_exc() # Für detaillierten Stacktrace beim Debuggen
            return 'unknown_exception'


if __name__ == '__main__':
    # Teste die PostureRecognitionAgent Klasse
    agent = PostureRecognitionAgent()
    
    # Lasse den Agenten eine Bewegung ausführen (z.B. "hello")
    # Währenddessen sollte er versuchen, die Haltung zu erkennen.
    # Beobachte die Konsolenausgaben für erkannte Haltungen oder Fehler.
    agent.keyframes = hello() 
    
    # Um spezifische Haltungen zu testen, müsstest du Keyframes laden,
    # die den Roboter in diese Haltungen bringen und dann die Ausgabe von agent.posture prüfen.
    # Beispiel (erfordert, dass der Agent eine Weile läuft, um die Pose zu erreichen):
    # agent.keyframes = some_pose_that_results_in_belly() 
    # Nach einiger Zeit: print(agent.posture)

    agent.run()