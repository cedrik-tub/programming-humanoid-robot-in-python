# standing_up.py
'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''

from recognize_posture import PostureRecognitionAgent # Erbt von dieser Klasse

# Importiere hier ALLE Keyframe-Bewegungen, die für das Aufstehen benötigt werden.
# Basierend auf deiner keyframes-Ordnerstruktur:
try:
    from keyframes import leftBellyToStand
except ImportError:
    print("WARNUNG: Keyframe 'leftBellyToStand' konnte nicht importiert werden.")
    leftBellyToStand = None 

try:
    from keyframes import rightBellyToStand
except ImportError:
    print("WARNUNG: Keyframe 'rightBellyToStand' konnte nicht importiert werden.")
    rightBellyToStand = None

try:
    from keyframes import leftBackToStand
except ImportError:
    print("WARNUNG: Keyframe 'leftBackToStand' konnte nicht importiert werden.")
    leftBackToStand = None

try:
    from keyframes import rightBackToStand
except ImportError:
    print("WARNUNG: Keyframe 'rightBackToStand' konnte nicht importiert werden.")
    rightBackToStand = None

# Optional: Für andere Posen oder als Fallback
try:
    from keyframes import hello # oder eine spezifischere Stand-/Init-Pose
except ImportError:
    print("WARNUNG: Keyframe 'hello' konnte nicht importiert werden.")
    hello = None

# Annahme: init_pose ist eine stabile Standpose oder ein Fallback
init_pose = hello # Du kannst dies durch eine spezifischere init_pose ersetzen, wenn vorhanden


class StandingUpAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(StandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.is_currently_executing_stand_up_motion = False
        self.last_executed_stand_up_posture = None
        # Wir brauchen eine Möglichkeit, die "Seite" zu bestimmen, wenn der Roboter auf dem Bauch oder Rücken liegt.
        # Dies ist oft schwer allein aus den Gelenkwinkeln zu erkennen.
        # Für dieses Beispiel nehmen wir an, dass "Belly" und "Back" generisch sind
        # und wir vielleicht einen Standard-Keyframe (z.B. rightBellyToStand) verwenden
        # oder eine zusätzliche Logik bräuchten, um die Seite zu bestimmen (z.B. basierend auf den letzten Gelenkbewegungen).
        # Vorerst wählen wir eine Standardseite oder du musst dies erweitern.
        self.preferred_stand_side = "right" # oder "left", oder dynamisch bestimmen

    def think(self, perception):
        action = super(StandingUpAgent, self).think(perception)
        self.standing_up()
        return action

    def standing_up(self):
        current_posture = self.posture
        # print(f"DEBUG StandingUpAgent: Aktuell erkannte Haltung = '{current_posture}'")

        if self.is_currently_executing_stand_up_motion and self.last_executed_stand_up_posture == current_posture:
            return 

        new_keyframes_set = False
        stand_up_motion_to_execute = None

        if current_posture == 'Belly':
            print(f"INFO StandingUpAgent: Haltung ist '{current_posture}'. Wähle Aufstehbewegung vom Bauch.")
            if self.preferred_stand_side == "right" and rightBellyToStand:
                stand_up_motion_to_execute = rightBellyToStand()
            elif self.preferred_stand_side == "left" and leftBellyToStand:
                stand_up_motion_to_execute = leftBellyToStand()
            elif rightBellyToStand: # Fallback, falls preferred_stand_side nicht passt, aber einer da ist
                print(f"WARNUNG StandingUpAgent: Bevorzugte Seite '{self.preferred_stand_side}' für Belly nicht verfügbar, nutze rechten Keyframe.")
                stand_up_motion_to_execute = rightBellyToStand()
            elif leftBellyToStand:
                print(f"WARNUNG StandingUpAgent: Bevorzugte Seite '{self.preferred_stand_side}' für Belly nicht verfügbar, nutze linken Keyframe.")
                stand_up_motion_to_execute = leftBellyToStand()
            else:
                print(f"WARNUNG StandingUpAgent: Keine 'BellyToStand' Keyframes verfügbar.")
        
        elif current_posture == 'Back':
            print(f"INFO StandingUpAgent: Haltung ist '{current_posture}'. Wähle Aufstehbewegung vom Rücken.")
            if self.preferred_stand_side == "right" and rightBackToStand:
                stand_up_motion_to_execute = rightBackToStand()
            elif self.preferred_stand_side == "left" and leftBackToStand:
                stand_up_motion_to_execute = leftBackToStand()
            elif rightBackToStand:
                print(f"WARNUNG StandingUpAgent: Bevorzugte Seite '{self.preferred_stand_side}' für Back nicht verfügbar, nutze rechten Keyframe.")
                stand_up_motion_to_execute = rightBackToStand()
            elif leftBackToStand:
                print(f"WARNUNG StandingUpAgent: Bevorzugte Seite '{self.preferred_stand_side}' für Back nicht verfügbar, nutze linken Keyframe.")
                stand_up_motion_to_execute = leftBackToStand()
            else:
                print(f"WARNUNG StandingUpAgent: Keine 'BackToStand' Keyframes verfügbar.")

        # HIER KÖNNTEST DU LOGIK FÜR 'Left' und 'Right' Haltungen hinzufügen,
        # falls deine Haltungserkennung diese spezifisch erkennt.
        # Wenn 'Left' bedeutet, er liegt auf der linken Seite (z.B. nach einem Fall zur Seite),
        # könntest du z.B. versuchen, ihn erst auf den Bauch oder Rücken zu rollen
        # oder eine spezifische Aufstehbewegung von der Seite zu haben.
        # Fürs Erste lassen wir das aus, da deine Keyframes "BellyToStand" und "BackToStand" sind.
        # Beispiel:
        # elif current_posture == 'Left':
        #     print(f"INFO StandingUpAgent: Haltung ist '{current_posture}'.")
        #     # Hier Logik, um z.B. leftBellyToStand oder leftBackToStand zu nutzen,
        #     # oder eine spezielle "fromSide" Bewegung.
        #     if leftBellyToStand: # Annahme: von der Seite erst auf den Bauch
        #         stand_up_motion_to_execute = leftBellyToStand() # Oder eine Rollbewegung
        #     else:
        #         print(f"WARNUNG StandingUpAgent: Keine passende Aktion für Haltung '{current_posture}'.")


        elif current_posture in ['Stand', 'StandInit', 'Crouch', 'Sit']: # Sitzen ist auch relativ stabil
            # print(f"INFO StandingUpAgent: Roboter ist in Haltung '{current_posture}'. Keine primäre Aufstehaktion nötig.")
            if self.is_currently_executing_stand_up_motion:
                print(f"INFO StandingUpAgent: Aufstehbewegung abgeschlossen, Roboter ist in Haltung '{current_posture}'.")
                if init_pose and current_posture != 'StandInit': # Optional: Zurück zur Init-Pose, wenn nicht schon da
                    # self.keyframes = init_pose() # ACHTUNG: Dies kann zu Endlosschleifen führen, wenn init_pose nicht zu 'Stand' führt
                    pass
            self.is_currently_executing_stand_up_motion = False
            self.last_executed_stand_up_posture = None
            return 
        
        else: # Unbekannte oder nicht behandelte Haltung
            # print(f"INFO StandingUpAgent: Keine spezifische Aufstehaktion für Haltung '{current_posture}'.")
            if self.is_currently_executing_stand_up_motion: # Wenn eine Bewegung lief, jetzt aber nicht mehr, zurücksetzen
                self.is_currently_executing_stand_up_motion = False
                self.last_executed_stand_up_posture = None
            return

        if stand_up_motion_to_execute:
            self.keyframes = stand_up_motion_to_execute
            new_keyframes_set = True
            print(f"INFO StandingUpAgent: Setze Keyframes für Aufstehbewegung aus Haltung '{current_posture}'.")
        
        if new_keyframes_set:
            self.is_currently_executing_stand_up_motion = True
            self.last_executed_stand_up_posture = current_posture
        # Ansonsten bleibt der Status wie er ist


class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0.0 
        self.stiffness_on_cycle = 10.0  
        self.stiffness_off_cycle = 3.0  

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        
        if self.stiffness_on_off_time <= time_now < self.stiffness_on_off_time + self.stiffness_off_cycle:
            action.stiffness = {j: 0.0 for j in self.joint_names}
        elif self.stiffness_on_off_time + self.stiffness_off_cycle <= time_now < self.stiffness_on_off_time + self.stiffness_off_cycle + self.stiffness_on_cycle:
            action.stiffness = {j: 1.0 for j in self.joint_names}
        elif time_now >= self.stiffness_on_off_time + self.stiffness_off_cycle + self.stiffness_on_cycle:
            self.stiffness_on_off_time = time_now 
            action.stiffness = {j: 0.0 for j in self.joint_names}
            self.is_currently_executing_stand_up_motion = False 
            self.last_executed_stand_up_posture = None
        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()