# -*- coding: utf-8 -*-
# convert_pickles_py2.py
import pickle
import os
import sys # Nötig für sys.getfilesystemencoding()

# Passe diese Pfade an deine Ordnerstruktur an!
# RELATIVE PFADE zum Speicherort dieses Skripts:
ORIGINAL_DATA_DIR = 'robot_pose_data_original'
CONVERTED_DATA_DIR = 'robot_pose_data_converted'

# Stelle sicher, dass der Pfad korrekt ist, basierend auf dem Ort des Skripts
script_dir = os.path.dirname(os.path.abspath(__file__))
original_data_path = os.path.join(script_dir, ORIGINAL_DATA_DIR)
converted_data_path = os.path.join(script_dir, CONVERTED_DATA_DIR)

print "Python Version:", sys.version
print "System-Dateisystem-Encoding:", sys.getfilesystemencoding() # Gut zu wissen für Pfadprobleme
print "Originaldaten-Verzeichnis (erwartet):", original_data_path
print "Konvertierte Daten-Verzeichnis (wird erstellt/verwendet):", converted_data_path


if not os.path.exists(original_data_path):
    print "FEHLER: Originaldaten-Verzeichnis nicht gefunden:", original_data_path
    print "Bitte erstelle den Ordner '{}' und kopiere die Pickle-Dateien dorthin.".format(ORIGINAL_DATA_DIR)
    exit()

if not os.path.exists(converted_data_path):
    os.makedirs(converted_data_path)
    print "Ausgabeverzeichnis erstellt:", converted_data_path

try:
    # Versuche, die Dateinamen mit dem System-Encoding zu dekodieren, falls nötig
    # Unter Windows kann das Encoding der von os.listdir zurückgegebenen Strings manchmal problematisch sein
    # Für Python 2 ist dies oft weniger ein Problem als für Python 3, aber sicher ist sicher.
    # Wenn deine Dateinamen nur ASCII sind, ist das nicht nötig.
    class_names = [unicode(f, sys.getfilesystemencoding()) for f in os.listdir(original_data_path)]
except TypeError: # Falls os.listdir schon unicode zurückgibt
    class_names = os.listdir(original_data_path)


print "Gefundene Klassen zum Konvertieren:", class_names

successful_conversions = 0
failed_conversions = 0

for class_name in class_names:
    # Unter Windows können Pfade mit unicode manchmal Probleme machen, wenn sie nicht korrekt behandelt werden.
    # os.path.join sollte das aber handhaben.
    filename_in = os.path.join(original_data_path, class_name)
    filename_out = os.path.join(converted_data_path, class_name)
    
    print "Verarbeite:", filename_in.encode('utf-8') # Für die Ausgabe in der Konsole
    
    try:
        # Lade mit Python 2 pickle
        with open(filename_in, 'rb') as f_in:
            data = pickle.load(f_in)
        
        # Speichere erneut mit Python 2 pickle, aber mit protocol=2
        # Protokoll 2 ist binär und wird von Python 3 gut verstanden.
        with open(filename_out, 'wb') as f_out:
            pickle.dump(data, f_out, protocol=2) 
        
        print "  -> Erfolgreich konvertiert und gespeichert nach:", filename_out.encode('utf-8')
        successful_conversions += 1
    except Exception as e:
        print "!! FEHLER beim Verarbeiten von", filename_in.encode('utf-8'), ":", e
        failed_conversions += 1

print "\nKonvertierung abgeschlossen."
print "Erfolgreich konvertiert:", successful_conversions
print "Fehlgeschlagen:", failed_conversions