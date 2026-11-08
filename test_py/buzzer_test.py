#!/usr/bin/env python3
# File name   : buzzer_test.py
# Description : Testskript fuer den passiven Buzzer am Adeept Robot HAT V3.2 (GPIO 18)
#               Gemaess Adeept Lesson 8: Use the Buzzer to Play Music
import sys
import time
from gpiozero import TonalBuzzer

BUZZER_PIN = 18

# Demo 1: Einzeltöne (Lesson 8, Seite 5)
SINGLE_NOTE = [
    ["C4", 0.5]
]

# Demo 2: 7 Grundnoten C4 bis B4 (Lesson 8, Seite 6)
# C4: 261.63 Hz, D4: 293.66 Hz, E4: 328.63 Hz, F4: 348.23 Hz,
# G4: 392.00 Hz, A4: 440.00 Hz, B4: 493.88 Hz
SEVEN_NOTES = [
    ["C4", 0.5],
    ["D4", 0.5],
    ["E4", 0.5],
    ["F4", 0.5],
    ["G4", 0.5],
    ["A4", 0.5],
    ["B4", 0.5]
]

# Demo 3: "Happy Birthday" (Lesson 8, Seite 7)
HAPPY_BIRTHDAY_SONG = [
    ["G4", 0.3], ["G4", 0.3], ["A4", 0.3], ["G4", 0.3], ["C5", 0.3], ["B4", 0.6],
    ["G4", 0.3], ["G4", 0.3], ["A4", 0.3], ["G4", 0.3], ["D5", 0.3], ["C5", 0.6],
    ["G4", 0.3], ["G4", 0.3], ["C5", 0.3], ["B4", 0.3], ["C5", 0.3], ["B4", 0.3], ["A4", 0.6],
    ["F5", 0.3], ["F5", 0.3], ["B4", 0.3], ["C5", 0.3], ["D5", 0.3], ["C5", 0.6]
]


def play(tb: TonalBuzzer, tune, pause_between_notes: float = 0.05):
    """Spielt eine Notenfolge ab (Note und Dauer in Sekunden)."""
    for note, duration in tune:
        print(f"  Note: {note} ({duration}s)")
        tb.play(note)
        time.sleep(float(duration))
        tb.stop()
        if pause_between_notes > 0:
            time.sleep(pause_between_notes)
    tb.stop()


def main():
    mode = "all"
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()

    print("=" * 60)
    print("Adeept Robot HAT V3.2 - Buzzer Test (GPIO 18)")
    print("Beenden mit Strg+C.")
    print("=" * 60)

    try:
        tb = TonalBuzzer(BUZZER_PIN)
    except Exception as e:
        print(f"Fehler beim Initialisieren des Buzzers an GPIO {BUZZER_PIN}: {e}")
        sys.exit(1)

    try:
        if mode in ("1", "single"):
            print("\n[Demo 1] Einzelton abspielen (C4)...")
            play(tb, SINGLE_NOTE)

        elif mode in ("2", "scale", "seven"):
            print("\n[Demo 2] Die 7 Grundnoten (C4 bis B4)...")
            play(tb, SEVEN_NOTES)

        elif mode in ("3", "song", "birthday"):
            print("\n[Demo 3] 'Happy Birthday' Melodie...")
            play(tb, HAPPY_BIRTHDAY_SONG)

        elif mode in ("beep", "short"):
            print("\nKurzer Bestätigungspiepton...")
            play(tb, [["A4", 0.1], ["C5", 0.15]])

        else:
            print("\n--- 1. Einzelton (C4) ---")
            play(tb, SINGLE_NOTE)
            time.sleep(1.0)

            print("\n--- 2. Die 7 Grundnoten (C4 bis B4) ---")
            play(tb, SEVEN_NOTES)
            time.sleep(1.0)

            print("\n--- 3. Melodie: 'Happy Birthday' ---")
            play(tb, HAPPY_BIRTHDAY_SONG)

        print("\nWiedergabe beendet.")

    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer.")
    finally:
        tb.stop()
        tb.close()
        print("Buzzer stummgeschaltet.")


if __name__ == "__main__":
    main()

