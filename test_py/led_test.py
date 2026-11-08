#!/usr/bin/env python3
# File name   : led_test.py
# Description : Direkter Hardware-Test fuer die LEDs am Adeept Robot HAT V3.2
import sys
import time
import gpiod
from gpiod.line import Direction, Value

CHIP_PATH = "/dev/gpiochip0"
PIN_LED1 = 9
PIN_LED2 = 25
PIN_LED3 = 11

PINS = {
    1: PIN_LED1,
    2: PIN_LED2,
    3: PIN_LED3,
}


def test_hardware():
    print("=" * 60)
    print("Direkter Hardware-Test: Adeept Robot HAT V3.2 LEDs")
    print(f"GPIO-Pins: LED1 = {PIN_LED1}, LED2 = {PIN_LED2}, LED3 = {PIN_LED3}")
    print("WICHTIG: Bitte prüfen, ob die Schalter 'LED1, LED2, LED3'")
    print("auf dem Adeept Robot HAT V3.2 eingeschaltet (ON) sind!")
    print("=" * 60)

    try:
        chip = gpiod.Chip(CHIP_PATH)
    except Exception as e:
        print(f"Fehler beim Oeffnen von {CHIP_PATH}: {e}")
        sys.exit(1)

    line_config = {
        (PIN_LED1, PIN_LED2, PIN_LED3): gpiod.LineSettings(
            direction=Direction.OUTPUT,
            output_value=Value.INACTIVE
        )
    }

    try:
        request = chip.request_lines(consumer="led_hardware_test", config=line_config)
    except Exception as e:
        print(f"Fehler beim Anfordern der GPIO-Lines: {e}")
        print("Hinweis: Laeuft die ROS-2 led_node bereits? Falls ja, bitte vorher beenden.")
        sys.exit(1)

    def set_leds(l1: bool, l2: bool, l3: bool):
        request.set_values({
            PIN_LED1: Value.ACTIVE if l1 else Value.INACTIVE,
            PIN_LED2: Value.ACTIVE if l2 else Value.INACTIVE,
            PIN_LED3: Value.ACTIVE if l3 else Value.INACTIVE,
        })

    try:
        for cycle in range(1, 4):
            print(f"\n>>> Durchlauf {cycle}/3 <<<")

            # 1. LED1 an
            print(f"[LED 1] AN (GPIO {PIN_LED1})")
            set_leds(True, False, False)
            time.sleep(1.0)

            # 2. LED2 an
            print(f"[LED 2] AN (GPIO {PIN_LED2})")
            set_leds(False, True, False)
            time.sleep(1.0)

            # 3. LED3 an
            print(f"[LED 3] AN (GPIO {PIN_LED3})")
            set_leds(False, False, True)
            time.sleep(1.0)

            # Alle an
            print("[ALLE] AN")
            set_leds(True, True, True)
            time.sleep(1.0)

            # Alle aus
            print("[ALLE] AUS")
            set_leds(False, False, False)
            time.sleep(1.0)

        print("\nTest erfolgreich beendet.")

    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer.")
    finally:
        set_leds(False, False, False)
        request.release()
        print("LEDs ausgeschaltet und GPIO-Pins freigegeben.")


if __name__ == "__main__":
    test_hardware()
