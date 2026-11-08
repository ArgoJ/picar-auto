#!/usr/bin/env python3
# File name   : conftest.py
# Description : Pytest Konfiguration und Fixture-Bereitstellung
import os
import sys

# Sicherstellen, dass das Testverzeichnis im Suchpfad ist
sys.path.insert(0, os.path.dirname(__file__))

from common import ros_context  # noqa: F401, E402
