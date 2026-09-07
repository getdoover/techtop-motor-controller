"""Techtop TTA-3 / Invertek Optidrive E3 motor controller application."""

from pydoover.docker import run_app

from .application import TechtopMotorControllerApplication


def main():
    run_app(TechtopMotorControllerApplication())
