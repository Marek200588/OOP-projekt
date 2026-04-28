import pybullet as p
import threading # Wbudowana biblioteka Pythona do wielowątkowości
Class Controller:
    def __init__(self, initial_xyz):
        self.target_xyz = list(initial_xyz)
