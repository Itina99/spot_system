"""
Deve contenere le seguenti classi astratte
- RobotInterface: interfaccia astratta per i robot, con metodi come move_forward(), turn_left(), turn_right(), stop(), etc.
-LocalGridProvider: interfaccia astratta per fornire una griglia locale dell'ambiente, con metodi come get_local_grid(), update_grid(), etc.
-MovementProvider: interfaccia astratta per fornire funzionalità di movimento, con metodi come move_to(x, y), rotate(angle), etc.
-StateProvider: interfaccia astratta per non so bene cosa
"""
from abc import abstractmethod, ABC
from typing import Tuple
import numpy as np


class LocalGridProvider(ABC):
    @abstractmethod
    def get_obstacle_distance_grid(self)-> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns (pts, cells_obstacle_dist, color)"""
        pass

class StateProvider(ABC):



class MovementProvider(ABC):


class VisualizerProvider(ABC):



class RecordingProvider(ABC):
