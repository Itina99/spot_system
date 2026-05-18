"""
Deve contenere le seguenti classi astratte
- RobotInterface: interfaccia astratta per i robot, con metodi come move_forward(), turn_left(), turn_right(), stop(), etc.
-LocalGridProvider: interfaccia astratta per fornire una griglia locale dell'ambiente, con metodi come get_local_grid(), update_grid(), etc.
-MovementProvider: interfaccia astratta per fornire funzionalità di movimento, con metodi come move_to(x, y), rotate(angle), etc.
-StateProvider: interfaccia astratta per non so bene cosa
"""
from abc import abstractmethod, ABC
from typing import Tuple,Dict, List, Optional, Any
import numpy as np


class LocalGridProvider(ABC):
    """
        Interfaccia per accedere ai dati della griglia locale di ostacoli.
        Astrae le differenze tra SDK Spot (obstacle_distance grid) e ROS (OccupancyGrid).
    """
    @abstractmethod
    def get_obstacle_distance_grid(self)-> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Ritorna i dati della griglia di distanza dagli ostacoli.

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]: (pts, cells_obstacle_dist, color)
                - pts: Array [N, 2] con coordinate (x, y) di ogni punto della griglia
                - cells_obstacle_dist: Array [N] con distanza da ostacoli per ogni punto
                                      (< 0 = dentro ostacolo, >= 0 = libero)
                - color: Array [N, 3] con colori per visualizzazione (0-255)
        """
        pass

class StateProvider(ABC):
    """
        Interfaccia per accedere allo stato del robot (posizione, orientamento).
        Astrae le differenze tra SDK (transforms_snapshot) e ROS (odometry).
    """

    @abstractmethod
    def get_position(self) -> Tuple[float, float, float]:
        """
        Ritorna la posizione assoluta del robot.

        Returns:
            Tuple[float, float, float]: (x, y, z) in coordinate mondo
        """
        pass

    @abstractmethod
    def get_yaw(self) -> float:
        """
        Ritorna l'angolo yaw (rotazione attorno asse Z).

        Returns:
            float: Angolo in radianti [-π, π]
        """
        pass

    @abstractmethod
    def get_quaternion(self) ->  Any:
        """
        Ritorna il quaternione di rotazione (formato nativo del sistema).
        Per SDK ritorna il quaternione Bosdyn, per ROS potrebbe essere None.

        Returns:
            Any: Quaternione (formato dipendente dalla piattaforma)
        """
        pass

class MovementProvider(ABC):
    """
        Interfaccia per controllare i movimenti del robot.
        Astrae le differenze tra SDK (relative_move) e ROS (motion controller).
    """

    @abstractmethod
    def rotate_by(self, dyaw: float) -> bool:
        """
        Ruota il robot di un certo angolo.

        Args:
            dyaw: Angolo di rotazione in radianti (positivo = counter-clockwise)

        Returns:
            bool: True se rotazione completata, False se fallita
        """
        pass

    @abstractmethod
    def move_forward(self, distance: float) -> bool:
        """
        Muove il robot in avanti della distanza specificata.

        Args:
            distance: Distanza in metri (positivo = avanti, negativo = indietro)

        Returns:
            bool: True se movimento completato, False se fallito
        """
        pass

    @abstractmethod
    def move_to(self, target_x: float, target_y: float, target_z:float) -> bool:
        """
        Muove il robot verso coordinate assolute (x, y).
        Per SDK potrebbe non essere direttamente supportato (usare rotate + move_forward).

        Args:
            target_x: Coordinata X target
            target_y: Coordinata Y target

        Returns:
            bool: True se movimento completato, False se fallito
        """
        pass

class VisualizerProvider(ABC):
    @abstractmethod
    def visualize_iteration(
            self,
            pts: np.ndarray,
            cells_obstacle_dist: np.ndarray,
            robot_x: float,
            robot_y: float,
            candidates: Dict[str, List[Tuple[float, float]]],
            chosen_point: Optional[Tuple[float, float]],
            iteration: int,
            env: Any,  # EnvironmentMap type
    ) -> None:
        """
        Visualizza lo stato corrente dell'esplorazione.

        Args:
            pts: Punti della griglia di ostacoli
            cells_obstacle_dist: Valori di distanza da ostacoli
            robot_x: Posizione X corrente del robot
            robot_y: Posizione Y corrente del robot
            candidates: Dict con 'rejected' e 'valid' sample points
            chosen_point: (x, y) del punto scelto come target, o None
            iteration: Numero iterazione esplorazione
            env: EnvironmentMap per plotting delle celle
        """
        pass

class RecordingProvider(ABC):
    """
        Interfaccia per gestione waypoint e recording della mappa.
        Astrae le differenze tra SDK (graph-nav RecordingInterface) e ROS (simple recording).
        """

    @abstractmethod
    def create_waypoint(
            self,
            cell_row: int,
            cell_col: int,
            x: float,
            y: float,
            z: float,
            yaw: float
    ) -> bool:
        """
        Crea un waypoint nella posizione specificata.

        Args:
            cell_row: Riga della cella nella griglia (per tracking)
            cell_col: Colonna della cella nella griglia (per tracking)
            x: Coordinata X del waypoint
            y: Coordinata Y del waypoint
            z: Coordinata Z del waypoint
            yaw: Orientamento yaw del waypoint

        Returns:
            bool: True se waypoint creato corrrettamente
        """
        pass

    @abstractmethod
    def get_all_waypoints(self) -> Dict[str, Dict[str, Any]]:
        """
        Ritorna tutti i waypoint creati.

        Returns:
            Dict: Dizionario waypoint {waypoint_name: {cell_row, cell_col, x, y, z, yaw, ...}}
        """
        pass

    @abstractmethod
    def find_nearest_waypoint_to_target(
            self,
            target_cell: Tuple[int, int]
    ) -> Optional[Tuple[int, int]]:
        """
        Trova il waypoint più vicino a una cella target.

        Args:
            target_cell: (row, col) della cella target

        Returns:
            Optional[Tuple[int, int]]: (row, col) del waypoint più vicino, o None
        """
        pass