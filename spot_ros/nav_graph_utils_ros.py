"""ROS-compatible stubs for GraphNav utilities.

These keep the EasyWalk flow running in simulation without GraphNav.
"""
from dataclasses import dataclass
import time

@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    cell_row: int | None = None
    cell_col: int | None = None


class RecordingInterface:
    def __init__(self, *args, **kwargs):
        self.waypoints = []
        self._download_filepath = None

    def stop_recording(self, *args, **kwargs):
        return True

    def start_recording(self, *args, **kwargs):
        return True

    def clear_map(self, *args, **kwargs):
        self.waypoints.clear()
        return True

    def set_download_filepath(self, filepath):
        self._download_filepath = filepath

    def create_default_waypoint(self, cell_row=None, cell_col=None, x=0.0, y=0.0, z=0.0, yaw=0.0):
        ##### info sul robot sono passate come parametri ####
        name = f"wp_{len(self.waypoints)}"
        wp = Waypoint(name=name, x=x, y=y, z=z, yaw=yaw, cell_row=cell_row, cell_col=cell_col)
        self.waypoints.append(wp)
        return wp

    def get_recording_status(self, *args, **kwargs):
        return True

    def get_all_manual_waypoints_with_cells(self):
        data = {}
        for wp in self.waypoints:
            if wp.cell_row is None or wp.cell_col is None:
                continue
            data[(wp.cell_row, wp.cell_col)] = {
                'id': wp.name,
                'name': wp.name,
                'x': wp.x,
                'y': wp.y,
                'z': wp.z,
                'yaw': wp.yaw,
            }
        return data

    def get_manual_waypoint_by_cell(self, cell_row, cell_col):
        for wp in self.waypoints:
            if wp.cell_row == cell_row and wp.cell_col == cell_col:
                return {'id': wp.name, 'name': wp.name, 'x': wp.x, 'y': wp.y, 'z': wp.z, 'yaw': wp.yaw}
        return None

    def find_nearest_waypoint_cell_to_target(self, target_cell, waypoints_by_cell, env_map):
        if not waypoints_by_cell:
            return None
        target_row, target_col = target_cell
        best_cell = None
        best_dist = 1e9
        for (row, col) in waypoints_by_cell.keys():
            d = abs(row - target_row) + abs(col - target_col)
            if d < best_dist:
                best_dist = d
                best_cell = (row, col)
        return best_cell

    def navigate_to_waypoint(self, waypoint_id:str, motion_controller = None, max_retries: int = 3, timeout: int = 30):

        target_waypoint = None
        for wp in self.waypoints:
            if wp.name == waypoint_id:
                target_waypoint = wp
                break
        if target_waypoint is None:
            print(f"[NAV_ROS] ✗ Waypoint '{waypoint_id}' not found in {[wp.name for wp in self.waypoints]}")
            return False
        print(
            f"\n[NAV_ROS] Navigazione verso {target_waypoint.name} a ({target_waypoint.x:.2f}, {target_waypoint.y:.2f})...")
        for attempt in range(1, max_retries + 1):
            print(f"[NAV_ROS] Tentativo {attempt}/{max_retries}...")
            try:
                success = motion_controller.move_to(target_waypoint.x,target_waypoint.y,timeout=timeout)
                if success:
                    print(f"[NAV_ROS] ✓ Arrived at {target_waypoint.name}")
                    return True
                else:
                    print(f"[NAV_ROS] Navigation failed, retry {attempt}/{max_retries}...")
                    time.sleep(1.0)

            except Exception as e:
                print(f"[NAV_ROS] ✗ Exception during navigation: {e}")
                if attempt < max_retries:
                    time.sleep(1.0)
                    continue
                else:
                    return False

        print(f"[NAV_ROS] ✗ Failed to reach waypoint after {max_retries} attempts")
        return False

    def auto_close_loops(self, *args, **kwargs):
        return True

    def optimize_anchoring(self, *args, **kwargs):
        return True

    def find_shortest_cell_path_bfs(self, start_cell, end_cell, env_map):
        """
        Trova il percorso più corto tra due celle usando BFS.
        Solo considera celle visitate (value = 1).

        Args:
            start_cell: (row, col) della cella di partenza
            end_cell: (row, col) della cella di destinazione
            env_map: EnvironmentMap instance

        Returns:
            list: Percorso di celle oppure None se non trovato
        """
        from collections import deque

        print(f"\n[BFS] Ricerca percorso da {start_cell} a {end_cell}")

        start_row, start_col = start_cell
        end_row, end_col = end_cell

        if env_map.map[start_row][start_col] != 1:
            print(f"[BFS] ✗ Cella inizio {start_cell} non visitata")
            return None

        if env_map.map[end_row][end_col] != 1:
            print(f"[BFS] ✗ Cella fine {end_cell} non visitata")
            return None

        queue = deque([(start_cell, [start_cell])])
        visited = {start_cell}

        while queue:
            current_cell, path = queue.popleft()
            current_row, current_col = current_cell

            if current_cell == end_cell:
                print(f"[BFS] ✓ Percorso trovato: {len(path)} celle")
                print(f"[BFS] Cammino: {' → '.join([f'({r},{c})' for r,c in path])}")
                return path

            neighbors = [
                (current_row - 1, current_col),  # Nord
                (current_row, current_col + 1),  # Est
                (current_row + 1, current_col),  # Sud
                (current_row, current_col - 1)   # Ovest
            ]

            for next_row, next_col in neighbors:
                if (0 <= next_row < env_map.rows and
                    0 <= next_col < env_map.cols):

                    next_cell = (next_row, next_col)

                    if (env_map.map[next_row][next_col] == 1 and
                        next_cell not in visited):

                        visited.add(next_cell)
                        queue.append((next_cell, path + [next_cell]))

        print(f"[BFS] ✗ Nessun percorso trovato")
        return None

    def find_waypoints_for_path(self, cell_path):
        """
        Converte un percorso di celle in una lista di waypoint.
        Richiede che ogni cella nel percorso abbia un waypoint associato.

        Args:
            cell_path: Lista di (row, col) del percorso

        Returns:
            list: Lista di waypoint names [wp_0, wp_1, ...] oppure None
        """
        print(f"\n[WAYPOINT_MAPPING] Mapping celle → waypoint ({len(cell_path)} celle)")

        # Crea mappa rapida celle → waypoint
        cell_to_wp = {}
        for wp in self.waypoints:
            if wp.cell_row is not None and wp.cell_col is not None:
                cell_to_wp[(wp.cell_row, wp.cell_col)] = wp.name

        waypoint_path = []
        for i, cell in enumerate(cell_path):
            if cell in cell_to_wp:
                waypoint_path.append(cell_to_wp[cell])
                print(f"  [{i}] Cella {cell} → {cell_to_wp[cell]}")
            else:
                print(f"[WAYPOINT_MAPPING] ✗ Nessun waypoint per cella {cell}")
                return None

        print(f"[WAYPOINT_MAPPING] ✓ Percorso: {' → '.join(waypoint_path)}")
        return waypoint_path

    def navigate_through_waypoint_path(self, waypoint_path, motion_controller, max_retries=3, timeout=30):
        """
        Naviga sequenzialmente attraverso una lista di waypoint.
        Riutilizza navigate_to_waypoint() per ogni waypoint.

        Args:
            waypoint_path: Lista di waypoint names [wp_0, wp_1, ...]
            motion_controller: Controller di movimento ROS
            max_retries: Tentativi per ogni waypoint
            timeout: Timeout per navigazione (secondi)

        Returns:
            bool: True se completato, False se fallito
        """
        print(f"\n{'='*70}")
        print(f"[RETURN_MISSION] Navigazione sequenziale di {len(waypoint_path)} waypoint")
        print(f"[RETURN_MISSION] Percorso: {' → '.join(waypoint_path)}")
        print(f"{'='*70}")

        for i, wp_name in enumerate(waypoint_path, 1):
            print(f"\n[RETURN_MISSION] [{i}/{len(waypoint_path)}] Verso {wp_name}...")

            # Riutilizza navigate_to_waypoint
            success = self.navigate_to_waypoint(
                wp_name,
                motion_controller=motion_controller,
                max_retries=max_retries,
                timeout=timeout
            )

            if not success:
                print(f"[RETURN_MISSION] ✗ Fallita navigazione verso {wp_name}")
                return False

            print(f"[RETURN_MISSION] ✓ Raggiunto {wp_name}")

        print(f"\n{'='*70}")
        print(f"[RETURN_MISSION] ✓ Navigazione completata con successo!")
        print(f"{'='*70}\n")
        return True

    def navigate_to_first_waypoint(self, motion_controller=None, env_map=None, max_retries=3, timeout=30):
        """
        Ritorna al primo waypoint (wp_0) trovando il percorso ottimale.
        MAIN ENTRY POINT per l'end mission in ROS.

        Orchestrazione completa:
        1. Trova percorso BFS dalle celle visitate
        2. Mappa celle a waypoint
        3. Naviga sequenzialmente

        Args:
            motion_controller: Controller di movimento ROS
            env_map: EnvironmentMap con griglia delle celle visitate
            max_retries: Tentativi per ogni navigazione (default: 3)
            timeout: Timeout per navigazione in secondi (default: 30)

        Returns:
            bool: True se successo, False altrimenti
        """
        print(f"\n{'='*70}")
        print(f"[END_MISSION] Procedura di ritorno a casa")
        print(f"[END_MISSION] Obiettivo: wp_0 (punto di partenza)")
        print(f"{'='*70}")

        # Validazione input
        if not self.waypoints:
            print(f"[END_MISSION] ✗ Nessun waypoint registrato")
            return False

        if motion_controller is None:
            print(f"[END_MISSION] ✗ motion_controller non fornito")
            return False

        if env_map is None:
            print(f"[END_MISSION] ✗ env_map non fornito")
            return False

        # Step 1: Determina celle di partenza e attuale
        first_wp = self.waypoints[0]
        start_cell = (first_wp.cell_row, first_wp.cell_col)

        last_wp = self.waypoints[-1]
        current_cell = (last_wp.cell_row, last_wp.cell_col)

        print(f"[END_MISSION] 📍 Cella inizio: {start_cell} ({first_wp.name})")
        print(f"[END_MISSION] 📍 Cella attuale: {current_cell} ({last_wp.name})")
        print(f"[END_MISSION] 📍 Waypoint totali registrati: {len(self.waypoints)}")

        # Se già a casa
        if current_cell == start_cell:
            print(f"[END_MISSION] ✓ Robot già a casa!")
            return True

        # Step 2: BFS per trovare il percorso più corto
        print(f"\n[END_MISSION] Step 1/3: Calcolo percorso ottimale...")
        cell_path = self.find_shortest_cell_path_bfs(current_cell, start_cell, env_map)

        if cell_path is None:
            print(f"[END_MISSION] ✗ Impossibile trovare percorso di ritorno")
            return False

        # Step 3: Mappa celle a waypoint
        print(f"\n[END_MISSION] Step 2/3: Mapping celle → waypoint...")
        waypoint_path = self.find_waypoints_for_path(cell_path)

        if waypoint_path is None:
            print(f"[END_MISSION] ✗ Percorso celle non completamente mappabile a waypoint")
            return False

        # Step 4: Naviga il percorso
        print(f"\n[END_MISSION] Step 3/3: Esecuzione navigazione...")
        success = self.navigate_through_waypoint_path(
            waypoint_path,
            motion_controller,
            max_retries=max_retries,
            timeout=timeout
        )

        if success:
            print(f"[END_MISSION] ✅ RITORNO A CASA COMPLETATO CON SUCCESSO!")
        else:
            print(f"[END_MISSION] ❌ Ritorno a casa fallito durante navigazione")

        return success

    def download_full_graph(self, *args, **kwargs):
        return self._download_filepath
