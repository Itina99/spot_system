"""ROS-compatible stubs for GraphNav utilities.

These keep the EasyWalk flow running in simulation without GraphNav.
"""
from dataclasses import dataclass
import time
import math
@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    cell_row: int | None = None
    cell_col: int | None = None
    previous_waypoint: str | None = None

class RecordingInterface:
    def __init__(self, *args, **kwargs):
        self.waypoints = []
        self.waypoint_index = {}
        self.current_waypoint_name = None
        self._download_filepath = None
        self.navigation_path = []  # [wp_0, wp_1, wp_2, ...]
        self.waypoint_adjacency = {}  # {wp_0: [wp_1, wp_3], wp_1: [wp_0, wp_2], ...}

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
        if not self.waypoints:
            wp = Waypoint(name=name,x=x, y=y, z=z, yaw=yaw,cell_row=cell_row, cell_col=cell_col)

            self.waypoint_adjacency[name] = []
        else:
            previous_wp_name = self.current_waypoint_name
            print(f"[GRAPH] Creating {name}" f" from {previous_wp_name}")
            wp = Waypoint(name=name, x=x, y=y, z=z, yaw=yaw, cell_row=cell_row, cell_col=cell_col, previous_waypoint=previous_wp_name)
            print(
                f"[GRAPH DEBUG] current_waypoint_name = "
                f"{self.current_waypoint_name}"
            )
            if previous_wp_name not in self.waypoint_adjacency:
                self.waypoint_adjacency[previous_wp_name] = []

            if name not in self.waypoint_adjacency:
                self.waypoint_adjacency[name] = []

            # arco forward
            self.waypoint_adjacency[previous_wp_name].append(name)

            # arco backward
            self.waypoint_adjacency[name].append(previous_wp_name)
        self.waypoints.append(wp)
        self.waypoint_index[wp.name] = wp
        self.navigation_path.append(name)
        self.current_waypoint_name = name
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
            print(
                f"[NAV] Current waypoint = "
                f"{self.current_waypoint_name}"
            )
            try:
                success = motion_controller.move_to(target_waypoint.x,target_waypoint.y,timeout=timeout)
                if success:
                    print(f"[NAV_ROS] ✓ Arrived at {target_waypoint.name}")
                    self.current_waypoint_name= waypoint_id
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

    def can_connect_waypoints(self,wp_a,wp_b,obstacle_grid,safety_margin=0.40,samples_per_meter=10):
        dx = wp_b.x - wp_a.x
        dy = wp_b.y - wp_a.y
        distance = math.hypot(dx, dy)
        n_samples = max(2,int(distance * samples_per_meter))

        for i in range(n_samples + 1):
            t = i / n_samples
            x = wp_a.x + t * dx
            y = wp_a.y + t * dy
            clearance = obstacle_grid.distance_at(x,y)
            if clearance < safety_margin:
                return False
        return True

    def auto_close_loops(self,local_distance,max_connection_distance=4.0,safety_margin=0.40):

        obstacle_grid = local_distance.obstacle_grid
        added_edges = 0

        print("\n[LOOP_CLOSURE] Ricerca nuove connessioni...")

        for i in range(len(self.waypoints)):
            wp_a = self.waypoints[i]
            for j in range(i + 1, len(self.waypoints)):
                wp_b = self.waypoints[j]
                # evita vicini già collegati
                if (
                    wp_b.name
                    in self.waypoint_adjacency.get(
                        wp_a.name,
                        []
                    )
                ):
                    continue
                distance = math.hypot( wp_a.x - wp_b.x, wp_a.y - wp_b.y)

                if distance > max_connection_distance:
                    continue

                if not self.can_connect_waypoints(wp_a,wp_b,obstacle_grid,safety_margin=safety_margin):
                    continue

                self.waypoint_adjacency.setdefault(wp_a.name,[])
                self.waypoint_adjacency.setdefault(wp_b.name,[])
                self.waypoint_adjacency[wp_a.name].append(wp_b.name)
                self.waypoint_adjacency[wp_b.name].append(wp_a.name)
                added_edges += 1

                print(
                    f"[LOOP_CLOSURE] "
                    f"{wp_a.name} <-> {wp_b.name} "
                    f"(dist={distance:.2f})"
                )
        print(
            f"[LOOP_CLOSURE] "
            f"Aggiunti {added_edges} archi"
        )
        return added_edges

    def optimize_anchoring(self, *args, **kwargs):
        return True

    def find_shortest_cell_path_bfs(self, start_cell, end_cell, env_map):
        """
        Trova il percorso più corto tra due celle sulla griglia statica.
        Usa BFS considerando SOLO celle libere (ostacoli evitati).

        Args:
            start_cell: (row, col) della cella di partenza
            end_cell: (row, col) della cella di destinazione
            env_map: EnvironmentMap con griglia statica

        Returns:
            list: Percorso di celle [(r,c), ...] oppure None se non trovato
        """
        from collections import deque

        print(f"\n[BFS] Ricerca percorso libero da {start_cell} a {end_cell}")

        start_row, start_col = start_cell
        end_row, end_col = end_cell

        # Valida celle sulla griglia
        if not (0 <= start_row < env_map.rows and 0 <= start_col < env_map.cols):
            print(f"[BFS] ✗ Start cell {start_cell} fuori dalla griglia")
            return None

        if not (0 <= end_row < env_map.rows and 0 <= end_col < env_map.cols):
            print(f"[BFS] ✗ End cell {end_cell} fuori dalla griglia")
            return None

        # BFS sulla griglia: naviga su celle libere (>= 0)
        queue = deque([(start_cell, [start_cell])])
        visited = {start_cell}

        while queue:
            current_cell, path = queue.popleft()
            current_row, current_col = current_cell

            if current_cell == end_cell:
                print(f"[BFS] ✓ Percorso trovato: {len(path)} celle")
                print(f"[BFS] Cammino: {' → '.join([f'({r},{c})' for r, c in path])}")
                return path

            # 4 vicini (N, E, S, W)
            neighbors = [
                (current_row - 1, current_col),  # Nord
                (current_row, current_col + 1),  # Est
                (current_row + 1, current_col),  # Sud
                (current_row, current_col - 1)  # Ovest
            ]

            for next_row, next_col in neighbors:
                next_cell = (next_row, next_col)

                # Verifica bounds
                if not (0 <= next_row < env_map.rows and 0 <= next_col < env_map.cols):
                    continue

                # Naviga su celle libere (0) o visitate (1), evita ostacoli
                cell_value = env_map.map[next_row][next_col]

                if cell_value >= 0 and next_cell not in visited:
                    visited.add(next_cell)
                    queue.append((next_cell, path + [next_cell]))

        print(f"[BFS] ✗ Nessun percorso libero trovato")
        return None

    def navigate_through_cell_path(self,cell_path,motion_controller,env_map, timeout=30):
        """
        Segue direttamente il percorso BFS cella per cella.

        Args:
            cell_path: [(row,col), ...]
            motion_controller: controller ROS
            env_map: EnvironmentMap
            timeout: timeout move_to

        Returns:
            bool
        """

        if not cell_path:
            print("[CELL_NAV] ✗ Percorso vuoto")
            return False

        print(f"\n{'=' * 70}")
        print(f"[CELL_NAV] Navigazione su {len(cell_path)} celle")
        print(f"{'=' * 70}")

        #
        # Saltiamo la prima cella
        # perché è la cella in cui il robot si trova già
        #
        for idx, (row, col) in enumerate(cell_path[1:], start=1):
            world_pos = env_map.get_world_position_from_cell(row, col)
            if world_pos is None:
                print(f"[CELL_NAV] ✗ Cella ({row},{col}) fuori mappa")
                return False
            target_x, target_y = world_pos
            print(f"[CELL_NAV] [{idx}/{len(cell_path) - 1}] "
                f"cella ({row},{col}) -> ({target_x:.2f}, {target_y:.2f})")
            success = motion_controller.move_to(target_x,target_y,timeout=timeout)

            if not success:
                print(f"[CELL_NAV] ✗ Fallito raggiungimento "f"cella ({row},{col})")
                return False

        print(f"[CELL_NAV] ✓ Percorso completato")
        return True

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

    def navigate_to_first_waypoint(self, current_pos = None, motion_controller=None, env_map=None, max_retries=3, timeout=30):
        """
        Ritorna a wp_0 usando il percorso libero sulla griglia statica.

        Orchestrazione:
        1. Trova cella di wp_0
        2. Ricerca BFS su griglia statica per percorso libero
        3. Converte celle in waypoint
        4. Naviga sequenzialmente

        Args:
            motion_controller: Controller di movimento ROS
            env_map: EnvironmentMap con griglia statica
            max_retries: Tentativi per ogni waypoint
            timeout: Timeout per navigazione (secondi)

        Returns:
            bool: True se successo, False altrimenti
            :param current_pos:
        """
        print(f"\n{'=' * 70}")
        print(f"[END_MISSION] Procedura di ritorno a casa")
        print(f"[END_MISSION] Obiettivo: wp_0 (punto di partenza)")
        print(f"{'=' * 70}")


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
        home_cell = (first_wp.cell_row, first_wp.cell_col)

        current_cell = env_map.get_cell_from_world(current_pos[0], current_pos[1])

        print(f"[END_MISSION] 📍 Cella home: {home_cell} ({first_wp.name})")
        print(f"[END_MISSION] 📍 Cella attuale: {current_cell}")
        print(f"[END_MISSION] 📍 Waypoint totali: {len(self.waypoints)}")

        # Se già a casa
        if current_cell == home_cell:
            print(f"[END_MISSION] ✓ Robot già a casa!")
            return True

        # Step 2: Ricerca percorso libero sulla griglia statica
        print(f"\n[END_MISSION] Step 1/2: Ricerca percorso libero sulla griglia...")

        current_wp = self.current_waypoint_name
        path = self.find_waypoint_path(current_wp,"wp_0")
        if path is None:
            print("[END_MISSION] Nessun percorso sul grafo")
            return False
        success = self.navigate_through_waypoint_path(path,motion_controller,max_retries=max_retries,timeout=timeout)

        if success:
            print(f"[END_MISSION] ✅ RITORNO A CASA COMPLETATO!")
        else:
            print(f"[END_MISSION] ❌ Ritorno a casa fallito")

        return success

    def download_full_graph(self, *args, **kwargs):
        return self._download_filepath

    def find_waypoint_path(self, start_waypoint_name, target_waypoint_name):
        from collections import deque
        print(
            f"\n[GRAPH_BFS] Ricerca percorso "
            f"{start_waypoint_name} -> {target_waypoint_name}"
        )

        if start_waypoint_name not in self.waypoint_adjacency:
            print(f"[GRAPH_BFS] Start node non trovato")
            return None

        if target_waypoint_name not in self.waypoint_adjacency:
            print(f"[GRAPH_BFS] Target node non trovato")
            return None
        queue = deque()
        queue.append((start_waypoint_name, [start_waypoint_name]))
        visited = {start_waypoint_name}
        while queue:
            current_node, path = queue.popleft()
            if current_node == target_waypoint_name:
                print(
                    f"[GRAPH_BFS] ✓ Path trovato: "
                    f"{' -> '.join(path)}"
                )
                return path
            neighbors = self.waypoint_adjacency.get(current_node,[])
            for neighbor in neighbors:
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                queue.append((neighbor,path + [neighbor]))
        print("[GRAPH_BFS] ✗ Nessun percorso trovato")
        return None

    def backtrack_to_waypoint(self,target_waypoint_name,motion_controller,max_retries=3,timeout=30):
        if not self.waypoints:
            return False
        current_waypoint = self.current_waypoint_name
        path = self.find_waypoint_path(current_waypoint,target_waypoint_name)

        if path is None:
            print(
                f"[BACKTRACK] impossibile raggiungere "
                f"{target_waypoint_name}"
            )
            return False
        success = self.navigate_through_waypoint_path(path,motion_controller,max_retries=max_retries,timeout=timeout)

        if success:
            self.current_waypoint_name = target_waypoint_name
            print(
                f"[BACKTRACK DEBUG] current waypoint updated to "
                f"{self.current_waypoint_name}"
            )

        return success