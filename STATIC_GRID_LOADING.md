# Caricamento della Griglia Statica - Schema Step by Step

## STEP 1: IL CARICAMENTO INIZIALE
- Quando il sistema parte, nella funzione `easy_walk()` di `easy_walk_ros.py` (linea 1251), viene chiamata la funzione `load_static_grid('worlds/test.sdf')`
- Questa è l'unica volta che accade durante l'intero ciclo di vita del programma
- La funzione riceve il percorso al file SDF (Simulation Description Format) che è un file XML che descrive il mondo virtuale di Gazebo

## STEP 2: IL PARSING DEL FILE SDF
- La funzione `load_static_grid` chiama `parse_sdf_obstacles(sdf_path, ignore)` per aprire e leggere il file XML
- Per ogni elemento `<model>` nel file XML, estrae:
  - Il nome del modello (ad esempio "obstacle_1", "wall_2")
  - La posizione nel mondo (x, y, z coordinate)
  - L'orientamento (roll, pitch, yaw)
  - Le dimensioni della scatola (larghezza, altezza, profondità)
- Alcuni modelli vengono ignorati: il piano di terra ("ground_plane") e il robot stesso ("spot")
- Al termine di questo step, hai una lista di oggetti `BoxObstacle` che rappresentano tutti gli ostacoli nel mondo

## STEP 3: CALCOLO DELLA SPECIFICA DELLA GRIGLIA
- Calcola l'intervallo della griglia usando `calculate_grid_range(obstacles)` determinando l'area di interesse
- Crea un oggetto `GridSpec` che contiene:
  - La dimensione della griglia in celle (60x60 nel tuo caso)
  - L'intervallo del mondo (quanto spazio fisico copre la griglia)
  - La risoluzione calcolata automaticamente (quanti metri per cella)

## STEP 4: COSTRUZIONE DELLA GRIGLIA DI OCCUPAZIONE
- Chiama `build_occupancy_grid(spec, obstacles)` per costruire una griglia 2D di 60x60 celle
- Per ogni ostacolo nell'elenco, calcola quali celle della griglia intersecano quell'ostacolo e le marca come occupate (valore 100)
- Le celle non occupate rimangono a 0 (libere)
- Tiene conto della rotazione degli ostacoli usando l'algoritmo AABB (Axis-Aligned Bounding Box)

## STEP 5: APPIATTIMENTO E MEMORIZZAZIONE NEL CACHE
- La griglia 2D (una lista di liste) viene appiattita in una lista lineare singola
- Un oggetto `StaticGridCache` viene creato e contiene:
  - `grid_data`: la lista appiattita di 3600 elementi (60 * 60)
  - `width` e `height`: entrambi 60
  - `resolution`: il numero di metri per cella (typicamente ~0.1m)
  - `origin_x` e `origin_y`: le coordinate del mondo dell'angolo inferiore sinistro della griglia
- Questo cache viene salvato nella variabile `static_cache` dentro il nodo ROS `EasyWalkROSNode` (linea 1251-1252)

## STEP 6: WRAPPING IN LocalDistanceField
- Il cache viene passato a un oggetto `LocalDistanceField` creato alla linea 1252
- All'interno del costruttore di `LocalDistanceField`, il cache statico viene processato
- La griglia appiattita viene passata a `ObstacleGrid.from_grid_data()` che la trasforma in un campo di distanza con segno
- Questo significa che non solo sa se una cella è occupata, ma calcola anche la distanza minima da qualsiasi ostacolo per ogni punto nello spazio

## STEP 7: UTILIZZO DURANTE L'ESECUZIONE
- Durante l'esecuzione del programma, quando il nodo ROS ha bisogno di verificare se un percorso è libero
- Viene chiamato il metodo `is_free(x, y, threshold)` per verificare se un punto in coordinate mondo è in spazio libero
- Internamente, converte le coordinate mondo in coordinate della griglia, estrae il valore di distanza dal campo di distanza
- Se la distanza dal punto all'ostacolo più vicino è maggiore di -0.15 metri, il punto è considerato libero e sicuro per il movimento del robot

## RIEPILOGO DELLA POSIZIONE DEL DATO
- **File caricato**: `worlds/test.sdf`
- **Salvato in**: `self.local_distance` (istanza di `LocalDistanceField` nel nodo ROS)
- **Usato per**: Verificare velocemente se un percorso è libero e sicuro senza dover aspettare i dati del LIDAR (SLAM) di Gazebo
- **Quando**: Durante tutto il ciclo di esplorazione e navigazione del robot, ogni volta che deve decidere se un percorso è praticabile

