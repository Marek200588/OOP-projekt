import pybullet as p
import math

class AutoSorter:
    def __init__(self, controller, robot_id, ee_index=7, machine_state="Observation"):
        self.controller = controller
        self.robot_id = robot_id
        self.ee_index = ee_index 
        self.machine_state = machine_state
        self.idle_time = 0
        self.virtual_joint_id = None
        
        # BEZPIECZNA PAMIĘĆ: żeby nie zapominać o zlokalizowanych klockach
        self.cube_in_memory = {
            "kolor": "czerwony", 
            "wspolrzedne_xyz": [0.2, 0.0, 0.05],
            "piksele": (0, 0)
        }
        
        # Miejsca odkładania klocków z dopasowanymi nazwami
        self.stack_xyz = {
            "czerwony": (-0.4, -0.3),
            "zielony": (-0.4, 0.0),
            "niebieski": (-0.4, 0.3)
        }
        
        # Liczniki 
        self.stack_tracker = {
            "czerwony": 0,
            "zielony": 0,
            "niebieski": 0
        }
    def polar_interpolate(self, p1, p2, t, arc_height=0.0):
            """Interpolacja cylindryczna (po łuku) z wygładzaniem i podrzutem w osi Z bo próbował szorować po ziemi."""
            t_eased = (1 - math.cos(t * math.pi)) / 2.0

            r1 = math.hypot(p1[0], p1[1])
            a1 = math.atan2(p1[1], p1[0])
            z1 = p1[2]

            r2 = math.hypot(p2[0], p2[1])
            a2 = math.atan2(p2[1], p2[0])
            z2 = p2[2]

            diff = a2 - a1
            diff = (diff + math.pi) % (2 * math.pi) - math.pi

            rt = r1 + (r2 - r1) * t_eased
            at = a1 + diff * t_eased
            
            # TWORZENIE ŁUKU: W połowie drogi (t_eased = 0.5) sinus(pi*0.5) daje 1, więc Z rośnie o arc_height. 
            # Na początku (0) i końcu (1) sinus daje 0.
            z_arc = math.sin(t_eased * math.pi) * arc_height
            zt = z1 + (z2 - z1) * t_eased + z_arc

            xt = rt * math.cos(at)
            yt = rt * math.sin(at)
            return [xt, yt, zt]
    def cube_id_radar(self, pos_xyz):
        """Skanuje fizykę PyBullet w poszukiwaniu ID klocka do spawania."""
        x, y, z = pos_xyz
        margin = 0.05
        
        overlapping_objects = p.getOverlappingObjects(
            [x - margin, y - margin, z - margin], 
            [x + margin, y + margin, z + margin]
        )
        
        if overlapping_objects:
            for obj in overlapping_objects:
                obj_id = obj[0] 
                if obj_id != self.robot_id and obj_id != 0:
                    return obj_id
        return None

    def update(self, detected_cubes):
            # === STAN 1: OBSERWACJA ===
            if self.machine_state == "Observation":
                self.controller.target_xyz = [0.2, 0.0, 0.3]
                self.controller.gripper_closed = False
                
                # Jeśli cokolwiek widzimy, zamrażamy to i przechodzimy dalej
                if detected_cubes and len(detected_cubes) > 0:
                    # Pobieramy klocek i kopiujemy jego dane na "sztywno", odcinając się od aktualizacji z kamery by nie lokalizowałą
                    #kolejnych
                    znaleziony_klocek = detected_cubes[0]
                    self.cube_in_memory = {
                        "kolor": znaleziony_klocek["kolor"],
                        # Twarda kopia listy współrzędnych, nikt nam już tego nie nadpisze:
                        "wspolrzedne_xyz": list(znaleziony_klocek["wspolrzedne_xyz"]) 
                    }
                    
                    print(f"[AutoSorter] Zlokalizowano i ZAMROŻONO cel: {self.cube_in_memory['kolor']}. Start!")
                    self.machine_state = "Approach"
                    self.idle_time = 0

            # === STAN 2: DOJAZD NAD KLOCEK ===
            elif self.machine_state == "Approach":
                self.controller.gripper_closed = False # Upewniamy się, że chwytak jest otwarty
                
                if self.idle_time == 0:
                    self.start_xyz = list(self.controller.target_xyz)
                
                target_x, target_y, _ = self.cube_in_memory["wspolrzedne_xyz"]
                end_xyz = [target_x, target_y, 0.25]
                max_time = 150 # Ok. 0.6 sekundy płynnego ruchu by nie szarpało
                
                self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
                
                self.idle_time += 1
                if self.idle_time >= max_time:
                    self.machine_state = "Descent"
                    self.idle_time = 0

            # === STAN 3: ZJAZD W DÓŁ ===
            elif self.machine_state == "Descent":
                self.controller.gripper_closed = False
                
                if self.idle_time == 0:
                    self.start_xyz = list(self.controller.target_xyz)
                    
                target_x, target_y, _ = self.cube_in_memory["wspolrzedne_xyz"]
                end_xyz = [target_x, target_y, 0.08] # Wysokość chwytania
                max_time = 80 # Szybszy zjazd w dół (ok. 0.3s)
                
                self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
                
                self.idle_time += 1
                if self.idle_time >= max_time:
                    self.machine_state = "Grab"
                    self.idle_time = 0

            # === STAN 4: CHWYTANIE I SPAWANIE ===
            elif self.machine_state == "Grab":
                self.controller.gripper_closed = True # Zaciskamy!
                
                # Dajemy fizyce chwilę (np. 30 klatek) na faktyczne zaciśnięcie palców przed stworzeniem spawu- virtual joint
                #w urdf żeby nie było opcji na wyślizgnięcie się
                if self.idle_time == 30:
                    target_id = self.cube_id_radar(self.controller.target_xyz)
                    if target_id is not None:
                        self.virtual_joint_id = p.createConstraint(
                            parentBodyUniqueId=self.robot_id,
                            parentLinkIndex=self.ee_index,
                            childBodyUniqueId=target_id,
                            childLinkIndex=-1,
                            jointType=p.JOINT_FIXED,
                            jointAxis=[0, 0, 0],
                            parentFramePosition=[0, 0, 0],
                            childFramePosition=[0, 0, 0]
                        )

                self.idle_time += 1
                if self.idle_time > 60: # Czekamy łącznie ćwierć sekundy na pewny chwyt
                    self.machine_state = "Ascent"
                    self.idle_time = 0

            # === STAN 5: PODNOSZENIE ===
            elif self.machine_state == "Ascent":
                self.controller.gripper_closed = True # PODTRZYMANIE CHWYTU!
                
                if self.idle_time == 0:
                    self.start_xyz = list(self.controller.target_xyz)
                    
                target_x, target_y, _ = self.cube_in_memory["wspolrzedne_xyz"]
                end_xyz = [target_x, target_y, 0.35]
                max_time = 80 
                
                self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
                
                self.idle_time += 1
                if self.idle_time >= max_time:
                    self.machine_state = "Move_To_Stack"
                    self.idle_time = 0

            # === STAN 6: LOT NAD STOS ===
            elif self.machine_state == "Move_To_Stack":
                self.controller.gripper_closed = True # PODTRZYMANIE CHWYTU!
                
                if self.idle_time == 0:
                    self.start_xyz = list(self.controller.target_xyz)
                    
                color = self.cube_in_memory["kolor"]
                stack_x, stack_y = self.stack_xyz.get(color, (-0.4, 0.0)) 
                end_xyz = [stack_x, stack_y, 0.35]
                max_time = 200 # Dłuższy lot (ok. 0.8 sekundy), powód jak wyżej
                
                self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time, arc_height=0.15)
                
                self.idle_time += 1
                if self.idle_time >= max_time:
                    self.machine_state = "Lower_To_Stack"
                    self.idle_time = 0

            # === STAN 7: OPUSZCZANIE NA STOS ===
            elif self.machine_state == "Lower_To_Stack":
                self.controller.gripper_closed = True # PODTRZYMANIE CHWYTU nadal xd
                
                if self.idle_time == 0:
                    self.start_xyz = list(self.controller.target_xyz)
                    
                color = self.cube_in_memory["kolor"]
                stack_x, stack_y = self.stack_xyz.get(color, (-0.4, 0.0))
                cubes_on_stack = self.stack_tracker.get(color, 0)
                
                target_z = 0.05 + (cubes_on_stack * 0.1) + 0.03
                end_xyz = [stack_x, stack_y, target_z]
                max_time = 100
                
                self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
                
                self.idle_time += 1
                if self.idle_time >= max_time:
                    self.machine_state = "Release"
                    self.idle_time = 0

            # === STAN 8: PUSZCZANIE ===
            elif self.machine_state == "Release":
                self.controller.gripper_closed = False # Puszczamy kloc
                
                if self.idle_time == 0:
                    if self.virtual_joint_id is not None:
                        p.removeConstraint(self.virtual_joint_id)
                        self.virtual_joint_id = None
                    
                    color = self.cube_in_memory["kolor"]
                    if color in self.stack_tracker:
                        self.stack_tracker[color] += 1
                    print(f"[AutoSorter] Sukces! Stos {color} liczy {self.stack_tracker.get(color)} klocków.")

                self.idle_time += 1
                if self.idle_time > 60: # Dajemy czas na otwarcie chwytaka
                    self.machine_state = "Lift_From_Stack" 
                    self.idle_time = 0

            # === STAN 9: podniesienie chwytaka znad stosu by nie próbował przejść przez siebie samegp ===
            elif self.machine_state == "Lift_From_Stack":
                self.controller.gripper_closed = False
                
                if self.idle_time == 0:
                    self.start_xyz = list(self.controller.target_xyz)
                    
                color = self.cube_in_memory["kolor"]
                stack_x, stack_y = self.stack_xyz.get(color, (-0.4, 0.0))
                end_xyz = [stack_x, stack_y, 0.35]
                max_time = 80
                
                self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
                
                self.idle_time += 1
                if self.idle_time >= max_time:
                    self.machine_state = "Return_To_Home"
                    self.idle_time = 0

            # === STAN 10: POWRÓT DO BAZY ===
            elif self.machine_state == "Return_To_Home":
                self.controller.gripper_closed = False
                
                if self.idle_time == 0:
                    self.start_xyz = list(self.controller.target_xyz)
                    
                end_xyz = [0.2, 0.0, 0.3] 
                max_time = 150 
                
                self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time, arc_height=0.15)
                
                self.idle_time += 1
                if self.idle_time >= max_time:
                    self.machine_state = "Observation"
                    self.idle_time = 0   