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
        
        # BEZPIECZNA PAMIĘĆ: Zamiast None, wstawiamy bezpieczne dane domyślne.
        # Dzięki temu kod wyciągający np. ["wspolrzedne_xyz"] nigdy nie wyrzuci błędu!
        self.cube_in_memory = {
            "kolor": "red", 
            "wspolrzedne_xyz": [0.2, 0.0, 0.05],
            "piksele": (0, 0)
        }
        
        # Zmienna do zapamiętywania pozycji startowej dla płynnych łuków
        self.start_xyz = [0.2, 0.0, 0.3]
        
        # Miejsca odkładania klocków
        self.stack_xyz = {
            "red": (-0.4, -0.3),
            "green": (-0.4, 0.0),
            "blue": (-0.4, 0.3)
        }
        
        self.stack_tracker = {
            "red": 0,
            "green": 0,
            "blue": 0
        }

    def polar_interpolate(self, p1, p2, t):
        """Interpolacja cylindryczna (po łuku) z wygładzaniem (easing)."""
        t = (1 - math.cos(t * math.pi)) / 2.0

        r1 = math.hypot(p1[0], p1[1])
        a1 = math.atan2(p1[1], p1[0])
        z1 = p1[2]

        r2 = math.hypot(p2[0], p2[1])
        a2 = math.atan2(p2[1], p2[0])
        z2 = p2[2]

        diff = a2 - a1
        diff = (diff + math.pi) % (2 * math.pi) - math.pi

        rt = r1 + (r2 - r1) * t
        at = a1 + diff * t
        zt = z1 + (z2 - z1) * t

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
            
            # Jeśli kamera cokolwiek widzi, nadpisujemy bezpieczną pamięć tym klockiem!
            if detected_cubes and len(detected_cubes) > 0:
                self.cube_in_memory = detected_cubes[0]
                print(f"[AutoSorter] Znalazłem: {self.cube_in_memory['kolor']}. Zaczynamy!")
                self.machine_state = "Approach"
                self.idle_time = 0

        # === STAN 2: DOJAZD NAD KLOCEK ===
        elif self.machine_state == "Approach":
            if self.idle_time == 0:
                self.start_xyz = list(self.controller.target_xyz)
            
            target_x, target_y, _ = self.cube_in_memory["wspolrzedne_xyz"]
            end_xyz = [target_x, target_y, 0.25]
            max_time = 10 
            
            self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
            
            self.idle_time += 1
            if self.idle_time >= max_time:
                self.machine_state = "Descent"
                self.idle_time = 0

        # === STAN 3: ZJAZD W DÓŁ ===
        elif self.machine_state == "Descent":
            if self.idle_time == 0:
                self.start_xyz = list(self.controller.target_xyz)
                
            target_x, target_y, _ = self.cube_in_memory["wspolrzedne_xyz"]
            end_xyz = [target_x, target_y, 0.08] 
            max_time = 5 
            
            self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
            
            self.idle_time += 1
            if self.idle_time >= max_time:
                self.machine_state = "Grab"
                self.idle_time = 0

        # === STAN 4: CHWYTANIE I SPAWANIE ===
        elif self.machine_state == "Grab":
            self.controller.gripper_closed = True
            
            if self.idle_time == 4:
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
            if self.idle_time > 8:
                self.machine_state = "Ascent"
                self.idle_time = 0

        # === STAN 5: PODNOSZENIE ===
        elif self.machine_state == "Ascent":
            if self.idle_time == 0:
                self.start_xyz = list(self.controller.target_xyz)
                
            target_x, target_y, _ = self.cube_in_memory["wspolrzedne_xyz"]
            end_xyz = [target_x, target_y, 0.35]
            max_time = 5
            
            self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
            
            self.idle_time += 1
            if self.idle_time >= max_time:
                self.machine_state = "Move_To_Stack"
                self.idle_time = 0

        # === STAN 6: LOT NAD STOS ===
        elif self.machine_state == "Move_To_Stack":
            if self.idle_time == 0:
                self.start_xyz = list(self.controller.target_xyz)
                
            color = self.cube_in_memory["kolor"]
            stack_x, stack_y = self.stack_xyz.get(color, (-0.4, 0.0)) 
            end_xyz = [stack_x, stack_y, 0.35]
            max_time = 12
            
            self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
            
            self.idle_time += 1
            if self.idle_time >= max_time:
                self.machine_state = "Lower_To_Stack"
                self.idle_time = 0

        # === STAN 7: OPUSZCZANIE NA STOS ===
        elif self.machine_state == "Lower_To_Stack":
            if self.idle_time == 0:
                self.start_xyz = list(self.controller.target_xyz)
                
            color = self.cube_in_memory["kolor"]
            stack_x, stack_y = self.stack_xyz.get(color, (-0.4, 0.0))
            cubes_on_stack = self.stack_tracker.get(color, 0)
            
            target_z = 0.05 + (cubes_on_stack * 0.1) + 0.03
            end_xyz = [stack_x, stack_y, target_z]
            max_time = 8
            
            self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
            
            self.idle_time += 1
            if self.idle_time >= max_time:
                self.machine_state = "Release"
                self.idle_time = 0

        # === STAN 8: PUSZCZANIE ===
        elif self.machine_state == "Release":
            self.controller.gripper_closed = False
            
            if self.idle_time == 0:
                if self.virtual_joint_id is not None:
                    p.removeConstraint(self.virtual_joint_id)
                    self.virtual_joint_id = None
                
                color = self.cube_in_memory["kolor"]
                if color in self.stack_tracker:
                    self.stack_tracker[color] += 1
                print(f"[AutoSorter] Sukces! Stos {color} liczy {self.stack_tracker.get(color)} klocków.")

            self.idle_time += 1
            if self.idle_time > 5:
                self.machine_state = "Return"
                self.idle_time = 0

        # === STAN 9: POWRÓT ===
        elif self.machine_state == "Return":
            if self.idle_time == 0:
                self.start_xyz = list(self.controller.target_xyz)
                
            color = self.cube_in_memory["kolor"]
            stack_x, stack_y = self.stack_xyz.get(color, (-0.4, 0.0))
            end_xyz = [stack_x, stack_y, 0.35] 
            max_time = 6
            
            self.controller.target_xyz = self.polar_interpolate(self.start_xyz, end_xyz, self.idle_time / max_time)
            
            self.idle_time += 1
            if self.idle_time >= max_time:
                self.machine_state = "Observation"
                self.idle_time = 0
                # NIE resetujemy pamięci. Zostaje tam klocek, który właśnie podnieśliśmy,
                # aż do momentu gdy stan 1. nadpisze go nowym.