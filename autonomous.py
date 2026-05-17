class AutoSorter:
    def __init__(self, controller, robot_id, machine_state="Observation"):
        self.controller = controller
        self.robot_id = robot_id
        self.machine_state = machine_state
        self.idle_time = 0
        self.cube_in_memory = None
        self.virtual_joint_id = None
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
    def cube_id_radar(pos_xyz, cube_id):
        """Sprawdza, czy wykryty klocek jest już w pamięci (czyli czy to ten sam klocek, czy nowy)."""
        if self.cube_in_memory is None:
            return True # Pamięć jest pusta, więc każdy klocek jest "nowy"
        
        mem_x, mem_y, mem_z = self.cube_in_memory['xyz']
        pos_x, pos_y, pos_z = pos_xyz
        
        # Sprawdzamy odległość w przestrzeni 3D
        distance = ((mem_x - pos_x) ** 2 + (mem_y - pos_y) ** 2 + (mem_z - pos_z) ** 2) ** 0.5
        
        # Jeśli odległość jest mniejsza niż pewien próg (np. 5 cm), uznajemy, że to ten sam klocek
        return distance > 0.05
    def update(self, detected_cubes):
        if self.machine_state == "Observation":
            if detected_cubes:
                
