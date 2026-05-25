import pybullet as p
import threading
import tkinter as tk

class UserInputHandler:
    def __init__(self, initial_xyz, robot_id=None):
        self.target_xyz = list(initial_xyz)
        self.gripper_closed = False
        self.mode = "MANUAL"
        self.running = True
        self.step_size = 0.01
        self.joint_indices = []
        
        # --- 1. TWORZENIE WŁASNEGO OKNA GUI ---
        self.root = tk.Tk()
        self.root.title("Panel Sterowania Ramieniem")
        self.root.geometry("350x450")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Wyświetlacz aktualnego trybu
        self.lbl_mode = tk.Label(self.root, text="TRYB: KLAWIATURA (IK)", font=("Arial", 12, "bold"), fg="green")
        self.lbl_mode.pack(pady=10)
        
        self.tk_vars = []
        self.suwak_ruszony_przez_usera = False
        self.ignore_slider_events = False
        
        if robot_id is not None:
            self._setup_tk_sliders(robot_id)

    def on_closing(self):
        """Kiedy użytkownik kliknie X na okienku GUI."""
        self.running = False
        self.root.destroy()

    def _setup_tk_sliders(self, robot_id):
        num_joints = p.getNumJoints(robot_id)

        # Funkcja (closure), która wie, kiedy to my ciągniemy suwak
        def make_callback():
            def on_move(val):
                # Ignorujemy zdarzenie, jeśli to sam kod przesuwa suwak z IK!
                if not self.ignore_slider_events:
                    self.suwak_ruszony_przez_usera = True
                    self.lbl_mode.config(text="TRYB: SUWAKI (RĘCZNY)", fg="red")
            return on_move

        for i in range(num_joints):
            info = p.getJointInfo(robot_id, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            
            if joint_type == p.JOINT_REVOLUTE:
                lower_limit = info[8]
                upper_limit = info[9]
                if lower_limit >= upper_limit:
                    lower_limit = -3.14
                    upper_limit = 3.14
                    
                self.joint_indices.append(i)
                
                var = tk.DoubleVar()
                slider = tk.Scale(self.root, variable=var, from_=lower_limit, to=upper_limit,
                                  resolution=0.01, orient=tk.HORIZONTAL, label=f"Przegub: {joint_name}",
                                  length=300, command=make_callback())
                slider.pack(pady=5)
                self.tk_vars.append(var)

    def read_sliders(self):
        """Zwraca obecne położenie suwaków z okna."""
        return [var.get() for var in self.tk_vars]

    def update_sliders_from_code(self, angles):
        """Animuje suwaki w GUI podczas jazdy z klawiatury."""
        self.ignore_slider_events = True
        for var, angle in zip(self.tk_vars, angles):
            var.set(angle)
        self.ignore_slider_events = False

    def reset_slider_flag(self):
        """Resetuje flagę, aby oddać sterowanie klawiaturze."""
        self.suwak_ruszony_przez_usera = False
        self.lbl_mode.config(text="TRYB: KLAWIATURA (IK)", fg="green")

    def update_gui(self):
        """Musi być odpalane w pętli symulacji, odświeża okno."""
        if self.running:
            try:
                self.root.update_idletasks() # Lepsze do płynnych suwaków
                self.root.update()
            except tk.TclError:
                self.running = False
    def set_mode(self, mode):
        self.mode = mode
        if mode == "MANUAL":
            self.lbl_mode.config(text="TRYB: KLAWIATURA (IK)", fg="green")
        elif mode == "AUTONOMOUS":
            self.lbl_mode.config(text="TRYB: AUTO (SORTOWANIE)", fg="blue")
        elif mode == "PLAY":
            self.lbl_mode.config(text="TRYB: ODTWARZANIE", fg="orange")
        else:
            self.lbl_mode.config(text=f"TRYB: {mode}", fg="black")

    # ============================================================
    # STARY KOD TERMINALA I KLAWIATURY (Zaktualizowane klawisze)
    # ============================================================
    def start_terminal_listener(self):
        thread = threading.Thread(target=self._terminal_loop, daemon=True)
        thread.start()

    def _terminal_loop(self):
        while self.running:
            try:
                user_input = input("Podaj cel XYZ > ").strip().lower()
                if user_input == 'q':
                    self.running = False
                    break
                parts = user_input.split()
                if len(parts) == 3:
                    new_x, new_y, new_z = map(float, parts)
                    self.target_xyz = [new_x, new_y, new_z]
            except ValueError:
                pass

    def process_keyboard_events(self):
        keys = p.getKeyboardEvents()
        action_request = None
        
        # Prawa strona klawiatury (X, Y, Z)
        if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN: self.target_xyz[0] += self.step_size
        if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN: self.target_xyz[0] -= self.step_size
        if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN: self.target_xyz[1] += self.step_size
        if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN: self.target_xyz[1] -= self.step_size
        if ord('o') in keys and keys[ord('o')] & p.KEY_IS_DOWN: self.target_xyz[2] += self.step_size
        if ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN: self.target_xyz[2] -= self.step_size

        if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_RELEASED:
            self.gripper_closed = not self.gripper_closed
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_RELEASED:
            action_request = "RECORD"
        if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_RELEASED:
            self.set_mode("PLAY")
            action_request = "PLAY"
        if ord('m') in keys and keys[ord('m')] & p.KEY_WAS_RELEASED:
            self.set_mode("MANUAL")
            action_request = "MANUAL"
        if ord('a') in keys and keys[ord('a')] & p.KEY_WAS_RELEASED:
            self.set_mode("AUTONOMOUS")
            action_request = "AUTONOMOUS"
        if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_RELEASED:
            action_request = "SPAWN_CUBE"
        return action_request
        #spawn kostki
        
         
    def get_state(self):
        return self.target_xyz, self.gripper_closed, self.mode