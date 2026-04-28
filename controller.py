import pybullet as p
import threading

class UserInputHandler:
    """
    Klasa odpowiedzialna za zbieranie komend od użytkownika.
    Obsługuje zarówno klawiaturę (okno 3D), jak i wpisywanie współrzędnych w terminalu.
    """

    def __init__(self, initial_xyz, robot_id=None):
        self.target_xyz = list(initial_xyz)
        self.gripper_closed = False
        self.mode = "MANUAL"
        self.running = True
        self.step_size = 0.002
        self.slider_ids = []
        self.joint_indices = [] # Poprawka: ujednolicona nazwa
        
        if robot_id is not None:
            self._setup_sliders(robot_id)

    def _setup_sliders(self, robot_id):
        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(robot_id, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            
            if joint_type == p.JOINT_REVOLUTE:
                lower_limit = info[8] # Poprawka: info[8] zamiast info=[8]
                upper_limit = info[9]
                
                if lower_limit >= upper_limit:
                    lower_limit = -3.14
                    upper_limit = 3.14
                    
                slider_id = p.addUserDebugParameter(joint_name, lower_limit, upper_limit, 0)
                self.slider_ids.append(slider_id)
                self.joint_indices.append(i)
    def read_sliders(self):
        joint_angles=[]
        for slider_id in self.slider_ids:
            angle=p.readUserDebugParameter(slider_id)
            joint_angles.append(angle)
        return joint_angles
    def start_terminal_listener(self):
        """Uruchamia asynchroniczny wątek nasłuchujący konsoli."""
        thread = threading.Thread(target=self._terminal_loop, daemon=True)
        thread.start()
        print("\n" + "="*30)
        print("KONTROLER TERMINALOWY AKTYWNY")
        print("Wpisz: X Y Z (np. 0.3 0.1 0.2)")
        print("Wpisz: 'q' aby wyjść z programu.")
        print("="*30 + "\n")

    def _terminal_loop(self):
        """Pętla działająca w tle, parsująca wpisy z konsoli."""
        while self.running:
            try:
                user_input = input("Podaj cel XYZ > ").strip().lower()
                
                if user_input == 'q':
                    self.running = False
                    break
                
                parts = user_input.split()
                if len(parts) == 3:
                    # Mapujemy części tekstu na liczby zmiennoprzecinkowe
                    new_x, new_y, new_z = map(float, parts)
                    self.target_xyz = [new_x, new_y, new_z]
                    print(f"[Terminal] Cel ustawiony na: {self.target_xyz}")
                else:
                    print("[Terminal] Błąd: Musisz podać dokładnie 3 współrzędne.")
            
            except ValueError:
                print("[Terminal] Błąd: Nieprawidłowy format liczb. Używaj kropki jako separatora.")

    def process_keyboard_events(self):
        """
        Sprawdza klawisze w oknie PyBullet.
        Zwraca: String z żądaniem akcji (np. 'RECORD') lub None.
        """
        keys = p.getKeyboardEvents()
        action_request = None

        # --- STEROWANIE CIĄGŁE (Klawisze trzymane: KEY_IS_DOWN) ---
        
        # Oś X (W/S)
        if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
            self.target_xyz[0] += self.step_size
        if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
            self.target_xyz[0] -= self.step_size

        # Oś Y (A/D)
        if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
            self.target_xyz[1] += self.step_size
        if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            self.target_xyz[1] -= self.step_size

        # Oś Z (Q/E)
        if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
            self.target_xyz[2] += self.step_size
        if ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
            self.target_xyz[2] -= self.step_size

        # --- PRZEŁĄCZNIKI (Klawisze puszczone: KEY_WAS_RELEASED) ---

        # Chwytak (Spacja)
        if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_RELEASED:
            self.gripper_closed = not self.gripper_closed
            print(f"[Klawiatura] Chwytak: {'ZAMKNIĘTY' if self.gripper_closed else 'OTWARTY'}")

        # Nagrywanie punktu (Klawisz R)
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_RELEASED:
            action_request = "RECORD"

        # Tryb powtarzania (Klawisz P)
        if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_RELEASED:
            self.mode = "PLAY"
            action_request = "PLAY"

        # Powrót do trybu ręcznego (Klawisz M)
        if ord('m') in keys and keys[ord('m')] & p.KEY_WAS_RELEASED:
            self.mode = "MANUAL"
            action_request = "MANUAL"
            print("[Tryb] Powrót do sterowania ręcznego.")

        return action_request

    def get_state(self):
        """Zwraca obecny stan żądań użytkownika."""
        return self.target_xyz, self.gripper_closed, self.mode