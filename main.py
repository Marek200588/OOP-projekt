import time
import cv2
import pybullet as p
from environment import SimulationEnv
from robot import RobotArm
from controller import UserInputHandler
from teach_pendant import TeachAndRepeat
from vision import VisionSystem
from autonomous import AutoSorter  # Upewnij się, że pierwszy kod wkleiłeś do pliku autonomous.py

def main():
    print("🚀 Inicjalizacja systemu...")
     
    # ==========================================
    # 1. BUDOWA ŚWIATA 
    # ==========================================
    env = SimulationEnv(use_gui=True)
    env.load_environment()
    
    sciezka_do_robota = "urdf/ramie.urdf" 
    try:
        robot = RobotArm(urdf_path=sciezka_do_robota, start_position=[0, 0, 0])
    except Exception as e:
        print(f"❌ BŁĄD KRYTYCZNY: Nie mogę załadować robota z pliku {sciezka_do_robota}")
        env.close()
        return

    # Na start spawniemy od razu 3 losowe kostki na stole
    for _ in range(3):
        env.spawn_random_cube(urdf_path="urdf/cube.urdf")

    # ==========================================
    # 2. INICJALIZACJA MÓZGU I SYSTEMÓW
    # ==========================================
    start_xyz = [0.2, 0.0, 0.2]
    controller = UserInputHandler(initial_xyz=start_xyz, robot_id=robot.robot_id)
    memory = TeachAndRepeat()
    
    # Kamera wizyjna
    vision = VisionSystem(camera_pos=[0.5, 0.0, 0.8], target_pos=[0.5, 0.0, 0.0])
    
    # Twój nowy, zaawansowany Sorter oparty na maszynie stanów!
    # Używamy robot.ee_index, aby automatycznie dopasować indeks chwytaka z URDF
    sorter = AutoSorter(controller=controller, robot_id=robot.robot_id, ee_index=robot.ee_index)

    controller.start_terminal_listener() 
    poprzedni_cel_xyz = list(controller.target_xyz)

    print("\n✅ System gotowy. Rozpoczynam pętlę symulacji.")
    print("--- KLAWISZOLOGIA OKNA 3D ---")
    print("U/J/I/K/O/L - Ruch dłonią (Kinematyka Odwrotna)")
    print("Spacja      - Zaciśnij/Puść chwytak")
    print("C           - Stwórz nową kostkę")
    print("A           - 🤖 AUTONOMICZNE SORTOWANIE (State Machine)")
    print("M           - Wróć do trybu MANUALNEGO")
    print("R/P         - Nagraj (Record) / Odtwórz (Play) trasę")
    print("-----------------------------\n")

    # ==========================================
    # 3. GŁÓWNA PĘTLA SYMULACJI
    # ==========================================
    try:
        # Wymuszamy stworzenie okna OpenCV już na samym starcie programu
        cv2.namedWindow("Oko Robota (Wizja)")
        ostatni_tryb = "MANUAL" # Zmienna do wykrywania momentu zmiany trybu
        while controller.running:
            # Odświeżanie GUI i Fizyki
            controller.update_gui()
            env.step_simulation()
            # (main.py - gdzieś na początku pętli while)
            action = controller.process_keyboard_events()
            target_xyz, gripper_closed, mode = controller.get_state()
            
            # ---> DODAJ TO:
            # ==========================================
            # PODGLĄD KAMERY NA ŻYWO (ZAWSZE WŁĄCZONY)
            # ==========================================
            img = vision.get_image()
            blocks = vision.detect_blocks(img)
            # ---> DODAJ TĘ LINIJKĘ DO DIAGNOSTYKI:
            
            podglad = vision.draw_detections(img, blocks)
            cv2.imshow("Oko Robota (Wizja)", podglad)
            cv2.waitKey(1) # To powstrzymuje kamerę przed zawieszeniem się!
            

            # --- Akcje jednorazowe ---
            if action == "RECORD":
                real_pos = robot.get_end_effector_pos()
                if real_pos: memory.record_waypoint(real_pos, gripper_closed)
            elif action == "SPAWN_CUBE":
                env.spawn_random_cube(urdf_path="urdf/cube.urdf")
                
            # --- Odtwarzanie nagranej trasy ---
            elif mode == "PLAY":
                sekwencja = memory.get_sequence()
                if sekwencja:
                    for punkt in sekwencja:
                        katy = robot.calculate_ik(punkt['xyz'])
                        if katy: robot.apply_arm_angles(katy)
                        robot.set_gripper(punkt['gripper'])
                        for _ in range(30): # Pamiętaj o 30Hz!
                            controller.update_gui()
                            env.step_simulation()
                controller.mode = "MANUAL"
                arm_position = robot.get_end_effector_pos()
                if arm_position: controller.target_xyz = list(arm_position)

            # ==========================================
            # ==========================================
            # TRYB AUTONOMICZNEGO SORTOWANIA (STATE MACHINE)
            # ==========================================
            elif mode == "AUTONOMOUS":
                # 1. Inicjalizacja przy pierwszym wejściu w tryb AUTO
                if ostatni_tryb != "AUTONOMOUS":
                    print("🤖 START AUTONOMII! Zmieniam stan na Observation.")
                    sorter.machine_state = "Observation"
                    ostatni_tryb = "AUTONOMOUS"
                
                # 2. BEZWARUNKOWE wywołanie update'u Sortera
                # Usunąłem warunki blokujące - teraz MUSI wejść do Sortera!
                sorter.update(blocks)
                
                # 3. Zabezpieczenie przed błędem - jeśli Sorter zmienił stan na Observation, 
                # a na stole nie ma klocków przez dłuższą chwilę, wracamy do ręcznego
                if sorter.machine_state == "Observation" and len(blocks) == 0:
                    print("⚠️ Brak klocków! Automatyczny powrót do trybu MANUAL.")
                    controller.mode = "MANUAL"
                    arm_pos = robot.get_end_effector_pos()
                    if arm_pos: 
                        controller.target_xyz = list(arm_pos)
                else:
                    # 4. Aplikowanie wyliczonej pozycji na robota
                    wyliczone_katy = robot.calculate_ik(controller.target_xyz)
                    if wyliczone_katy:
                        robot.apply_arm_angles(wyliczone_katy)
                        # To pozwala suwakom w oknie podążać za automatem:
                        controller.update_sliders_from_code(wyliczone_katy)
                    
                    # Aplikowanie chwytaka
                    robot.set_gripper(controller.gripper_closed)
            # ==========================================
            # TRYB RĘCZNY (MANUAL)
            # ==========================================
            elif mode == "MANUAL":
                if controller.target_xyz != poprzedni_cel_xyz:
                    controller.reset_slider_flag()
                
                if controller.suwak_ruszony_przez_usera:
                    aktualne_odczyty = controller.read_sliders()
                    robot.apply_arm_angles(aktualne_odczyty)
                    arm_position = robot.get_end_effector_pos()
                    if arm_position: controller.target_xyz = list(arm_position)
                else:
                    wyliczone_katy = robot.calculate_ik(controller.target_xyz)
                    if wyliczone_katy:
                        robot.apply_arm_angles(wyliczone_katy)
                        controller.update_sliders_from_code(wyliczone_katy)

                poprzedni_cel_xyz = list(controller.target_xyz)
                robot.set_gripper(controller.gripper_closed)
                ostatni_tryb = mode
    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        cv2.destroyAllWindows()
        print("[Main] Program zakończony.")

if __name__ == "__main__":
    main()