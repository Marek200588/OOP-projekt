import time
import cv2
import pybullet as p
from environment import SimulationEnv
from robot import RobotArm
from controller import UserInputHandler
from teach_pendant import TeachAndRepeat
from vision import VisionSystem

def main():
    print("🚀 Inicjalizacja systemu...")
     
    # ==========================================
    # 1. BUDOWA ŚWIATA (ŚRODOWISKO I OBIEKTY)
    # ==========================================
    env = SimulationEnv(use_gui=True)
    env.load_environment()
    
    sciezka_do_robota = "urdf/ramie.urdf" 
    
    try:
        robot = RobotArm(urdf_path=sciezka_do_robota, start_position=[0, 0, 0])
    except Exception as e:
        print("\n" + "="*50)
        print(f"❌ BŁĄD KRYTYCZNY: Nie mogę załadować robota!")
        print(f"Ścieżka '{sciezka_do_robota}' jest niepoprawna.")
        print("Zmień zmienną 'sciezka_do_robota' w pliku main.py!")
        print("="*50 + "\n")
        env.close()
        return

    # Generujemy kostkę do podnoszenia (upewnij się, że ścieżka do cube.urdf jest poprawna)
    env.spawn_random_cube(urdf_path="urdf/cube.urdf")


    # ==========================================
    # 2. URUCHOMIENIE MÓZGU I PAMIĘCI
    # ==========================================
    start_xyz = [0.2, 0.0, 0.2]
    
    # Przekazujemy robot_id, żeby wygenerowały się nasze suwaki!
    controller = UserInputHandler(initial_xyz=start_xyz, robot_id=robot.robot_id)
    memory = TeachAndRepeat()
    czy_suwak_ruszony = False

    # Odpalamy konsolę terminala w tle
    controller.start_terminal_listener() 
    
    # Zczytujemy pierwszą, początkową pozycję suwaków
    stare_katy_suwakow = controller.read_sliders()
    poprzedni_cel_xyz = list(controller.target_xyz)
    aktywne_sterowanie = "IK"
    print("\n✅ System gotowy. Rozpoczynam pętlę symulacji.")
    print("--- KLAWISZOLOGIA OKNA 3D ---")
    print("U/J/I/K/O/L - Ruch dłonią (Kinematyka Odwrotna)")
    print("Spacja      - Zaciśnij/Puść chwytak")
    print("R           - Zapisz obecny punkt do pamięci (Record)")
    print("P           - Odtwórz nagraną trasę (Play)")
    print("-----------------------------\n")

    poprzedni_cel_xyz = list(controller.target_xyz)

    # ==========================================
    # 3. GŁÓWNA PĘTLA SYMULACJI (SERCE PROGRAMU)
    # ==========================================
    try:
        while controller.running:
            # ODŚWIEŻANIE OKIENKA Z SUWAKAMI
            controller.update_gui()
            
            env.step_simulation()
            action = controller.process_keyboard_events()
            target_xyz, gripper_closed, mode = controller.get_state()

            if action == "RECORD":
                real_pos = robot.get_end_effector_pos()
                if real_pos: memory.record_waypoint(real_pos, gripper_closed)
            if action == "SPAWN_CUBE":
                env.spawn_random_cube(urdf_path="urdf/cube.urdf")
            elif mode == "PLAY":
                print("\n[Main] 🎬 Odtwarzanie...")
                sekwencja = memory.get_sequence()
                if sekwencja:
                    for punkt in sekwencja:
                        katy = robot.calculate_ik(punkt['xyz'])
                        if katy: robot.apply_arm_angles(katy)
                        robot.set_gripper(punkt['gripper'])
                        for _ in range(240): 
                            controller.update_gui() # <--- WAŻNE! Nie zamrażajmy GUI w trakcie odtwarzania
                            env.step_simulation()
                controller.mode = "MANUAL"
                arm_position = robot.get_end_effector_pos()
                if arm_position: controller.target_xyz = list(arm_position)

            # -- RUCH RĘCZNY --
            elif mode == "MANUAL":
                # 1. Zabezpieczenie przed zablokowaniem suwaków: jeśli użyto klawiatury, zdejmij flagę!
                if controller.target_xyz != poprzedni_cel_xyz:
                    controller.reset_slider_flag()
                
                # 2. Tryb SUWAKÓW (uruchamiany z wnętrza Tkinter)
                if controller.suwak_ruszony_przez_usera:
                    aktualne_odczyty = controller.read_sliders()
                    robot.apply_arm_angles(aktualne_odczyty)
                    
                    arm_position = robot.get_end_effector_pos()
                    if arm_position: 
                        controller.target_xyz = list(arm_position)
                
                # 3. Tryb KLAWIATURY (IK)
                else:
                    wyliczone_katy = robot.calculate_ik(controller.target_xyz)
                    if wyliczone_katy:
                        robot.apply_arm_angles(wyliczone_katy)
                        # Animizujemy suwaki w GUI na żywo!
                        controller.update_sliders_from_code(wyliczone_katy)

                poprzedni_cel_xyz = list(controller.target_xyz)
                robot.set_gripper(controller.gripper_closed)
              
    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        print("[Main] Program zakończony.")

if __name__ == "__main__":
    main()