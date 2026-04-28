import time
from environment import SimulationEnv
from robot import RobotArm
from controller import UserInputHandler
from teach_pendant import TeachAndRepeat

def main():
    print("🚀 Inicjalizacja systemu...")
     
    # ==========================================
    # 1. BUDOWA ŚWIATA (ŚRODOWISKO I OBIEKTY)
    # ==========================================
    env = SimulationEnv(use_gui=True)
    env.load_environment()
    
    # WAŻNE: Podmień "urdf/moj_robot.urdf" na DOKŁADNĄ nazwę pliku z Twoim ramieniem!
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
    env.spawn_cube(position=[0.3, 0.0, 0.05], urdf_path="urdf/cube.urdf")


    # ==========================================
    # 2. URUCHOMIENIE MÓZGU I PAMIĘCI
    # ==========================================
    start_xyz = [0.2, 0.0, 0.2]
    # Przekazujemy robot_id, żeby wygenerowały się nasze suwaki!
    controller = UserInputHandler(initial_xyz=start_xyz, robot_id=robot.robot_id)
    memory = TeachAndRepeat()

    # Odpalamy konsolę terminala w tle
    controller.start_terminal_listener()

    print("\n✅ System gotowy. Rozpoczynam pętlę symulacji.")
    print("--- KLAWISZOLOGIA OKNA 3D ---")
    print("W/S/A/D/Q/E - Ruch dłonią (Kinematyka Odwrotna)")
    print("Spacja      - Zaciśnij/Puść chwytak")
    print("R           - Zapisz obecny punkt do pamięci (Record)")
    print("P           - Odtwórz nagraną trasę (Play)")
    print("-----------------------------\n")

    # ==========================================
    # 3. GŁÓWNA PĘTLA SYMULACJI (SERCE PROGRAMU)
    # ==========================================
    try:
        while controller.running:
            # Krok czasu do przodu
            env.step_simulation()
            
            # Nasłuchiwanie klawiatury w oknie PyBullet
            action = controller.process_keyboard_events()
            target_xyz, gripper_closed, mode = controller.get_state()

            # -- REAKCJA NA KLAWISZ "R" --
            if action == "RECORD":
                # Zapisujemy faktyczną, odczytaną z fizyki pozycję dłoni
                real_pos = robot.get_end_effector_pos()
                if real_pos:
                    memory.record_waypoint(real_pos, gripper_closed)

            # -- REAKCJA NA KLAWISZ "P" --
            elif mode == "PLAY":
                print("\n[Main] 🎬 Rozpoczynam odtwarzanie nagranej sekwencji...")
                sekwencja = memory.get_sequence()
                
                if not sekwencja:
                    print("[Main] Sekwencja jest pusta! Wracam do sterowania.")
                else:
                    for punkt in sekwencja:
                        print(f"[Main] Jadę do punktu: {punkt['xyz']}")
                        
                        # Wyliczamy IK dla nagranego punktu
                        katy = robot.calculate_ik(punkt['xyz'])
                        if katy:
                            robot.apply_arm_angles(katy)
                        robot.set_gripper(punkt['gripper'])
                        
                        # Zatrzymujemy się w każdym punkcie na sekundę (240 klatek) 
                        # żebyś widział płynne przeskoki, a nie teleportację
                        for _ in range(240): 
                            env.step_simulation()
                            
                print("[Main] 🛑 Koniec sekwencji. Wracam do sterowania ręcznego.")
                controller.mode = "MANUAL"
                
                # Zabezpieczenie: Po odtworzeniu trasy podmieniamy nasz cel klawiatury
                # na ostatni punkt z trasy, żeby ramię nagle nie "szarpnęło" do starej pozycji
                aktualna_pozycja = robot.get_end_effector_pos()
                if aktualna_pozycja:
                    controller.target_xyz = list(aktualna_pozycja)

            # -- STANDARDOWY RUCH RĘCZNY --
            elif mode == "MANUAL":
                # Ramię podąża za koordynatami XYZ narzucanymi z terminala lub WSADQE
                katy = robot.calculate_ik(target_xyz)
                if katy:
                    robot.apply_arm_angles(katy)
                robot.set_gripper(gripper_closed)

    except KeyboardInterrupt:
        print("\n[Main] Przerwano kombinacją CTRL+C.")
    finally:
        # Kod w bloku finally wykona się zawsze - posprząta po nas i wyłączy silnik
        env.close()
        print("[Main] Program zakończony.")

if __name__ == "__main__":
    main()