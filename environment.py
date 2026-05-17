import pybullet as p
import pybullet_data 
import time  
import random
class SimulationEnv:
    def __init__(self, use_gui=True):
        self.min_x=0.2
        self.max_x=1.0
        self.min_y= -0.8
        self.max_y=0.8
        self.z=0.05
        self.kolory = {
            "czerwony": [1, 0, 0, 1],
            "zielony": [0, 1, 0, 1],
            "niebieski": [0, 0, 1, 1],
        }
        self.use_gui = use_gui
        if self.use_gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setGravity(0, 0, -9.81)
        print("grawitacja włączona ")

    def load_environment(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        print("załadowano płaszczyznę")
    #stara kostka
    # def spawn_cube(self,position, urdf_path="urdf/cube.urdf"):
    #     cube_id = p.loadURDF(urdf_path, basePosition=position)
    #     print(f"[Środowisko] Załadowano kostkę z pliku {urdf_path} na koordynatach: {position}")
    #     return cube_id
    def spawn_random_cube(self, urdf_path="urdf/cube.urdf"):
        random_x = random.uniform(self.min_x, self.max_x)
        random_y = random.uniform(self.min_y, self.max_y)
        position = [random_x, random_y, self.z]
        cube_id = p.loadURDF(urdf_path, basePosition=position)
        p.changeVisualShape(cube_id, -1, rgbaColor=random.choice(list(self.kolory.values())))
        print(f"[Środowisko] Załadowano kostkę z pliku {urdf_path} na koordynatach: {position}")
        return cube_id, position

    def step_simulation(self):
        p.stepSimulation()
        if self.use_gui:
            time.sleep(1./30.)
    def close(self):
        p.disconnect()
        print("symulacja zakończona")
if __name__ == "__main__":
    moje_srodowisko = SimulationEnv(use_gui=True)
    moje_srodowisko.load_environment()
    
    # Podajemy ścieżkę do naszego nowego pliku (zakładając, że wrzuciłeś go do folderu urdf/)
    ID_kostki = moje_srodowisko.spawn_random_cube(urdf_path="urdf/cube.urdf")
    
    
    print("\nNaciśnij CTRL+C w terminalu, aby zakończyć test...")
    try:
        while True:
            moje_srodowisko.step_simulation()
    except KeyboardInterrupt:
        moje_srodowisko.close()
    