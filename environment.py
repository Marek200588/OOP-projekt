import pybullet as p
import pybullet_data 
import time  
class SimulationEnv:
    def __init__(self, use_gui=True):

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
    
    def spawn_cube(self,position, urdf_path="urdf/cube.urdf"):
        cube_id = p.loadURDF(urdf_path, basePosition=position)
        print(f"[Środowisko] Załadowano kostkę z pliku {urdf_path} na koordynatach: {position}")
        return cube_id
     
    def step_simulation(self):
        p.stepSimulation()
        if self.use_gui:
            time.sleep(1./240.)
    def close(self):
        p.disconnect()
        print("symulacja zakończona")
if __name__ == "__main__":
    moje_srodowisko = SimulationEnv(use_gui=True)
    moje_srodowisko.load_environment()
    
    # Podajemy ścieżkę do naszego nowego pliku (zakładając, że wrzuciłeś go do folderu urdf/)
    ID_kostki = moje_srodowisko.spawn_cube(position=[0.3, 0.0, 0.1], urdf_path="urdf/cube.urdf")
    
    print("\nNaciśnij CTRL+C w terminalu, aby zakończyć test...")
    try:
        while True:
            moje_srodowisko.step_simulation()
    except KeyboardInterrupt:
        moje_srodowisko.close()
    