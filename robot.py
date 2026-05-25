import pybullet as p
import time

class RobotArm:
    def __init__(self, urdf_path, start_position=[0, 0, 0]): # Poprawka: dodano przecinki
        self.robot_id = p.loadURDF(urdf_path, basePosition=start_position, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.movable_joints = []
        self.arm_joints = []
        self.gripper_joints = []
        self.ee_index = -1
        self._scan_joints() # Poprawka: dodano podłogę i 's' na końcu

    def _scan_joints(self): # Poprawka: 's' na końcu
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            p.changeDynamics(self.robot_id, i, jointDamping=2.0)
            info = p.getJointInfo(self.robot_id, i)
            joint_type = info[2]
            link_name = info[12].decode('utf-8')
        
            # Poprawka: Wcięcia! Te 'ify' muszą być wewnątrz pętli for
            if link_name == "hand": # Poprawka: == zamiast =
                self.ee_index = i
                
            if joint_type != p.JOINT_FIXED:
                self.movable_joints.append(i) # Poprawka: kropka zamiast podłogi
                
                if joint_type == p.JOINT_REVOLUTE:
                    self.arm_joints.append(i)
                elif joint_type == p.JOINT_PRISMATIC:
                    self.gripper_joints.append(i)
                    
        print(f"[Robot] Wykryto {len(self.arm_joints)} stawów ramienia i {len(self.gripper_joints)} palców chwytaka.")
    
    def set_gripper(self, is_closed):
        """Zamyka (True) lub otwiera (False) chwytak."""
        target_pos = 0.035 if is_closed else 0.0
        
        for joint_idx in self.gripper_joints:
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos,
                force=100 # Siła zacisku w Newtonach
            )

    def calculate_ik(self, target_xyz):
        if self.ee_index == -1: # Poprawka: == zamiast ===
            print("[Robot] Błąd: Nie można znaleźć indeksu efektora końcowego.")
            return None
            
        ik_angles_all = p.calculateInverseKinematics(self.robot_id, self.ee_index, target_xyz)
        arm_angles = []
        
        # Poprawka: użyto enumerate, aby pętla działała prawidłowo
        for i, joint_idx in enumerate(self.movable_joints):
            if joint_idx in self.arm_joints:
                arm_angles.append(ik_angles_all[i]) # Poprawka: indeksujemy po 'i', nie 'joint_idx'
                
        return arm_angles

    def apply_arm_angles(self, angles):
        for i, joint_idx in enumerate(self.arm_joints):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angles[i],
                force=500,
                positionGain=0.2,
                velocityGain=1.0,
                maxVelocity=2.0
            )

    def get_end_effector_pos(self):
        if self.ee_index == -1:
            print("[Robot] Błąd: Nie można znaleźć indeksu efektora końcowego.")
            return None
            
        state = p.getLinkState(self.robot_id, self.ee_index)
        return list(state[0]) # Poprawka: zamieniamy krotkę (tuple) na listę dla wygody