import pinocchio as pin
import os

def verify():
    urdf_dir = "/workspaces/raptor/Robots/unitree-go2"
    files = {
        "go_base.urdf": 6,  # 6-DOF floating base
        "go2_FR.urdf": 3,    # 3-DOF (hip, thigh, calf)
        "go2_FL.urdf": 3,
        "go2_RR.urdf": 3,
        "go2_RL.urdf": 3
    }
    
    all_ok = True
    for name, expected_nv in files.items():
        path = os.path.join(urdf_dir, name)
        print(f"\n--- Verifying {name} ---")
        try:
            # Note: since the meshes are relative, we build the model
            model = pin.buildModelFromUrdf(path)
            print(f"Successfully loaded model from {name}!")
            print(f"Number of joints (nq): {model.nq}")
            print(f"Number of velocity variables (nv): {model.nv}")
            print(f"Joint names: {[model.names[i] for i in range(model.njoints)]}")
            
            if model.nv != expected_nv:
                print(f"ERROR: Expected nv={expected_nv}, but got nv={model.nv}")
                all_ok = False
        except Exception as e:
            print(f"ERROR loading {name}: {e}")
            all_ok = False
            
    if all_ok:
        print("\n=== ALL URDFS ARE CORRECT! ===")
    else:
        print("\n=== SOME URDFS FAILED VERIFICATION ===")

if __name__ == "__main__":
    verify()
