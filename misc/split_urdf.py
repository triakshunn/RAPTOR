import xml.etree.ElementTree as ET
import os

def split_urdf():
    urdf_path = "../Robots/unitree-go1/go1.urdf"
    output_dir = "../Robots/unitree-go1"

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    legs = {
        "FR": "1_FR",
        "FL": "2_FL",
        "RR": "3_RR",
        "RL": "4_RL"
    }

    # 1. Generate base URDF (only base, trunk, and imu)
    base_tree = ET.parse(urdf_path)
    base_root = base_tree.getroot()
    base_root.set("name", "go1_base")

    # Change floating_base joint type to floating for base identification
    for joint in base_root.findall("joint"):
        if joint.get("name") == "floating_base":
            joint.set("type", "floating")

    # Remove all leg-related elements (joints, links, transmissions)
    for elem in list(base_root):
        name = elem.get("name", "")
        # Remove if it belongs to any leg
        is_leg = any(prefix in name for prefix in ["1_FR", "2_FL", "3_RR", "4_RL"])
        # Remove transmissions or gazebo tags referencing legs
        is_leg_ref = False
        if elem.tag == "gazebo":
            ref = elem.get("reference", "")
            if any(prefix in ref for prefix in ["1_FR", "2_FL", "3_RR", "4_RL"]):
                is_leg_ref = True
        
        if is_leg or is_leg_ref or elem.tag == "transmission":
            base_root.remove(elem)
            
        # Also remove other camera/ultrasound to be clean, except imu
        if elem.tag in ["joint", "link"] and name not in ["base", "trunk", "imu_joint", "imu_link", "floating_base"]:
            if any(prefix in name for prefix in ["1_FR", "2_FL", "3_RR", "4_RL"]) or "camera" in name or "ultraSound" in name or "mac_mini" in name:
                try:
                    base_root.remove(elem)
                except ValueError:
                    pass

    base_tree.write(os.path.join(output_dir, "go1_base.urdf"), encoding="utf-8", xml_declaration=True)
    print("Generated go1_base.urdf")

    # 2. Generate per-leg URDFs
    for leg_name, prefix in legs.items():
        leg_tree = ET.parse(urdf_path)
        leg_root = leg_tree.getroot()
        leg_root.set("name", f"go1_{leg_name}")

        # Keep only elements belonging to this leg, base elements, and IMU
        for elem in list(leg_root):
            name = elem.get("name", "")
            
            # Determine if this element should be kept
            is_base_element = name in ["base", "trunk", "imu_joint", "imu_link", "floating_base"]
            is_this_leg = prefix in name
            
            # Gazebo reference check
            is_gazebo_reference_ok = False
            if elem.tag == "gazebo":
                ref = elem.get("reference", "")
                if ref in ["imu_link", "trunk", "base"] or prefix in ref:
                    is_gazebo_reference_ok = True
            
            # We keep:
            # - Materials (which have no name in the same way or are materials)
            # - Gazebo plugins that are global (no reference attribute) or reference base/this leg
            # - Base joints/links/imu
            # - Joints/links/transmissions for this leg
            
            keep = False
            if elem.tag == "material":
                keep = True
            elif elem.tag == "gazebo":
                if elem.get("reference") is None or is_gazebo_reference_ok:
                    keep = True
            elif is_base_element or is_this_leg:
                keep = True
                
            if not keep:
                try:
                    leg_root.remove(elem)
                except ValueError:
                    pass

        # Write to file
        output_path = os.path.join(output_dir, f"go1_{leg_name}.urdf")
        leg_tree.write(output_path, encoding="utf-8", xml_declaration=True)
        print(f"Generated {output_path}")

if __name__ == "__main__":
    split_urdf()
