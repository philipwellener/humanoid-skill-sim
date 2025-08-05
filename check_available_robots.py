#!/usr/bin/env python3
"""
Check what built-in robot models are available in PyBullet
"""

import pybullet as p
import pybullet_data
import os

def check_available_robots():
    """Check what robot URDF files are available"""
    
    # Connect to PyBullet
    p.connect(p.DIRECT)  # Use DIRECT mode (no GUI) for checking
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Get the PyBullet data path
    data_path = pybullet_data.getDataPath()
    print(f"PyBullet data path: {data_path}")
    print()
    
    # List all URDF files in the data directory
    urdf_files = []
    for root, dirs, files in os.walk(data_path):
        for file in files:
            if file.endswith('.urdf'):
                relative_path = os.path.relpath(os.path.join(root, file), data_path)
                urdf_files.append(relative_path)
    
    print("🤖 Available URDF robot models:")
    print("=" * 50)
    
    # Categorize the files
    humanoid_robots = []
    manipulation_robots = []
    vehicles = []
    other_robots = []
    objects = []
    
    for urdf in sorted(urdf_files):
        urdf_lower = urdf.lower()
        if any(keyword in urdf_lower for keyword in ['humanoid', 'atlas', 'nao', 'pepper']):
            humanoid_robots.append(urdf)
        elif any(keyword in urdf_lower for keyword in ['arm', 'gripper', 'hand', 'manipulator', 'kuka', 'panda', 'ur']):
            manipulation_robots.append(urdf)
        elif any(keyword in urdf_lower for keyword in ['car', 'vehicle', 'truck', 'wheel', 'racecar', 'husky']):
            vehicles.append(urdf)
        elif any(keyword in urdf_lower for keyword in ['robot', 'r2d2', 'drone', 'quadrotor', 'minitaur']):
            other_robots.append(urdf)
        else:
            objects.append(urdf)
    
    if humanoid_robots:
        print("🚶 HUMANOID ROBOTS:")
        for robot in humanoid_robots:
            print(f"  - {robot}")
        print()
    
    if manipulation_robots:
        print("🦾 MANIPULATION ROBOTS (Arms/Grippers):")
        for robot in manipulation_robots:
            print(f"  - {robot}")
        print()
    
    if other_robots:
        print("🤖 OTHER ROBOTS:")
        for robot in other_robots:
            print(f"  - {robot}")
        print()
    
    if vehicles:
        print("🚗 VEHICLES:")
        for robot in vehicles:
            print(f"  - {robot}")
        print()
    
    print("📦 OBJECTS/PRIMITIVES:")
    for obj in objects[:10]:  # Show first 10 objects
        print(f"  - {obj}")
    if len(objects) > 10:
        print(f"  ... and {len(objects) - 10} more objects")
    
    print()
    print(f"📊 TOTAL: {len(urdf_files)} URDF files found")
    
    # Test loading a few popular robots
    print()
    print("🧪 Testing robot loading capabilities:")
    print("=" * 50)
    
    test_robots = [
        "r2d2.urdf",
        "humanoid/nao.urdf", 
        "kuka_iiwa/model.urdf",
        "franka_panda/panda.urdf",
        "quadrotor.urdf",
        "husky/husky.urdf",
        "racecar/racecar.urdf"
    ]
    
    for robot_urdf in test_robots:
        try:
            robot_id = p.loadURDF(robot_urdf, [0, 0, 1])
            num_joints = p.getNumJoints(robot_id)
            
            # Get joint info
            joint_names = []
            for i in range(num_joints):
                joint_info = p.getJointInfo(robot_id, i)
                joint_name = joint_info[1].decode('utf-8')
                joint_type = joint_info[2]
                joint_names.append(f"{joint_name}({joint_type})")
            
            print(f"✅ {robot_urdf}: {num_joints} joints")
            if num_joints > 0:
                print(f"   Joints: {', '.join(joint_names[:5])}")
                if len(joint_names) > 5:
                    print(f"   ... and {len(joint_names) - 5} more")
            
            p.removeBody(robot_id)
            
        except Exception as e:
            print(f"❌ {robot_urdf}: Failed to load - {e}")
    
    p.disconnect()

if __name__ == "__main__":
    check_available_robots()
