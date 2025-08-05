"""
Simulation world environment with objects, terrain, and noise injection
"""

import pybullet as p
import pybullet_data
import numpy as np
import random
from typing import Dict, List, Tuple, Optional
import time


class SimulationWorld:
    """Manages the simulation environment including objects, terrain, and noise"""
    
    def __init__(self, use_gui: bool = True, gravity: float = -15.0):  # Increased gravity for faster falls
        self.use_gui = use_gui
        self.gravity = gravity
        self.objects: Dict[str, int] = {}  # object_id -> pybullet_id
        self.object_positions: Dict[str, List[float]] = {}
        self.physics_client = None
        self.plane_id = None
        self.noise_config = {
            'position_noise': 0.01,  # meters
            'sensor_noise': 0.02,    # measurement noise
            'time_delay': 0.0        # seconds
        }
        
    def initialize(self) -> bool:
        """Initialize PyBullet simulation environment"""
        if self.use_gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
            
        # Set additional search path for URDF files
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Set gravity
        p.setGravity(0, 0, self.gravity)
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Set up camera (if GUI) - positioned to see robot from front-side angle
        if self.use_gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=4.0,      # Move camera back a bit
                cameraYaw=135,           # Rotate to see front-right side of robot (opposite side)
                cameraPitch=-20,         # Slightly elevated view
                cameraTargetPosition=[0, 0, 0]
            )
            
            # Configure GUI for better visibility
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        
        return True
    
    def center_camera_on_scene(self) -> None:
        """Center camera on all objects in the scene"""
        if not self.use_gui or not self.object_positions:
            return
            
        # Get all object positions including robot
        all_positions = []
        
        # Add all object positions
        for pos in self.object_positions.values():
            all_positions.append(pos[:2])  # Just x, y coordinates
            
        # Add robot position if available (assuming robot starts at origin)
        all_positions.append([0, 0])
        
        # Add bin position (commonly at [3, 1])
        all_positions.append([3, 1])
        
        if all_positions:
            # Calculate center of all positions
            all_positions = np.array(all_positions)
            center_x = np.mean(all_positions[:, 0])
            center_y = np.mean(all_positions[:, 1])
            
            # Calculate how far to place camera to see everything
            max_distance = np.max([
                np.linalg.norm([pos[0] - center_x, pos[1] - center_y]) 
                for pos in all_positions
            ])
            
            # Set camera distance to see all objects with closer view
            camera_distance = max(3.5, max_distance * 1.8)  # Reduced from 6.0 and 2.5 for closer zoom
            
            print(f"📷 Centering camera on scene: center=({center_x:.1f}, {center_y:.1f}), distance={camera_distance:.1f}m")
            
            p.resetDebugVisualizerCamera(
                cameraDistance=camera_distance,
                cameraYaw=135,           # Good angle to see the scene
                cameraPitch=-30,         # Elevated view to see everything
                cameraTargetPosition=[center_x, center_y, 0]
            )
    
    def set_camera_view(self, distance: float = 4.0, yaw: float = 135, pitch: float = -20, target: List[float] = None) -> None:
        """Set camera view for better observation"""
        if self.use_gui:
            if target is None:
                target = [0, 0, 0]
            p.resetDebugVisualizerCamera(
                cameraDistance=distance,
                cameraYaw=yaw,
                cameraPitch=pitch,
                cameraTargetPosition=target
            )
            print(f"📷 Camera set: distance={distance}, yaw={yaw}°, pitch={pitch}°")
    
    def follow_robot_camera(self, robot_pos: List[float]) -> None:
        """Update camera to follow robot"""
        if self.use_gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=3.5,
                cameraYaw=-30,
                cameraPitch=-15,
                cameraTargetPosition=robot_pos
            )
            
    def shutdown(self) -> None:
        """Shutdown the simulation"""
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
            
    def step_simulation(self) -> None:
        """Step the physics simulation"""
        p.stepSimulation()
        
    def add_object(self, object_id: str, urdf_path: str, position: List[float], 
                   orientation: List[float] = None) -> int:
        """
        Add an object to the simulation
        
        Args:
            object_id: Unique identifier for the object
            urdf_path: Path to URDF file or built-in name
            position: [x, y, z] position
            orientation: [x, y, z, w] quaternion, defaults to upright
            
        Returns:
            PyBullet object ID
        """
        if orientation is None:
            orientation = [0, 0, 0, 1]
            
        # Add position noise if configured
        noisy_position = self._add_position_noise(position)
        
        try:
            pybullet_id = p.loadURDF(urdf_path, noisy_position, orientation)
            self.objects[object_id] = pybullet_id
            self.object_positions[object_id] = position.copy()
            return pybullet_id
        except Exception as e:
            print(f"Failed to load object {object_id}: {e}")
            return -1
            
    def add_platform(self, platform_id: str, position: List[float], 
                     size: List[float] = None, color: List[float] = None) -> int:
        """
        Add a platform/pedestal to elevate objects to gripper height
        
        Args:
            platform_id: Unique identifier for the platform
            position: [x, y, z] position of platform base
            size: [x, y, z] dimensions, defaults to optimal gripper height platform
            color: [r, g, b, a] color, defaults to brown/wooden color
            
        Returns:
            PyBullet object ID
        """
        if size is None:
            # Ultra-thin, taller platform: 8cm x 8cm x 58cm high (optimal for R2D2 gripper height ~60cm)
            size = [0.04, 0.04, 0.29]  # Half-extents: 8cm x 8cm x 58cm full height
        if color is None:
            color = [0.6, 0.4, 0.2, 1.0]  # Brown/wooden color
            
        # Create platform collision and visual shapes
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
        
        # Platform position (base is at ground level, top at gripper height)
        platform_position = [position[0], position[1], size[2]]  # Raise by half height
        
        # Create multi-body platform
        pybullet_id = p.createMultiBody(
            baseMass=1000.0,  # Extra heavy platform - absolutely won't move even with robot collision
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=platform_position,
            baseOrientation=[0, 0, 0, 1]
        )
        
        self.objects[platform_id] = pybullet_id
        self.object_positions[platform_id] = platform_position.copy()
        
        print(f"  🏛️  Added platform {platform_id} at {platform_position} (top at {platform_position[2] + size[2]:.2f}m)")
        return pybullet_id
        
    def add_object_on_platform(self, object_id: str, platform_id: str, urdf_path: str = None,
                              offset: List[float] = None) -> int:
        """
        Add an object on top of a platform at gripper height
        
        Args:
            object_id: Unique identifier for the object
            platform_id: ID of the platform to place object on
            urdf_path: Path to URDF file, defaults to cube_small.urdf
            offset: [x, y] offset from platform center, defaults to [0, 0]
            
        Returns:
            PyBullet object ID
        """
        if urdf_path is None:
            urdf_path = "cube_small.urdf"
        if offset is None:
            offset = [0.0, 0.0]
            
        # Get platform position and size
        if platform_id not in self.objects:
            print(f"  ❌ Platform {platform_id} not found")
            return -1
            
        platform_pos = self.object_positions[platform_id]
        
        # Platform center is at 0.29m, so top surface is at 0.29 + 0.29 = 0.58m
        # Place object just above the platform top surface
        object_position = [
            platform_pos[0] + offset[0],  # X with offset
            platform_pos[1] + offset[1],  # Y with offset
            platform_pos[2] + 0.29 + 0.02  # Platform top + small clearance = ~60cm height
        ]
        
        print(f"  📦 Placing {object_id} on platform {platform_id} at optimal gripper height: {object_position}")
        
        # Add the object at the calculated position
        return self.add_object(object_id, urdf_path, object_position)

    def add_simple_box(self, object_id: str, position: List[float], 
                       size: List[float] = None, color: List[float] = None) -> int:
        """
        Add a simple box object
        
        Args:
            object_id: Unique identifier
            position: [x, y, z] position
            size: [x, y, z] dimensions, defaults to [0.1, 0.1, 0.1]
            color: [r, g, b, a] color, defaults to random
            
        Returns:
            PyBullet object ID
        """
        if size is None:
            size = [0.1, 0.1, 0.1]
        if color is None:
            color = [random.random(), random.random(), random.random(), 1.0]
            
        # Create box collision shape
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
        
        # Add position noise
        noisy_position = self._add_position_noise(position)
        
        # Create multi-body
        pybullet_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=noisy_position
        )
        
        self.objects[object_id] = pybullet_id
        self.object_positions[object_id] = position.copy()
        return pybullet_id
        
    def remove_object(self, object_id: str) -> bool:
        """Remove an object from the simulation"""
        if object_id in self.objects:
            p.removeBody(self.objects[object_id])
            del self.objects[object_id]
            del self.object_positions[object_id]
            return True
        return False
        
    def get_object_position(self, object_id: str) -> Optional[List[float]]:
        """Get the current position of an object with sensor noise"""
        if object_id not in self.objects:
            return None
            
        pybullet_id = self.objects[object_id]
        pos, _ = p.getBasePositionAndOrientation(pybullet_id)
        
        # Add sensor noise
        noisy_pos = self._add_sensor_noise(list(pos))
        
        # Simulate time delay
        if self.noise_config['time_delay'] > 0:
            time.sleep(self.noise_config['time_delay'])
            
        return noisy_pos
        
    def get_object_orientation(self, object_id: str) -> Optional[List[float]]:
        """Get the current orientation of an object"""
        if object_id not in self.objects:
            return None
            
        pybullet_id = self.objects[object_id]
        _, orn = p.getBasePositionAndOrientation(pybullet_id)
        return list(orn)
        
    def set_object_position(self, object_id: str, position: List[float]) -> bool:
        """Manually set object position (for teleop override)"""
        if object_id not in self.objects:
            return False
            
        pybullet_id = self.objects[object_id]
        _, current_orn = p.getBasePositionAndOrientation(pybullet_id)
        p.resetBasePositionAndOrientation(pybullet_id, position, current_orn)
        self.object_positions[object_id] = position.copy()
        return True
        
    def create_fetch_and_place_scenario(self) -> None:
        """Set up a typical fetch and place scenario with platform-elevated objects"""
        print("🏛️ Creating elevated fetch-and-place scenario...")
        
        # Create platforms for objects at optimal gripper height - spread out more to avoid collisions
        self.add_platform("platform_1", [1.5, 1.0, 0.0])   # Spread further apart
        self.add_platform("platform_2", [1.2, 0.0, 0.0])   # Positioned to allow smart routing without excessive distance
        self.add_platform("platform_3", [1.5, -1.0, 0.0])  # Symmetric spacing
        
        # Add boxes on platforms at gripper height for easy picking
        self.add_object_on_platform("box_01", "platform_1", "cube_small.urdf")  # Red box
        self.add_object_on_platform("box_02", "platform_2", "cube_small.urdf")  # Green box
        self.add_object_on_platform("box_03", "platform_3", "cube_small.urdf")  # Blue box
        
        # Add target bins (further away) - keep these on ground as they are targets, not pickup items
        self.add_simple_box("bin_1", [3.0, 1.0, 0.1], [0.2, 0.2, 0.1], [0.5, 0.5, 0.5, 1.0])
        self.add_simple_box("bin_2", [3.0, 0.0, 0.1], [0.2, 0.2, 0.1], [0.5, 0.5, 0.5, 1.0])
        self.add_simple_box("bin_3", [3.0, -1.0, 0.1], [0.2, 0.2, 0.1], [0.5, 0.5, 0.5, 1.0])
        
        print("✅ Elevated scenario created - all pickup objects now at optimal gripper height!")
        
        # Center camera on the entire scene
        self.center_camera_on_scene()
        
    def inject_object_drift(self, object_id: str, drift_magnitude: float = 0.02) -> bool:
        """Simulate object drift during manipulation"""
        if object_id not in self.objects:
            return False
            
        current_pos = self.get_object_position(object_id)
        if current_pos is None:
            return False
            
        # Add random drift
        drift = [
            random.uniform(-drift_magnitude, drift_magnitude),
            random.uniform(-drift_magnitude, drift_magnitude),
            0  # Don't drift in Z
        ]
        
        new_pos = [current_pos[i] + drift[i] for i in range(3)]
        return self.set_object_position(object_id, new_pos)
        
    def set_noise_config(self, position_noise: float = None, sensor_noise: float = None, 
                        time_delay: float = None) -> None:
        """Configure noise parameters"""
        if position_noise is not None:
            self.noise_config['position_noise'] = position_noise
        if sensor_noise is not None:
            self.noise_config['sensor_noise'] = sensor_noise
        if time_delay is not None:
            self.noise_config['time_delay'] = time_delay
            
    def _add_position_noise(self, position: List[float]) -> List[float]:
        """Add noise to position during object placement"""
        noise_mag = self.noise_config['position_noise']
        noise = [random.uniform(-noise_mag, noise_mag) for _ in range(3)]
        return [position[i] + noise[i] for i in range(3)]
        
    def _add_sensor_noise(self, measurement: List[float]) -> List[float]:
        """Add noise to sensor measurements"""
        noise_mag = self.noise_config['sensor_noise']
        noise = [random.uniform(-noise_mag, noise_mag) for _ in range(len(measurement))]
        return [measurement[i] + noise[i] for i in range(len(measurement))]
        
    def get_all_objects(self) -> List[str]:
        """Get list of all object IDs"""
        return list(self.objects.keys())
    
    def get_objects(self) -> List[str]:
        """Get list of all object IDs (alias for get_all_objects)"""
        return self.get_all_objects()
    
    def spawn_object(self, object_type: str, position: List[float], 
                     size: List[float] = None, color: List[float] = None) -> str:
        """
        Spawn an object in the world
        
        Args:
            object_type: Type of object to spawn ('box', 'sphere', etc.)
            position: [x, y, z] position
            size: Object dimensions
            color: Object color [r, g, b, a]
            
        Returns:
            Object ID string
        """
        # Generate unique object ID
        object_id = f"{object_type}_{len(self.objects):03d}"
        
        if object_type == "box":
            pybullet_id = self.add_simple_box(object_id, position, size, color)
        else:
            # Default to box for now
            pybullet_id = self.add_simple_box(object_id, position, size, color)
            
        if pybullet_id >= 0:
            return object_id
        else:
            return None
        
    def reset_scene(self) -> None:
        """Reset the scene by removing all objects"""
        for object_id in list(self.objects.keys()):
            self.remove_object(object_id)
    
    def reset(self) -> None:
        """Reset the scene (alias for reset_scene)"""
        self.reset_scene()
