# Diagnostic Code for Laser Transformation Issue
# Add this cell to your notebook BEFORE the main simulation loop

"""
DIAGNOSTIC: Laser Transformation Verification

This cell adds comprehensive debugging to identify the root cause of
scattered laser points. Run this BEFORE the main loop to understand
what's happening with the coordinate transformations.
"""

import numpy as np

def diagnose_laser_transformation(controller, iteration_to_check=10):
    """
    Diagnostic function to verify laser-to-world transformation.
    
    This function will:
    1. Check sensor mounting orientation
    2. Verify raw laser data ranges
    3. Compare manual vs. automatic transformation
    4. Identify any coordinate frame mismatches
    """
    print("\n" + "="*70)
    print("LASER TRANSFORMATION DIAGNOSTIC")
    print("="*70)
    
    # Step 1: Check sensor mounting in scene
    print("\n1. SENSOR MOUNTING CONFIGURATION")
    print("-" * 70)
    
    try:
        # Get sensor handle
        laser_handle = controller.sim.getObject('/Kobuki/fastHokuyo')
        robot_handle = controller.robot_handle
        
        # Get sensor position and orientation RELATIVE TO ROBOT
        laser_pos_rel = controller.sim.getObjectPosition(laser_handle, robot_handle)
        laser_ori_rel = controller.sim.getObjectOrientation(laser_handle, robot_handle)
        
        print(f"Laser position relative to robot: {np.array(laser_pos_rel)}")
        print(f"Laser orientation relative to robot (radians): {np.array(laser_ori_rel)}")
        print(f"Laser orientation relative to robot (degrees): {np.rad2deg(laser_ori_rel)}")
        
        # CRITICAL: Check if laser is rotated
        laser_yaw_deg = np.rad2deg(laser_ori_rel[2])
        print(f"\n⚠️  SENSOR YAW (Z-rotation): {laser_yaw_deg:.1f}°")
        
        if abs(laser_yaw_deg) < 5:
            print("   ✅ Sensor aligned with robot (pointing forward)")
            sensor_offset = 0.0
        elif abs(laser_yaw_deg - 90) < 5:
            print("   ⚠️  Sensor rotated 90° LEFT relative to robot!")
            sensor_offset = np.pi/2
        elif abs(laser_yaw_deg + 90) < 5:
            print("   ⚠️  Sensor rotated 90° RIGHT relative to robot!")
            sensor_offset = -np.pi/2
        elif abs(abs(laser_yaw_deg) - 180) < 5:
            print("   ⚠️  Sensor rotated 180° (pointing backward)!")
            sensor_offset = np.pi
        else:
            print(f"   ⚠️  Sensor has unusual rotation: {laser_yaw_deg:.1f}°")
            sensor_offset = laser_ori_rel[2]
        
        print(f"\n   Required offset in transformation: {np.rad2deg(sensor_offset):.1f}°")
        
    except Exception as e:
        print(f"   ❌ Error checking sensor mounting: {e}")
        sensor_offset = None
    
    # Step 2: Check raw sensor data
    print("\n2. RAW SENSOR DATA VERIFICATION")
    print("-" * 70)
    
    # Run simulation for a few steps to get data
    controller.sim.startSimulation()
    for _ in range(5):
        controller.sim.step()
    
    # Get one laser scan
    laser_data = controller.get_laser_data(debug=False)
    
    if laser_data is not None:
        angles = laser_data[:, 0]
        distances = laser_data[:, 1]
        
        print(f"Number of points: {len(laser_data)}")
        print(f"Angle range: [{np.rad2deg(angles.min()):7.1f}°, {np.rad2deg(angles.max()):7.1f}°]")
        print(f"Distance range: [{distances.min():5.3f}m, {distances.max():5.3f}m]")
        
        print(f"\n⚠️  EXPECTED for Hokuyo:")
        print(f"   Angles: [-120.0° to +120.0°] (240° FOV)")
        print(f"   Distances: [0.0m to 5.0m]")
        
        # Check if angles are as expected
        expected_min = np.deg2rad(-120)
        expected_max = np.deg2rad(120)
        
        if abs(angles.min() - expected_min) < 0.1 and abs(angles.max() - expected_max) < 0.1:
            print(f"\n   ✅ Angle range matches expected Hokuyo FOV")
        else:
            print(f"\n   ❌ Angle range DOES NOT match expected!")
            print(f"      Difference: min={np.rad2deg(angles.min() - expected_min):.1f}°, "
                  f"max={np.rad2deg(angles.max() - expected_max):.1f}°")
    
    # Step 3: Compare transformations
    print("\n3. TRANSFORMATION COMPARISON (First 5 Points)")
    print("-" * 70)
    
    # Get robot pose
    robot_pose = controller.get_pose()
    x_robot, y_robot, theta_robot = controller.get_pose_2d()
    
    print(f"Robot pose: x={x_robot:.3f}m, y={y_robot:.3f}m, θ={np.rad2deg(theta_robot):.1f}°")
    
    # Transform using current method
    from utils.tp3_utils import transform_laser_to_global
    laser_points_current = transform_laser_to_global(robot_pose, laser_data[:5])
    
    # Transform using corrected method (WITH sensor offset if found)
    if sensor_offset is not None:
        print(f"\nApplying sensor offset: {np.rad2deg(sensor_offset):.1f}°")
        laser_points_corrected = []
        for beam_angle, range_measurement in laser_data[:5]:
            world_angle = theta_robot + beam_angle + sensor_offset
            x_world = x_robot + range_measurement * np.cos(world_angle)
            y_world = y_robot + range_measurement * np.sin(world_angle)
            laser_points_corrected.append([x_world, y_world])
        laser_points_corrected = np.array(laser_points_corrected)
    else:
        laser_points_corrected = laser_points_current
    
    print(f"\n{'Point':<6} {'Angle':<8} {'Dist':<7} {'Current X':<10} {'Current Y':<10} "
          f"{'Corrected X':<12} {'Corrected Y':<12} {'Delta':<10}")
    print("-" * 100)
    
    for i in range(min(5, len(laser_data))):
        angle_deg = np.rad2deg(laser_data[i, 0])
        dist = laser_data[i, 1]
        curr_x, curr_y = laser_points_current[i]
        corr_x, corr_y = laser_points_corrected[i]
        delta = np.sqrt((curr_x - corr_x)**2 + (curr_y - corr_y)**2)
        
        print(f"{i:<6} {angle_deg:7.1f}° {dist:6.3f}m {curr_x:9.3f}m {curr_y:9.3f}m "
              f"{corr_x:11.3f}m {corr_y:11.3f}m {delta:9.4f}m")
    
    # Step 4: Physical sanity check
    print("\n4. PHYSICAL SANITY CHECK")
    print("-" * 70)
    
    # Calculate actual spread of laser points
    all_laser_points = transform_laser_to_global(robot_pose, laser_data)
    
    x_min, x_max = all_laser_points[:, 0].min(), all_laser_points[:, 0].max()
    y_min, y_max = all_laser_points[:, 1].min(), all_laser_points[:, 1].max()
    x_span = x_max - x_min
    y_span = y_max - y_min
    
    max_sensor_range = 5.0  # Hokuyo max range
    expected_max_span = 2 * max_sensor_range  # Diameter of sensor circle
    
    print(f"Laser points X span: {x_span:.3f}m (from {x_min:.3f} to {x_max:.3f})")
    print(f"Laser points Y span: {y_span:.3f}m (from {y_min:.3f} to {y_max:.3f})")
    print(f"\n⚠️  EXPECTED maximum span: {expected_max_span:.1f}m (2 × sensor range)")
    
    if x_span > expected_max_span * 1.1 or y_span > expected_max_span * 1.1:
        print(f"\n   ❌ PROBLEM DETECTED: Laser points span TOO WIDE!")
        print(f"      This indicates coordinate transformation error.")
        print(f"      Likely cause: Sensor mounting angle NOT accounted for")
    else:
        print(f"\n   ✅ Laser point spread is physically reasonable")
    
    controller.sim.stopSimulation()
    
    # Summary and recommendations
    print("\n" + "="*70)
    print("DIAGNOSTIC SUMMARY")
    print("="*70)
    
    if sensor_offset is not None and abs(sensor_offset) > 0.01:
        print(f"\n⚠️  ACTION REQUIRED:")
        print(f"\n   The sensor is mounted at {np.rad2deg(sensor_offset):.1f}° relative to robot.")
        print(f"   You MUST add this offset to transform_laser_to_global()!")
        print(f"\n   Modify tp3_utils.py line ~563:")
        print(f"   FROM: world_angle = theta_robot + beam_angle")
        print(f"   TO:   world_angle = theta_robot + beam_angle + {sensor_offset:.6f}")
        print(f"         (or in degrees: + np.deg2rad({np.rad2deg(sensor_offset):.1f}))")
    else:
        print(f"\n   ℹ️  Sensor appears to be aligned with robot.")
        print(f"      If you still see scattered points, check other issues:")
        print(f"      - Verify robot pose is correct")
        print(f"      - Check for sensor data corruption")
        print(f"      - Verify laser_data format is [angle, distance]")
    
    print("\n" + "="*70)
    
    return sensor_offset


# Run the diagnostic
# IMPORTANT: Make sure CoppeliaSim is running with your scene loaded!
print("Starting laser transformation diagnostic...")
print("Make sure CoppeliaSim is running with cena-tp3-estatico.ttt loaded!")
print("\nInitializing controller...")

# This will be run from the notebook, so controller should already exist
# If running standalone, uncomment these lines:
# from utils.kobuki_controller import KobukiController
# controller = KobukiController(robot_name='kobuki')
# controller.connect()
# controller.initialize_scene()

# Run diagnostic
detected_offset = diagnose_laser_transformation(controller, iteration_to_check=10)

if detected_offset is not None:
    print(f"\n✅ Diagnostic complete!")
    print(f"📝 Detected sensor offset: {np.rad2deg(detected_offset):.1f}°")
    print(f"\nSave this value and apply it in transform_laser_to_global()!")
else:
    print(f"\n❌ Could not detect sensor offset automatically.")
    print(f"   Manual inspection of scene required.")
