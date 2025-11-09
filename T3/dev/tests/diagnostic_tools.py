# Occupancy Grid Debugging Script
# Run this after your simulation to diagnose mapping issues

import numpy as np
import matplotlib.pyplot as plt

def analyze_mapping_alignment(robot_trajectory, mapper, laser_points_global):
    """
    Comprehensive analysis of grid-scene alignment.
    
    This function checks if your grid origin and size are correctly configured
    for the actual robot movement and laser data collected.
    """
    print("\n" + "="*70)
    print("OCCUPANCY GRID ALIGNMENT ANALYSIS")
    print("="*70)
    
    # Convert trajectory to array
    traj = np.array(robot_trajectory)
    laser_pts = np.array(laser_points_global)
    
    # 1. ROBOT TRAJECTORY BOUNDS
    print("\n1. ROBOT TRAJECTORY ANALYSIS")
    print("-" * 70)
    x_min_traj, x_max_traj = traj[:, 0].min(), traj[:, 0].max()
    y_min_traj, y_max_traj = traj[:, 1].min(), traj[:, 1].max()
    print(f"   X range: [{x_min_traj:7.3f}, {x_max_traj:7.3f}] m  (span: {x_max_traj-x_min_traj:.3f}m)")
    print(f"   Y range: [{y_min_traj:7.3f}, {y_max_traj:7.3f}] m  (span: {y_max_traj-y_min_traj:.3f}m)")
    
    # 2. LASER POINTS BOUNDS
    print("\n2. LASER POINTS ANALYSIS")
    print("-" * 70)
    x_min_laser, x_max_laser = laser_pts[:, 0].min(), laser_pts[:, 0].max()
    y_min_laser, y_max_laser = laser_pts[:, 1].min(), laser_pts[:, 1].max()
    print(f"   X range: [{x_min_laser:7.3f}, {x_max_laser:7.3f}] m  (span: {x_max_laser-x_min_laser:.3f}m)")
    print(f"   Y range: [{y_min_laser:7.3f}, {y_max_laser:7.3f}] m  (span: {y_max_laser-y_min_laser:.3f}m)")
    
    # 3. GRID CONFIGURATION
    print("\n3. GRID CONFIGURATION")
    print("-" * 70)
    grid_x_min = mapper.origin[0]
    grid_x_max = mapper.origin[0] + mapper.map_size[0]
    grid_y_min = mapper.origin[1]
    grid_y_max = mapper.origin[1] + mapper.map_size[1]
    print(f"   Origin:     ({mapper.origin[0]:7.3f}, {mapper.origin[1]:7.3f}) m")
    print(f"   Size:       ({mapper.map_size[0]:7.3f}, {mapper.map_size[1]:7.3f}) m")
    print(f"   Cell size:   {mapper.cell_size:.4f} m")
    print(f"   Grid shape:  {mapper.grid_height} × {mapper.grid_width} cells")
    print(f"   X coverage: [{grid_x_min:7.3f}, {grid_x_max:7.3f}] m")
    print(f"   Y coverage: [{grid_y_min:7.3f}, {grid_y_max:7.3f}] m")
    
    # 4. ALIGNMENT CHECK
    print("\n4. ALIGNMENT VERIFICATION")
    print("-" * 70)
    
    # Check trajectory coverage
    traj_x_in_grid = (traj[:, 0] >= grid_x_min) & (traj[:, 0] <= grid_x_max)
    traj_y_in_grid = (traj[:, 1] >= grid_y_min) & (traj[:, 1] <= grid_y_max)
    traj_in_grid = traj_x_in_grid & traj_y_in_grid
    traj_coverage = 100 * traj_in_grid.sum() / len(traj)
    
    print(f"   Trajectory coverage:  {traj_coverage:.1f}% within grid bounds")
    if traj_coverage < 80:
        print("   ⚠️  WARNING: Less than 80% of robot path is inside grid!")
        print("   → Consider adjusting MAP_ORIGIN or MAP_SIZE_M")
    else:
        print("   ✅ Good coverage!")
    
    # Check laser points coverage
    laser_x_in_grid = (laser_pts[:, 0] >= grid_x_min) & (laser_pts[:, 0] <= grid_x_max)
    laser_y_in_grid = (laser_pts[:, 1] >= grid_y_min) & (laser_pts[:, 1] <= grid_y_max)
    laser_in_grid = laser_x_in_grid & laser_y_in_grid
    laser_coverage = 100 * laser_in_grid.sum() / len(laser_pts)
    
    print(f"   Laser points coverage: {laser_coverage:.1f}% within grid bounds")
    if laser_coverage < 70:
        print("   ⚠️  WARNING: Less than 70% of laser data is inside grid!")
        print("   → Grid might be too small or misaligned")
    else:
        print("   ✅ Good coverage!")
    
    # 5. RECOMMENDED ADJUSTMENTS
    print("\n5. RECOMMENDED GRID CONFIGURATION")
    print("-" * 70)
    
    # Calculate optimal grid to cover all data
    data_x_min = min(x_min_traj, x_min_laser) - 1.0  # Add 1m margin
    data_x_max = max(x_max_traj, x_max_laser) + 1.0
    data_y_min = min(y_min_traj, y_min_laser) - 1.0
    data_y_max = max(y_max_traj, y_max_laser) + 1.0
    
    # Round to nice values
    suggested_origin_x = np.floor(data_x_min)
    suggested_origin_y = np.floor(data_y_min)
    suggested_size_x = np.ceil(data_x_max - suggested_origin_x)
    suggested_size_y = np.ceil(data_y_max - suggested_origin_y)
    
    print(f"   Based on actual data range:")
    print(f"   MAP_ORIGIN = ({suggested_origin_x:.1f}, {suggested_origin_y:.1f})")
    print(f"   MAP_SIZE_M = ({suggested_size_x:.1f}, {suggested_size_y:.1f})")
    print(f"   (Covers: X=[{suggested_origin_x:.1f}, {suggested_origin_x+suggested_size_x:.1f}], "
          f"Y=[{suggested_origin_y:.1f}, {suggested_origin_y+suggested_size_y:.1f}])")
    
    # Check if current grid is adequate
    if traj_coverage >= 80 and laser_coverage >= 70:
        print("\n   ✅ CURRENT GRID CONFIGURATION IS ADEQUATE")
        print("   If you're seeing distortions, the problem is NOT grid alignment.")
        print("   Check: sensor calibration, pose accuracy, or other issues.")
    else:
        print("\n   ⚠️  GRID CONFIGURATION NEEDS ADJUSTMENT")
        print("   Update your notebook with the recommended values above.")
    
    # 6. VISUAL DIAGNOSTIC
    print("\n6. GENERATING DIAGNOSTIC PLOT...")
    print("-" * 70)
    
    fig, ax = plt.subplots(figsize=(14, 14))
    
    # Plot grid bounds (as rectangle)
    from matplotlib.patches import Rectangle
    grid_rect = Rectangle((grid_x_min, grid_y_min), 
                           mapper.map_size[0], mapper.map_size[1],
                           fill=False, edgecolor='red', linewidth=3, 
                           linestyle='--', label='Grid Bounds')
    ax.add_patch(grid_rect)
    
    # Plot laser points
    ax.scatter(laser_pts[:, 0], laser_pts[:, 1], 
               c='blue', s=1, alpha=0.3, label='Laser Points')
    
    # Plot robot trajectory
    ax.plot(traj[:, 0], traj[:, 1], 
            'g-', linewidth=2, alpha=0.8, label='Robot Path')
    ax.scatter(traj[0, 0], traj[0, 1], 
               c='green', s=200, marker='o', edgecolor='black', 
               linewidth=2, zorder=5, label='Start')
    ax.scatter(traj[-1, 0], traj[-1, 1], 
               c='red', s=200, marker='X', edgecolor='black', 
               linewidth=2, zorder=5, label='End')
    
    # Add grid origin marker
    ax.scatter(mapper.origin[0], mapper.origin[1], 
               c='red', s=300, marker='+', linewidth=3, 
               zorder=5, label='Grid Origin')
    
    # Configure plot
    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    ax.set_title('Grid Alignment Diagnostic\n'
                 f'Grid: {mapper.map_size[0]}×{mapper.map_size[1]}m @ origin ({mapper.origin[0]}, {mapper.origin[1]})',
                 fontsize=16)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=12)
    ax.set_aspect('equal')
    
    # Set limits to show everything
    all_x = np.concatenate([traj[:, 0], laser_pts[:, 0], [grid_x_min, grid_x_max]])
    all_y = np.concatenate([traj[:, 1], laser_pts[:, 1], [grid_y_min, grid_y_max]])
    margin = 2.0
    ax.set_xlim([all_x.min() - margin, all_x.max() + margin])
    ax.set_ylim([all_y.min() - margin, all_y.max() + margin])
    
    plt.tight_layout()
    plt.savefig('grid_alignment_diagnostic.png', dpi=150, bbox_inches='tight')
    print("   ✅ Diagnostic plot saved: grid_alignment_diagnostic.png")
    plt.show()
    
    print("\n" + "="*70)
    print("ANALYSIS COMPLETE")
    print("="*70 + "\n")
    
    return {
        'trajectory_coverage': traj_coverage,
        'laser_coverage': laser_coverage,
        'suggested_origin': (suggested_origin_x, suggested_origin_y),
        'suggested_size': (suggested_size_x, suggested_size_y)
    }


# HOW TO USE THIS SCRIPT:
# -----------------------
# After running your simulation in the notebook, add this cell:
#
# result = analyze_mapping_alignment(robot_trajectory, mapper, all_laser_points)
#
# This will generate a comprehensive report and diagnostic plot showing:
# 1. Whether your grid covers the actual robot movement
# 2. Whether laser points are within grid bounds
# 3. Recommended grid configuration if needed
# 4. Visual diagnostic showing grid vs. actual data
