# TP3 Occupancy Grid Mapping - Development Log

**Student:** Daniel Terra Gomes (Matrícula: 2025702870)  
**Program:** PPGCC - Mestrado  
**Institution:** ICEx-UFMG  
**Date:** November 2025  

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Development Timeline](#development-timeline)
3. [Technical Challenges and Solutions](#technical-challenges-and-solutions)
4. [Implementation Status](#implementation-status)
5. [System Architecture](#system-architecture)
6. [Testing Results](#testing-results)
7. [Known Issues and Limitations](#known-issues-and-limitations)
8. [Next Steps](#next-steps)
9. [References](#references)

---

## Project Overview

### Objective
Implement an **Occupancy Grid Mapping** system for the Kobuki differential-drive robot equipped with a Hokuyo laser sensor in CoppeliaSim 4.10.0.

### Key Requirements (from TP3.md)
- ✅ Implement Occupancy Grid algorithm with log-odds updates
- ✅ Add sensor noise (distance and angle)
- ✅ Implement coordinate transformations (laser → robot → global)
- ⏳ Implement ray tracing for free space marking (Bresenham)
- ✅ Simple reactive navigation strategy
- ⏳ Test 3 cell sizes (0.01, 0.1, 0.5 meters)
- ⏳ Run experiments in static and dynamic scenarios
- ⏳ Generate maps and trajectory plots
- ⏳ Documentation and 8-minute video

### Robot Specifications
- **Robot**: Kobuki (differential-drive, similar to Roomba)
- **Wheelbase (L)**: 0.230 m
- **Wheel radius (r)**: 0.035 m
- **Sensor**: fastHokuyo (dual vision sensor configuration)
- **Laser coverage**: 180° (-90° to +90°)
- **Laser beams**: 684 (342 per camera)
- **Max range**: 5.0 meters

---

## Development Timeline

### Phase 1: Project Setup ✅ (Completed)

**Tasks:**
- Created project structure (`T3/dev/` directory)
- Set up utility modules:
  - `utils/tp3_utils.py` - Reused transformation functions from TP1
  - `utils/kobuki_controller.py` - Robot controller based on TP2 Pioneer
  - `utils/occupancy_grid_mapper.py` - Occupancy grid implementation
  - `utils/exploration_planner.py` - Reactive navigation
- Created main notebook: `TP3_OccupancyGrid.ipynb`

**Code Reuse Strategy:**
- ✅ Reused `CoppeliaSimConnector` from TP1
- ✅ Reused transformation functions from TP1 (homogeneous matrices)
- ✅ Adapted differential-drive controller from TP2
- ✅ Adapted potential fields logic for obstacle avoidance

---

### Phase 2: Connection and Robot Control ✅ (Completed)

**Initial Challenges:**
1. **Robot naming convention**: Scene used lowercase `/kobuki` not `/Kobuki`
2. **Motor discovery**: Motors were nested 3 levels deep in hierarchy
3. **Module caching**: Python imports weren't reflecting file changes

**Solutions:**
- Updated robot name to lowercase in `KobukiController`
- Implemented recursive motor discovery in scene hierarchy
- Added `importlib.reload()` mechanism in notebook (Cell 4)

**Result:**
- ✅ Successfully connected to CoppeliaSim
- ✅ Robot handle: 116
- ✅ Motors: `kobuki_leftMotor`, `kobuki_rightMotor` initialized
- ✅ Pose retrieval working: `get_pose()` and `get_pose_2d()`
- ✅ Velocity control: `set_velocity(v, w)` functional

---

### Phase 3: Laser Sensor Integration ⚠️ (Major Challenge)

This phase encountered the **most significant technical challenge** of the project.

#### Challenge 3.1: Understanding fastHokuyo Structure

**Problem:** The fastHokuyo sensor has a complex hierarchical structure.

**Investigation:**
```
/kobuki/fastHokuyo/
  ├── fastHokuyo_body/
  │   ├── fastHokuyo_joint1/
  │   │   └── fastHokuyo_sensor1 (mill object with vision sensor child)
  │   └── fastHokuyo_joint2/
  │       └── fastHokuyo_sensor2 (mill object with vision sensor child)
  └── fastHokuyo_ref (reference dummy)
```

**Key Findings:**
- fastHokuyo uses **dual vision sensors** (342 beams each = 684 total)
- Each camera covers 90° with tangent projection
- Sensors are "mill" objects (type 9) containing vision sensor children
- Lua script handles unprojection and data packaging

---

#### Challenge 3.2: String Signal API Deprecation ❌ (Critical Issue)

**The Core Problem:**

**Initial Symptoms:**
- `get_laser_data()` consistently returned `None`
- No error messages in Python
- Multiple retry attempts failed
- Even after waiting 5+ seconds for Lua script initialization

**Error Evolution:**
1. **First error**: "Resource temporarily unavailable"
2. **Persistent error**: "Operation cannot be accomplished in current state"
3. **Root cause revealed** (from CoppeliaSim terminal):
   ```
   [Connectivity >> ZMQ remote API server@addOnScript:error]
   ...conman\cbor.lua:395: TEXT: not UTF-8 text
   {ret = {{data = [long string (2736 bytes)], }}, }
   ```

**Technical Analysis:**

The Lua script was using **deprecated API**:
```lua
-- DEPRECATED (causing UTF-8 encoding error)
sim.setStringSignal('hokuyo_range_data', sim.packFloatTable(distData))
sim.setStringSignal('hokuyo_angle_data', sim.packFloatTable(angleData))
```

**Why It Failed:**
1. `sim.packFloatTable()` returns **binary data** (raw float bytes)
2. `sim.setStringSignal()` expects **UTF-8 text strings**
3. ZMQ Remote API uses **CBOR encoding** which validates UTF-8
4. Binary float data ≠ valid UTF-8 → **CBOR encoder throws error**
5. Signal transmission fails → Python receives error

**Official Documentation Confirmation:**
- Fetched from: `https://manual.coppeliarobotics.com/en/regularApi/simGetStringSignal.htm`
- **Status**: "DEPRECATED - See properties instead"
- Recommended: Use `sim.getBufferProperty()` / `sim.setBufferProperty()`

---

#### Solution 3.2: Modern Buffer Property API ✅

**Modern API Implementation:**

**Python Side** (`kobuki_controller.py`):
```python
def get_laser_data(self) -> Optional[np.ndarray]:
    """Get laser data using modern buffer property API (CoppeliaSim 4.10+)"""
    try:
        # Modern API - handles binary data correctly
        ranges_packed = self.sim.getBufferProperty(
            self.sim.handle_scene,           # Target: scene-level
            'signal.hokuyo_range_data',      # Property with signal prefix
            {'noError': True}                # Suppress errors if missing
        )
        angles_packed = self.sim.getBufferProperty(
            self.sim.handle_scene,
            'signal.hokuyo_angle_data',
            {'noError': True}
        )
        
        if ranges_packed is None or angles_packed is None:
            return None
        
        # Unpack binary data
        ranges = self.sim.unpackFloatTable(ranges_packed)
        angles = self.sim.unpackFloatTable(angles_packed)
        
        # Combine into Nx2 array [angle, distance]
        laser_data = np.column_stack((angles, ranges))
        return laser_data
        
    except Exception as e:
        print(f"⚠ Error: {e}")
        return None
```

**Lua Side** (fastHokuyo.lua - lines 235-236):
```lua
-- MODERN API (correct implementation)
if #dists>0 then
    sim.setBufferProperty(sim.handle_scene, 'signal.hokuyo_range_data', sim.packFloatTable(dists))
    sim.setBufferProperty(sim.handle_scene, 'signal.hokuyo_angle_data', sim.packFloatTable(angles))
end
```

**Additional Bug Fix:**
The initial Lua update used wrong variable names:
```lua
-- WRONG (variables don't exist)
sim.packFloatTable(distData)  -- should be 'dists'
sim.packFloatTable(angleData) -- should be 'angles'
```

**Final Working Version:**
```lua
-- CORRECT (using actual variable names)
sim.packFloatTable(dists)     -- ✓ local variable in script
sim.packFloatTable(angles)    -- ✓ global variable from init
```

**Documentation Created:**
- Created `T3/LASER_SIGNAL_FIX.md` with complete step-by-step instructions
- Added explanation cell in notebook
- Updated all references in code

**Result:**
- ✅ Successfully receiving **684 laser points**
- ✅ Angle range: **-90° to +90°** (perfect 180° coverage)
- ✅ Distance range: **0.927m to 4.999m** (realistic within 5m max)
- ✅ Buffer properties working flawlessly with ZMQ Remote API

**References:**
- https://manual.coppeliarobotics.com/en/properties.htm
- https://manual.coppeliarobotics.com/en/regularApi/simSetBufferProperty.htm
- https://manual.coppeliarobotics.com/en/regularApi/simGetBufferProperty.htm

---

### Phase 4: Sensor Noise Implementation ✅ (Completed)

**Requirement:** Add Gaussian noise to laser readings (TP3.md specification)

**Implementation:**
```python
distance_noise_std = 0.02   # 2 cm standard deviation
angle_noise_std = 0.005     # ~0.3 degrees standard deviation

noisy_data = clean_data.copy()
noisy_data[:, 0] += np.random.normal(0, angle_noise_std, len(noisy_data))
noisy_data[:, 1] += np.random.normal(0, distance_noise_std, len(noisy_data))
noisy_data[:, 1] = np.clip(noisy_data[:, 1], 0, 5.0)  # Clamp to sensor range
```

**Testing Results:**
```
Sample noise values:
  Point 0: Δθ= 0.0011 rad, Δd=-0.011 m
  Point 1: Δθ= 0.0052 rad, Δd= 0.002 m
  Point 2: Δθ=-0.0011 rad, Δd= 0.007 m
  Point 3: Δθ= 0.0029 rad, Δd=-0.036 m
  Point 4: Δθ=-0.0000 rad, Δd= 0.018 m
```

**Status:** ✅ Working correctly - realistic sensor noise applied

---

### Phase 5: Coordinate Transformations ✅ (Completed)

**Requirement:** Transform laser points from laser frame → robot frame → global frame

**Reused from TP1:**
- `create_homogeneous_matrix(position, orientation)`
- `invert_homogeneous_matrix(matrix)`
- Homogeneous transformation logic

**Implementation:**
```python
def transform_laser_to_global(robot_pose, laser_data):
    """
    Transform laser points from laser frame to global frame.
    
    Args:
        robot_pose: ((x, y, z), (euler_x, euler_y, euler_z))
        laser_data: Nx2 array [angle, distance]
    
    Returns:
        Nx3 array [x, y, z] in global frame
    """
    # 1. Robot to world transform
    W_T_R = create_homogeneous_matrix(robot_pose[0], robot_pose[1])
    
    # 2. Laser to robot transform (static, at robot's height)
    R_T_L = create_homogeneous_matrix([0, 0, 0.061], [0, 0, 0])
    
    # 3. Combined transform
    W_T_L = W_T_R @ R_T_L
    
    # 4. Convert laser data to Cartesian in laser frame
    L_points = laser_polar_to_cartesian(laser_data)
    
    # 5. Transform all points to global frame
    W_points = transform_points(L_points, W_T_L)
    
    return W_points
```

**Testing Results:**
- Robot pose: `x=-3.970m, y=-4.060m, θ=-0.206rad`
- Sample transformed points:
  - Point 0: `x=-4.157, y=-4.961, z=0.161`
  - Point 1: `x=-4.164, y=-4.951, z=0.161`
  - Point 2: `x=-4.149, y=-4.957, z=0.161`

**Visualization:** ✅ Wall structures clearly visible in global frame plot

**Status:** ✅ Transformations working correctly

---

### Phase 6: Occupancy Grid Mapper ⚠️ (Partially Complete)

**Implementation Status:**

**Completed ✅:**
- Grid initialization (log-odds representation)
- World ↔ Grid coordinate conversions
- Occupied cell marking from laser endpoints
- Visualization with probability coloring
- Statistics tracking

**Current Results:**
```
Map Configuration:
  Size: 10m x 10m
  Cell size: 0.1m
  Grid: 100 x 100 cells (10,000 total)
  Log-odds: l_occ=0.9, l_free=-0.7

After 1 scan:
  Occupied: 137 cells (1.37%)
  Free: 0 cells (0.00%)      ← ISSUE: Should mark free space
  Unknown: 9863 cells (98.63%)
```

**Missing Feature ⚠️:**
- **Bresenham ray tracing** not yet implemented
- Currently only marks endpoints (occupied cells)
- Needs to mark all cells along laser beam as "free"
- Required by TP3.md: "discretização da leitura no mundo contínuo"

**Theoretical Basis:**
From `aula18-mapeamento-occupancy-grid.pdf`:
```
For each laser beam:
  1. Start at robot position
  2. Trace line to endpoint using Bresenham
  3. Mark all intermediate cells as FREE (l_free = -0.7)
  4. Mark endpoint cell as OCCUPIED (l_occ = 0.9)
  5. Update using log-odds: l(c) = l(c) + l_update
```

**Status:** ⚠️ Functional but incomplete - needs ray tracing

---

### Phase 7: Exploration Planner ✅ (Completed)

**Strategy:** Simple reactive navigation with potential fields (adapted from TP2)

**Implementation:**
```python
class ExplorationPlanner:
    def __init__(self, v_nominal=0.15, d_safe=0.8):
        self.v_nominal = v_nominal      # Nominal forward velocity
        self.w_max = 0.8                # Max angular velocity
        self.d_safe = d_safe            # Safe distance threshold
        self.k_repulsive = 0.5          # Repulsive gain
    
    def plan_step(self, laser_data):
        """
        Compute velocities based on laser scan.
        
        Returns:
            (v, w): Linear and angular velocities
        """
        # Analyze obstacles in front
        front_readings = laser_data[abs(laser_data[:, 0]) < np.pi/4]
        
        if len(front_readings) > 0:
            min_dist = front_readings[:, 1].min()
            
            if min_dist < self.d_safe:
                # Obstacle detected - apply repulsive force
                # ... (potential field calculations)
                pass
        
        return v, w
```

**Testing Results:**
```
Current scan analysis:
  Linear velocity (v):  0.150 m/s  (forward motion)
  Angular velocity (w): 0.000 rad/s (no obstacles ahead)
```

**Status:** ✅ Working - generates reasonable velocities

---

## Technical Challenges and Solutions

### Challenge 1: Deprecated String Signal API ⚠️⚠️⚠️ (CRITICAL)

**Severity:** Critical - Blocked all progress for significant time  
**Time to Resolve:** ~6 hours of investigation  
**Root Cause:** CoppeliaSim 4.10.0 deprecated `sim.setStringSignal()` for binary data  

**Detailed Timeline:**
1. **Hour 1-2:** Initial connection tests showed `None` laser data
2. **Hour 3:** Multiple debugging attempts, simulation restarts
3. **Hour 4:** Discovered "Resource temporarily unavailable" errors
4. **Hour 5:** User provided CoppeliaSim terminal showing UTF-8 error
5. **Hour 6:** Fetched documentation, identified modern API solution

**Solution Summary:**
- Migrated from `sim.getStringSignal()` → `sim.getBufferProperty()`
- Updated both Python and Lua code
- Fixed variable naming bug in Lua script
- Validated against CoppeliaSim 4.10.0 documentation

**Key Learning:** Always check CoppeliaSim terminal for underlying errors!

---

### Challenge 2: Scene Object Hierarchy

**Problem:** Robot components not found at expected paths  
**Cause:** Scene uses different naming conventions than expected

**Investigation Process:**
1. Listed all scene objects (found 50+ objects)
2. Filtered for "kobuki", "hokuyo", "motor" keywords
3. Discovered nested structure (motors 3 levels deep)
4. Found mill objects containing vision sensors

**Solution:**
- Implemented recursive object discovery
- Updated paths in controller
- Created diagnostic cells for future debugging

---

### Challenge 3: Module Reloading in Notebook

**Problem:** File changes not reflected in notebook execution  
**Cause:** Python imports cached in kernel memory

**Solution:**
```python
import importlib
import sys

if 'utils.kobuki_controller' in sys.modules:
    importlib.reload(sys.modules['utils.kobuki_controller'])
```

**Result:** Can now modify .py files and reload without kernel restart

---

### Challenge 4: HokuyoSensorSim Compatibility

**Problem:** Original `HokuyoSensorSim` class incompatible with fastHokuyo structure  
**Cause:** Class designed for single vision sensor, not dual-camera setup

**Solution:** 
- Use `KobukiController.get_laser_data()` directly
- Bypassed `HokuyoSensorSim` wrapper class
- Simplified code by working directly with buffer properties

---

## Implementation Status

### ✅ Completed Components

1. **Project Structure**
   - All utility modules created
   - Main notebook organized and documented
   - Code properly separated into reusable modules

2. **Robot Connection & Control**
   - CoppeliaSim connection stable
   - Kobuki controller fully functional
   - Differential drive kinematics working
   - Pose retrieval accurate

3. **Laser Sensor**
   - fastHokuyo integration complete
   - 684-point scans working reliably
   - Modern buffer property API implemented
   - Data validated (angles, distances correct)

4. **Sensor Noise**
   - Gaussian noise implementation
   - Distance noise: 2 cm std dev
   - Angle noise: 0.3° std dev
   - Proper clamping to sensor range

5. **Coordinate Transformations**
   - Homogeneous matrix operations
   - Laser → Robot → Global transforms
   - Visualizations confirm accuracy
   - Reused robust code from TP1

6. **Exploration Planner**
   - Reactive navigation strategy
   - Obstacle avoidance logic
   - Velocity generation working

7. **Occupancy Grid (Partial)**
   - Grid initialization
   - Log-odds representation
   - Occupied cell marking
   - Visualization functional
   - Statistics calculation

---

### ⏳ Pending Work

1. **Occupancy Grid Enhancement**
   - ⚠️ Implement Bresenham ray tracing for free space
   - This is **required by TP3.md** specification
   - Critical for proper map quality

2. **Main Simulation Loop**
   - Integrate all components
   - Implement exploration loop:
     - Get pose
     - Get laser data (with noise)
     - Transform to global
     - Update occupancy grid
     - Plan next move
     - Execute motion
     - Repeat until map complete

3. **Experiments (TP3.md Requirements)**
   - **Test 1:** 3 cell sizes (0.01, 0.1, 0.5m) in static scene
   - **Test 2:** 2+ experiments in static scene (different positions)
   - **Test 3:** 2+ experiments in dynamic scene (moving obstacles)
   - Generate all required maps and plots

4. **Trajectory Plotting**
   - Incremental plot showing:
     - Robot path (line)
     - All laser points (scatter)
     - Similar to TP1 Exercise 6

5. **Documentation**
   - PDF documentation with:
     - Introduction
     - Execution instructions
     - Navigation strategy explanation
     - Implementation details
     - Algorithm descriptions
     - Test results and analysis
     - Conclusions
     - Bibliography
   - 8-minute presentation video

---

## System Architecture

### File Structure
```
T3/
├── dev/                                    # Development workspace
│   ├── TP3_OccupancyGrid.ipynb            # Main notebook (35 cells)
│   └── utils/
│       ├── tp3_utils.py                   # Transformations, noise, plotting
│       ├── kobuki_controller.py           # Robot controller
│       ├── occupancy_grid_mapper.py       # Occupancy grid algorithm
│       └── exploration_planner.py         # Navigation strategy
├── cena-tp3-estatico.ttt                  # Static scene
├── cena-tp3-dinamico.ttt                  # Dynamic scene
├── TP3.md                                 # Assignment specification
├── LASER_SIGNAL_FIX.md                    # API fix documentation
├── DEVELOPMENT_LOG.md                     # This file
└── aula18-mapeamento-occupancy-grid.md    # Lecture notes
```

### Data Flow
```
CoppeliaSim Scene
    ↓
fastHokuyo Lua Script (dual vision sensors)
    ↓ (sim.setBufferProperty)
ZMQ Remote API (buffer properties)
    ↓ (sim.getBufferProperty)
KobukiController.get_laser_data()
    ↓
Sensor Noise Addition
    ↓
Coordinate Transformation (laser → global)
    ↓
OccupancyGridMapper.update_map()
    ↓
ExplorationPlanner.plan_step()
    ↓
KobukiController.set_velocity(v, w)
    ↓
Robot Motion in Simulation
```

### Key Classes

**KobukiController**
- `connect()` - Establish CoppeliaSim connection
- `initialize_scene()` - Find robot, motors, sensors
- `get_pose()` - Get 3D pose (position, orientation)
- `get_pose_2d()` - Get 2D pose (x, y, theta)
- `get_laser_data()` - Get 684 laser points (modern API)
- `set_velocity(v, w)` - Control robot motion
- `stop()` - Stop robot

**OccupancyGridMapper**
- `__init__(map_size, cell_size)` - Initialize grid
- `update_map(robot_pose, laser_points)` - Update with scan
- `world_to_grid(x, y)` - Convert coordinates
- `grid_to_world(i, j)` - Convert coordinates
- `get_statistics()` - Get map statistics
- `visualize_map()` - Display probability map
- `save_map_image(filename)` - Export map

**ExplorationPlanner**
- `plan_step(laser_data)` - Compute (v, w) velocities

---

## Testing Results

### Component Tests (All Passing ✅)

**Test 1: Connection** ✅
```
Robot: kobuki (handle: 116)
Motors: kobuki_leftMotor, kobuki_rightMotor
Pose: x=-3.970m, y=-4.060m, theta=-0.206rad
```

**Test 2: Laser Sensor** ✅
```
Points: 684
Angle range: -90.0° to +90.0°
Distance range: 0.927m to 4.999m
Sample: angle=0.0°, dist=4.999m (max range)
```

**Test 3: Sensor Noise** ✅
```
Noise applied successfully:
  Angle noise std: 0.005 rad (0.3°)
  Distance noise std: 0.02 m (2 cm)
All points within valid ranges
```

**Test 4: Transformations** ✅
```
684 points transformed to global frame
Visualizations show correct wall positions
Reference frames align properly
```

**Test 5: Occupancy Grid** ⚠️ (Partial)
```
Grid: 100x100 cells (10m x 10m, 0.1m resolution)
Occupied: 137 cells (1.37%) ✅
Free: 0 cells (0.00%) ⚠️ (ray tracing needed)
Unknown: 9863 cells (98.63%)
```

**Test 6: Exploration Planner** ✅
```
Velocities generated:
  v = 0.150 m/s (forward)
  w = 0.000 rad/s (straight)
Reasonable for current scan
```

---

## Known Issues and Limitations

### Issue 1: Missing Ray Tracing ⚠️ (HIGH PRIORITY)

**Problem:** Occupancy grid only marks occupied cells, not free space  
**Impact:** Map quality significantly reduced  
**Required By:** TP3.md specification  
**Solution:** Implement Bresenham line algorithm  
**Priority:** HIGH - Must complete before experiments

**Implementation Plan:**
```python
def bresenham_line(x0, y0, x1, y1):
    """
    Generate all grid cells along line from (x0,y0) to (x1,y1)
    Returns list of (i, j) cell coordinates
    """
    # Bresenham's line algorithm
    pass

def update_map_with_ray_tracing(robot_pose, laser_points):
    """
    For each laser point:
      1. Get robot cell
      2. Get endpoint cell
      3. Trace line between them
      4. Mark all intermediate cells as FREE
      5. Mark endpoint as OCCUPIED
    """
    pass
```

---

### Issue 2: Exploration Strategy Simplicity

**Current State:** Basic reactive navigation  
**Limitation:** May not achieve optimal exploration  
**Impact:** Longer exploration times, incomplete coverage  
**Acceptance:** Simple strategy acceptable per TP3.md  

**Possible Improvements (Future Work):**
- Frontier-based exploration
- Voronoi diagram paths
- A* path planning with unknown space
- Hybrid approaches

---

### Issue 3: No Test Coverage

**Problem:** No unit tests or integration tests  
**Risk:** Regressions not detected  
**Mitigation:** Manual testing in notebook  
**Future:** Add pytest suite

---

### Issue 4: Hardcoded Parameters

**Examples:**
- Map size: 10m x 10m
- Cell size: 0.1m (for testing)
- Laser max range: 5.0m
- Noise std devs: 0.02m, 0.005rad

**Should Be:** Configurable via parameters or config file

---

## Next Steps

### Immediate (Before Experiments)

1. **Implement Bresenham Ray Tracing** ⚠️ HIGH PRIORITY
   - Add `bresenham_line()` function
   - Update `update_map()` to trace rays
   - Test with single scan
   - Verify free space is marked

2. **Implement Main Simulation Loop**
   - Create new notebook cell
   - Integrate all components
   - Add progress indicators
   - Handle edge cases (collisions, stuck states)

3. **Add Trajectory Recording**
   - Store robot poses over time
   - Store all laser points
   - Prepare for plotting

### Experiments (TP3.md Requirements)

**Experiment 1: Cell Size Comparison** (Static Scene)
- Run with cell_size = 0.01m → save map
- Run with cell_size = 0.1m → save map
- Run with cell_size = 0.5m → save map
- Analyze quality vs memory/computation tradeoff
- Select best cell size for remaining tests

**Experiment 2: Static Scene** (Best Cell Size)
- Position 1: Run full exploration, save results
- Position 2: Run full exploration, save results
- Generate trajectory + laser points plot
- Generate occupancy grid maps

**Experiment 3: Dynamic Scene** (Best Cell Size)
- Load `cena-tp3-dinamico.ttt`
- Position 1: Run with moving obstacles
- Position 2: Run with moving obstacles
- Compare results with static scene
- Analyze impact of dynamic objects

### Documentation

1. **PDF Report** (per TP3.md structure)
   - Introduction: Problem, robot specs, scenes
   - Execution: Dependencies, how to run
   - Navigation: Strategy explanation
   - Implementation: Algorithms, data structures
   - Tests: Results, analysis, plots
   - Conclusion: Summary, difficulties
   - Bibliography: Including documentation URLs

2. **Presentation Video** (≤8 minutes)
   - System demonstration
   - Map generation in real-time
   - Results overview
   - Key implementation points

---

## References

### Official Documentation

**CoppeliaSim 4.10.0:**
- Main Manual: https://manual.coppeliarobotics.com/
- Remote API Overview: https://manual.coppeliarobotics.com/en/remoteApiOverview.htm
- ZMQ Remote API: https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm
- Properties System: https://manual.coppeliarobotics.com/en/properties.htm
- Buffer Properties: https://manual.coppeliarobotics.com/en/regularApi/simSetBufferProperty.htm
- String Signals (deprecated): https://manual.coppeliarobotics.com/en/regularApi/simGetStringSignal.htm

**Kobuki Robot:**
- Parameters: https://yujinrobot.github.io/kobuki/enAppendixKobukiParameters.html
- Official Documentation: https://yujinrobot.github.io/kobuki/

### Academic References

**Occupancy Grid Mapping:**
- Moravec, H., & Elfes, A. (1985). "High Resolution Maps from Wide Angle Sonar"
- Thrun, S., Burgard, W., & Fox, D. (2005). "Probabilistic Robotics"
- Lecture Slides: `aula18-mapeamento-occupancy-grid.pdf`

### Course Materials

- Assignment Specification: `T3/TP3.md`
- Previous Work: TP1 (transformations), TP2 (control, potential fields)
- Scene Files: `cena-tp3-estatico.ttt`, `cena-tp3-dinamico.ttt`

---

## Appendix: Code Snippets

### Modern Buffer Property API (Complete Solution)

**Python (`kobuki_controller.py`):**
```python
def get_laser_data(self) -> Optional[np.ndarray]:
    """
    Get laser data using modern buffer property API (CoppeliaSim 4.10+).
    
    Returns:
        Nx2 numpy array [angle, distance] or None if no data
    """
    try:
        # Read binary data from buffer properties
        ranges_packed = self.sim.getBufferProperty(
            self.sim.handle_scene,          # Scene-level target
            'signal.hokuyo_range_data',     # Property name
            {'noError': True}               # Silent if missing
        )
        angles_packed = self.sim.getBufferProperty(
            self.sim.handle_scene,
            'signal.hokuyo_angle_data',
            {'noError': True}
        )
        
        # Check if data available
        if ranges_packed is None or angles_packed is None:
            return None
        
        # Unpack binary float data
        ranges = self.sim.unpackFloatTable(ranges_packed)
        angles = self.sim.unpackFloatTable(angles_packed)
        
        # Validate data
        if not ranges or not angles or len(ranges) != len(angles):
            return None
        
        # Combine into structured array
        laser_data = np.column_stack((angles, ranges))
        return laser_data
        
    except Exception as e:
        print(f"⚠ Error reading laser data: {e}")
        return None
```

**Lua (`fastHokuyo.lua` - lines 235-236):**
```lua
-- Send laser data using modern buffer property API
if #dists>0 then
    -- Pack float arrays to binary and send via buffer properties
    sim.setBufferProperty(
        sim.handle_scene,                           -- Scene-level target
        'signal.hokuyo_range_data',                 -- Property name
        sim.packFloatTable(dists)                   -- Binary data
    )
    sim.setBufferProperty(
        sim.handle_scene,
        'signal.hokuyo_angle_data',
        sim.packFloatTable(angles)
    )
end
```

---

## Document Changelog

| Date | Version | Changes |
|------|---------|---------|
| 2025-11-06 | 1.0 | Initial comprehensive documentation created |

---

**End of Development Log**

*This document will be updated as development progresses.*
