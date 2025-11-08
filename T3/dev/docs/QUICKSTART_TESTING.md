# QUICKSTART: Testing Enhanced Stuck Detection

## Pre-Flight Checklist

✅ **CoppeliaSim running** with `cena-tp3-estatico.ttt` loaded  
✅ **Notebook kernel restarted** (to load updated `exploration_planner.py`)  
✅ **All cells cleared** (clear outputs for fresh run)

---

## Step-by-Step Execution

### Step 1: Restart Kernel and Import Modules

Run **Cell 1** (Imports):
```python
# Import all modules
# Should print: "All modules imported successfully!"
```

**Expected Output:**
```
============================================================
                 TP3 - Occupancy Grid Mapping                
============================================================

All modules imported successfully!
Debug mode: ENABLED
```

---

### Step 2: Configuration

Run **Cell 17** (Configuration):

**Check Output:**
```
Configuration:
  Scene: static
  Cell size: 0.1 m
  Duration: 60.0 s
  Map size: (10, 10) m
  V_nominal: 0.15 m/s
  D_safe: 0.5 m (ENHANCED: closer navigation)  <-- Should say 0.5, not 0.8
```

---

### Step 3: Initialize Components

Run **Cell 18** (Initialize):

**CRITICAL: Check Planner Configuration:**
```
Planner configuration:
  v_nominal: 0.15 m/s
  d_safe: 0.5 m           <-- MUST be 0.5 (not 0.8)
  cell_size: 0.1 m        <-- MUST match CELL_SIZE from config
  stuck_threshold: 15 iterations    <-- MUST be 15 (not 30)
  oscillation_threshold: 0.2        <-- MUST be 0.2 (not 0.5)
```

**If values are wrong, you need to restart kernel!**

---

### Step 4: Run Main Simulation

Run **Cell 19** (Main Loop):

**What to Watch For:**

#### Every 1 Second (20 position samples):
```
[Debug] Stuck diagnostics:
  Position variance: 0.0183 (threshold: 0.2)
  Variance check: True
  Stuck counter: 10/15
  Recent angular velocities: [0.15, 0.12, -0.05, -0.18, -0.22]
```

#### When Escape Triggers:
```
[Escape] Trap detected! Executing short nudge (20 steps)...
  Trigger: variance=True, oscillation=False
  Stuck counter reached: 15 > 15
```

#### Progress Every 5 Seconds:
```
[5.0s] Mode: ESCAPE | Pose: (-3.95, -4.10, 0.85) | Vel: (0.00, 0.64)
[10.0s] Mode: NORMAL | Pose: (-3.50, -3.85, 1.20) | Vel: (0.15, 0.02)
```

---

## What Success Looks Like

### ✅ GOOD BEHAVIOR (Working Correctly):

1. **Escape triggers within first 5-10 seconds**
   ```
   [Escape] Trap detected! Executing short nudge (20 steps)...
   ```

2. **Mode alternates between NORMAL and ESCAPE**
   ```
   [5.0s] Mode: NORMAL
   [8.5s] Mode: ESCAPE   <-- Escape triggered
   [10.2s] Mode: NORMAL  <-- Returned to normal
   ```

3. **Debug shows stuck counter incrementing**
   ```
   Stuck counter: 0/15  → 5/15 → 10/15 → 15/15 → ESCAPE!
   ```

4. **Position variance stays low when stuck**
   ```
   Position variance: 0.0183 (below 0.2 threshold)
   Variance check: True
   ```

5. **Robot explores more area**
   ```
   Visited cells: 120+ (not stuck at 42 like before)
   ```

---

### ❌ BAD BEHAVIOR (Still Broken):

1. **No escape messages for entire 60 seconds**
   ```
   [0.4s] Mode: NORMAL
   [35.0s] Mode: NORMAL   <-- Should have escaped by now!
   [60.0s] Mode: NORMAL
   ```

2. **Mode always NORMAL, never ESCAPE**

3. **Stuck counter never reaches 15**
   ```
   Stuck counter: 0/15 → 3/15 → 5/15 → 2/15 → 0/15  <-- Resets before threshold
   ```

4. **Position variance always above 0.2**
   ```
   Position variance: 0.85 (threshold: 0.2)
   Variance check: False   <-- Never triggers
   ```

5. **Visited cells very low (~40-50)**

---

## Troubleshooting

### Problem: "d_safe: 0.8" in planner configuration

**Cause:** Old code still loaded in memory  
**Fix:**
1. Kernel → Restart Kernel
2. Re-run Cell 1 (Imports)
3. Re-run Cell 17 (Config)
4. Re-run Cell 18 (Initialize)
5. Check planner.d_safe again

---

### Problem: No debug messages appearing

**Cause:** `position_history` not filling up  
**Check:**
```python
# After a few seconds, run this in a new cell:
print(f"Position history length: {len(planner.position_history)}")
print(f"Should be: 20-50")
```

**If length is 0:** `current_pos` not being passed to `plan_step_with_escape()`

**Fix:** Cell 19 should have:
```python
v, w = planner.plan_step_with_escape(laser_data, current_pos=(x, y))
#                                                 ^^^^^^^^^^^^^^^^^^^^
```

---

### Problem: Escape never triggers, variance always high

**Possible Causes:**
1. Robot is actually moving normally (good!)
2. Thresholds still too conservative
3. Robot in very dynamic area (lots of space)

**Diagnosis:** Check debug output:
```
Position variance: 1.245 (threshold: 0.2)
```

**If variance > 0.2 consistently:** Robot is legitimately exploring, not stuck!

**To test stuck detection:** Place robot in tight corner manually in CoppeliaSim

---

### Problem: Escape triggers too often

**Symptoms:**
```
[2.5s] Mode: ESCAPE
[4.0s] Mode: ESCAPE
[6.2s] Mode: ESCAPE   <-- Too frequent!
```

**Cause:** Thresholds too aggressive  
**Fix:** Increase thresholds in `exploration_planner.py`:
```python
self.stuck_threshold = 20  # Was 15
self.oscillation_threshold = 0.15  # Was 0.2
```

---

## Quick Validation Commands

After simulation completes, run in a new cell:

```python
# Check final statistics
print(f"Total iterations: {iteration}")
print(f"Trajectory length: {len(robot_trajectory)}")
print(f"Visited cells: {len(planner.visited_cells)}")
print(f"Velocity history length: {len(velocity_history)}")

# Check if escape ever triggered
escape_count = sum(1 for v, w in velocity_history if v == 0.0 and abs(w) > 0.5)
print(f"Approximate escape phases: {escape_count // 15}")  # 15 steps per escape

# Check exploration coverage
stats = mapper.get_statistics()
print(f"Map coverage: {stats['occupied_percent'] + stats['free_percent']:.1f}%")
```

**Expected Values:**
- Visited cells: **120+** (was 42 before fix)
- Escape phases: **2-4** times (was 0 before fix)
- Map coverage: **40-60%** (was ~30% before fix)

---

## Success Criteria

✅ **Escape mechanism triggers** (at least once in 60s)  
✅ **Mode alternates** between NORMAL and ESCAPE  
✅ **Visited cells > 100** (shows exploration progress)  
✅ **Debug logs appear** every 1 second  
✅ **Stuck counter reaches 15** before escape  
✅ **Variance detection works** (True when stuck)

---

## Next Steps After Validation

Once escape mechanism works correctly:

1. **Complete TP3 Experiments:**
   - Test 3 cell sizes: 0.01, 0.1, 0.5
   - Test 2 positions per scene (static + dynamic)
   
2. **Tune Parameters (if needed):**
   - Adjust `stuck_threshold` for optimal escape timing
   - Adjust `oscillation_threshold` for sensitivity
   
3. **Document Results:**
   - Save occupancy grids
   - Save incremental plots
   - Analyze efficiency and efficacy

---

**Status:** 🚀 READY TO TEST
