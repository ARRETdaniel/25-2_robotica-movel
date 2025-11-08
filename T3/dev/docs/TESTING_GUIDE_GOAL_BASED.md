# Testing Guide: Goal-Based vs Reactive Exploration

**Quick Start Guide for Testing the New Planner**

---

## Option 1: Test Goal-Based Planner (NEW)

### Step 1: Update Cell 1 (Imports)

Add this import:
```python
from utils.goal_based_exploration_planner import GoalBasedExplorationPlanner
```

### Step 2: Replace Cell 18 (Planner Initialization)

**Replace this:**
```python
planner = ExplorationPlanner(
    v_nominal=V_NOMINAL,
    d_safe=D_SAFE,
    cell_size=CELL_SIZE
)
```

**With this:**
```python
planner = GoalBasedExplorationPlanner(
    v_nominal=V_NOMINAL,
    d_safe=D_SAFE,
    goal_distance=2.5,      # NEW: Virtual goal distance
    goal_threshold=0.5,     # NEW: Goal reached threshold
    map_size=MAP_SIZE,
    cell_size=CELL_SIZE
)
```

### Step 3: Update Cell 19 (Main Loop)

**Find this line (~line 55 in Cell 19):**
```python
v, w = planner.plan_step_with_escape(laser_data, current_pos=(x, y))
```

**Replace with:**
```python
v, w = planner.plan_step(laser_data, current_pos=(x, y), robot_theta=theta)
```

### Step 4: Run!

1. Restart Kernel
2. Run Cell 1 (Imports)
3. Run Cell 2 (Connect)
4. Run Cell 17 (Config)
5. Run Cell 18 (Initialize with NEW planner)
6. Run Cell 19 (Main Loop)

Watch for these messages:
```
[Goal] New goal set: (x, y)
[Goal] Goal #1 reached! New goal: (x, y)
[Stuck] Detected! Triggering RUTF escape...
[RUTF] Generating random force escape...
```

---

## Option 2: Keep Reactive Planner (CURRENT)

No changes needed! Just run as normal:
1. Restart Kernel
2. Run Cells 1, 2, 17, 18, 19

---

## Expected Differences

### Console Output

**Reactive Planner:**
```
[Debug] Stuck diagnostics:
  Collision stall: True/False
[WARNING] Collision stall detected!
[Escape] Trap detected! Executing short nudge...
```

**Goal-Based Planner:**
```
[Goal] New goal set: (-2.35, 3.87)
[Goal] Goal #1 reached! New goal: (4.12, -1.53)
[Stuck] Detected! Triggering RUTF escape...
[RUTF] Escape queued: 20 steps, then new goal
```

### Performance Metrics

| Metric | Reactive | Goal-Based (Expected) |
|--------|----------|----------------------|
| Visited Cells | ~60-80 | ~100-150 |
| Coverage | 15-20% | 25-35% |
| Stuck Events | 2-4 | 1-2 |
| Exploration Pattern | Erratic | Systematic |

---

## Side-by-Side Comparison Test

Want to compare both? Run this procedure:

### Test A: Reactive Planner

1. Use current notebook (no changes)
2. Run Cell 19, wait 60 seconds
3. Note results:
   - Visited cells: _____
   - Final position: (_____, _____)
   - Map coverage: _____%
4. Save map: `occupancy_grid_reactive.png`
5. Save plot: `incremental_plot_reactive.png`

### Test B: Goal-Based Planner

1. Apply changes from Option 1 above
2. Restart kernel, run Cells 1, 2, 17, 18
3. **Reset robot to SAME start position** (if needed)
4. Run Cell 19, wait 60 seconds
5. Note results:
   - Visited cells: _____
   - Goals reached: _____
   - Final position: (_____, _____)
   - Map coverage: _____%
6. Save map: `occupancy_grid_goalbased.png`
7. Save plot: `incremental_plot_goalbased.png`

### Compare

Open both map images side-by-side:
- Which has better coverage?
- Which has more uniform exploration?
- Which avoided stuck situations better?

---

## Troubleshooting

### Problem: "module 'utils' has no attribute 'GoalBasedExplorationPlanner'"

**Solution:** Restart kernel! The new file needs to be loaded.

### Problem: Robot goes to map edge and gets stuck

**Possible cause:** `map_size` parameter incorrect

**Solution:** Check that `MAP_SIZE = (10, 10)` matches your scene bounds

### Problem: Robot oscillates near goal

**Possible cause:** `goal_threshold` too small

**Solution:** Increase `goal_threshold` from 0.5 to 1.0:
```python
planner = GoalBasedExplorationPlanner(
    ...,
    goal_threshold=1.0  # Was 0.5
)
```

### Problem: Robot generates goals outside map

**Check:** Are goal bounds correct?
```python
print(f"Map bounds: {planner.map_min} to {planner.map_max}")
```

**Solution:** Verify `map_size` parameter matches scene

### Problem: Not enough goals reached

**Possible causes:**
1. `goal_distance` too large (goals unreachable)
2. Too many obstacles between robot and goals
3. Robot getting stuck before reaching goals

**Solutions:**
1. Reduce `goal_distance` from 2.5 to 2.0
2. Increase `k_att` from 0.8 to 1.0 (stronger goal attraction)
3. Check stuck detection is working

---

## Quick Validation Checklist

After running goal-based planner, verify:

- [ ] Console shows "Goal-Based Exploration Planner initialized"
- [ ] Console shows "[Goal] New goal set: ..." messages
- [ ] Console shows "[Goal] Goal #X reached!" at least once
- [ ] Robot visits different areas of map (not just circling)
- [ ] Visited cells > 80 (better than reactive ~60)
- [ ] No collisions with walls (emergency stop working)
- [ ] If stuck, RUTF escape triggers within ~10 seconds

If ALL checkboxes ✅, planner is working correctly!

---

## Parameter Tuning Guide

### To Increase Coverage

**Option 1:** Increase goal distance
```python
goal_distance=3.0  # Was 2.5, robot explores farther
```

**Option 2:** Reduce goal threshold
```python
goal_threshold=0.3  # Was 0.5, generates goals more frequently
```

**Option 3:** Increase attractive gain
```python
k_att=1.0  # Was 0.8, robot prioritizes reaching goals
```

### To Improve Safety

**Option 1:** Increase repulsive gain
```python
k_rep=0.7  # Was 0.5, stronger obstacle avoidance
```

**Option 2:** Increase safe distance
```python
d_safe=0.6  # Was 0.5, maintains larger margin
```

### To Reduce Stuck Events

**Option 1:** More sensitive stuck detection
```python
stuck_threshold=10  # Was 15, triggers escape sooner
oscillation_threshold=0.15  # Was 0.2, more sensitive
```

**Option 2:** New goal after every stuck escape
*(Already implemented! No changes needed)*

---

## Recommended Testing Sequence

### Day 1: Basic Validation

1. Test goal-based planner in static scene
2. Verify goals are generated and reached
3. Check coverage improvement (should be >80 cells)
4. Validate RUTF escape triggers when needed

### Day 2: Parameter Optimization

1. Try different `goal_distance` values (2.0, 2.5, 3.0)
2. Try different `k_att` values (0.6, 0.8, 1.0)
3. Find best combination for your scene

### Day 3: TP3 Experiments

1. **Test 1 (Cell Size):** Run with 0.01, 0.1, 0.5m
2. **Test 2 (Static):** 2 positions, best cell size
3. **Test 3 (Dynamic):** 2 positions with human

### Day 4: Analysis & Documentation

1. Compare maps (reactive vs goal-based)
2. Analyze coverage statistics
3. Write final report section on navigation strategy

---

## Final Recommendation

**For TP3 submission:**

✅ **Use Goal-Based Planner** if:
- You have time to test and validate (~2-3 hours)
- You want better coverage and systematic exploration
- You want to cite academic research (RUTF paper)
- You want impressive results for your report

⚠️ **Use Reactive Planner** if:
- You're short on time (<1 hour before deadline)
- Current planner "good enough" for TP3 requirements
- Risk-averse (stick with what's working)

**Both planners meet TP3 requirements!** Choose based on your timeline and goals.

---

**Quick Start:** Follow Option 1 above, run tests, done! 🚀

**Status:** ✅ IMPLEMENTATION COMPLETE - READY FOR TESTING

