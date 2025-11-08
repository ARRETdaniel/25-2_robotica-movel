# QUICK START: Goal-Based Exploration Planner

**3-Minute Setup Guide**

---

## Step 1: Update Imports (Cell 1)

Find this line:
```python
from utils.exploration_planner import ExplorationPlanner
```

Add below it:
```python
from utils.goal_based_exploration_planner import GoalBasedExplorationPlanner
```

---

## Step 2: Replace Planner (Cell 18)

**FIND:**
```python
planner = ExplorationPlanner(
    v_nominal=V_NOMINAL,
    d_safe=D_SAFE,
    cell_size=CELL_SIZE
)
```

**REPLACE WITH:**
```python
planner = GoalBasedExplorationPlanner(
    v_nominal=V_NOMINAL,
    d_safe=D_SAFE,
    goal_distance=2.5,      # Virtual goal distance (m)
    goal_threshold=0.5,     # Goal reached threshold (m)
    map_size=MAP_SIZE,
    cell_size=CELL_SIZE
)
```

---

## Step 3: Update Plan Call (Cell 19)

**FIND (around line 55):**
```python
v, w = planner.plan_step_with_escape(laser_data, current_pos=(x, y))
```

**REPLACE WITH:**
```python
v, w = planner.plan_step(laser_data, current_pos=(x, y), robot_theta=theta)
```

---

## Step 4: Run!

1. **Kernel** → **Restart Kernel**
2. Run Cell 1 (Imports)
3. Run Cell 2 (Connect)
4. Run Cell 17 (Config)
5. Run Cell 18 (Initialize)
6. Run Cell 19 (Main Loop)

---

## What to Look For

### Console Messages
```
Goal-Based Exploration Planner initialized:
  Strategy: Virtual goals + Potential Fields + RUTF escape
  Goal distance: 2.5 m
  ...

[Goal] New goal set: (2.34, -1.87)
[Goal] Goal #1 reached! New goal: (-0.58, 3.12)
[Goal] Goal #2 reached! New goal: (4.21, 0.93)
[Stuck] Detected! Triggering RUTF escape...
[RUTF] Escape queued: 20 steps, then new goal
```

### Expected Results
- **Visited cells:** >100 (vs ~60 for reactive)
- **Goals reached:** 5-10 in 60 seconds
- **Exploration:** Systematic, directed movement
- **Stuck events:** 1-2 (vs 2-4 for reactive)

---

## Troubleshooting

### "Module not found"
→ Restart kernel!

### Robot goes to edge
→ Check `MAP_SIZE = (10, 10)` matches scene

### Too many stuck events
→ Reduce `stuck_threshold` from 15 to 10

### Robot oscillates at goal
→ Increase `goal_threshold` from 0.5 to 1.0

---

## Revert to Old Planner

Just undo the 3 changes above:
1. Remove/comment the import
2. Change back to `ExplorationPlanner(...)`
3. Change back to `plan_step_with_escape(...)`

---

## Files Reference

📄 **Implementation:** `utils/goal_based_exploration_planner.py`  
📖 **Full Docs:** `docs/GOAL_BASED_EXPLORATION_IMPLEMENTATION.md`  
🧪 **Testing Guide:** `docs/TESTING_GUIDE_GOAL_BASED.md`  
📊 **Summary:** `docs/TP3_NAVIGATION_SUMMARY.md`  

---

## Your Idea Implemented ✅

You said:
> "Use random setter location x times distance of the current location"

Result:
- Random goals at `goal_distance` (2.5m) from current position
- Robot navigates towards each goal
- New goal generated when reached
- RUTF escape if stuck
- Systematic exploration!

---

**Ready in:** 3 minutes  
**Expected improvement:** 40-60% more coverage  
**Academic foundation:** IEEE paper + TP2 code  

🚀 **Let's go!**
