# Quick Start: Creating CoppeliaSim Scenes from PNG Maps

## Simple 3-Step Workflow

### Step 1: Open CoppeliaSim
Start CoppeliaSim and create a new scene or open an existing one.

### Step 2: Run the Generator
```bash
conda activate tp2
python scripts/simple_heightfield_generator.py --map mapas/paredes.png
```

### Step 3: Save in CoppeliaSim
In CoppeliaSim: **File > Save Scene** (save as `.ttt` file)

---

## Examples

### Simple map with default settings (10m x 10m, 1m obstacles)
```bash
python scripts/simple_heightfield_generator.py --map mapas/paredes.png
```

### Larger environment
```bash
python scripts/simple_heightfield_generator.py --map mapas/circular_maze.png --width 15 --height 15
```

### Taller obstacles
```bash
python scripts/simple_heightfield_generator.py --map mapas/cave.png --obstacle-height 2.0
```

### Inverted map (if black = free space in your PNG)
```bash
python scripts/simple_heightfield_generator.py --map mapas/mymap.png --invert
```

---

## What It Does

1. **Loads** your PNG map
2. **Converts** white pixels → free space (height = 0), black pixels → obstacles (height = obstacle_height)
3. **Downsamples** if map is too large (>200x200) for CoppeliaSim performance
4. **Creates** heightfield shape in your current CoppeliaSim scene
5. **Positions** it at origin (0, 0, 0)
6. **Configures** gray color for terrain

---

## Adding a Robot Manually

After creating the heightfield, you can add a robot in CoppeliaSim:

1. **Menu**: Model browser > robots > mobile
2. **Select**: "Omnidirectional Platform.ttm" (holonomic YouBot)
3. **Drag & drop** into scene
4. **Position** robot above terrain (Z > 0.1m)

---

## Tips

- **CoppeliaSim must be running** before you execute the script
- Maps are automatically downsampled if too large (preserves performance)
- Use `--max-dim 150` if you want smaller/faster heightfields
- Save your scene in CoppeliaSim after the heightfield is created
- You can run the script multiple times (each run adds a new heightfield)

---

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--map`, `-m` | *required* | Path to PNG map file |
| `--width`, `-w` | 10.0 | World width in meters |
| `--height`, `-ht` | 10.0 | World height in meters |
| `--obstacle-height`, `-oh` | 1.0 | Height of obstacles in meters |
| `--invert`, `-i` | False | Invert map (black=free, white=obstacle) |
| `--threshold`, `-t` | 0.5 | Binarization threshold (0-1) |
| `--max-dim` | 200 | Max dimension before downsampling |

---

## Troubleshooting

### "Could not connect to CoppeliaSim"
**Solution**: Make sure CoppeliaSim is running.

### "Map file not found"
**Solution**: Check the path. Use paths relative to project root.

### Heightfield appears too small/large
**Solution**: Adjust `--width` and `--height` parameters to match your desired world size.

### Obstacles too high/low
**Solution**: Adjust `--obstacle-height` parameter (default is 1.0m).

---

## Integration with Your Path Planning

### Workflow:
1. **Plan path in 2D** using your roadmap algorithm on the PNG map
2. **Generate scene** using this script
3. **Test in 3D** by opening the scene in CoppeliaSim
4. **Validate** that the planned path works in simulation

### Example:
```python
# 1. Plan path (your TP2 code)
from utils.roadmap_planner import RoadmapPlanner
planner = RoadmapPlanner(occupancy_grid, 10, 10, robot_radius=0.2)
path = planner.plan(start=(1, 1), goal=(9, 9), num_samples=300)

# 2. Generate scene (run in terminal)
# python scripts/simple_heightfield_generator.py --map mapas/paredes.png

# 3. Open scene in CoppeliaSim and test
```

---

## Available Maps

All maps in `mapas/` folder can be converted:
- `paredes.png` - Simple walls (good for testing)
- `circular_maze.png` - Circular maze
- `square_maze.png` - Square maze
- `cave.png` - Cave-like environment
- `autolab.png` - Lab environment
- `icex-terceiro.png` - Building floor plan

---

**Author**: Daniel Terra Gomes  
**Course**: Mobile Robotics (TP2)  
**Date**: 2025
