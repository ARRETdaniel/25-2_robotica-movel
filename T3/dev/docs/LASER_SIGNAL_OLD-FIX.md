# FastHokuyo Laser Signal Fix for CoppeliaSim 4.10.0

## Problem

The current `fastHokuyo.lua` script uses the **deprecated** `sim.setStringSignal()` function, which causes UTF-8 encoding errors when transmitting binary (packed float) data through the ZMQ Remote API:

```
TEXT: not UTF-8 text
{ret = {{data = [long string (2736 bytes)], }}, }
```

## Solution

Replace the deprecated string signal API with the **modern buffer property API** that properly handles binary data.

## Required Changes to fastHokuyo.lua

### Step 1: Open the Scene in CoppeliaSim

1. Open `cena-tp3-estatico.ttt` in CoppeliaSim 4.10.0
2. In the scene hierarchy, find `/kobuki/fastHokuyo`
3. Double-click on the fastHokuyo object
4. Click on the "Script" button to open the script editor

### Step 2: Locate the Signal Publishing Code

Find the lines that look like this (near the end of the script):

```lua
sim.setStringSignal('hokuyo_range_data', sim.packFloatTable(distData))
sim.setStringSignal('hokuyo_angle_data', sim.packFloatTable(angleData))
```

### Step 3: Replace with Modern Buffer Property API

**OLD CODE (Deprecated):**
```lua
sim.setStringSignal('hokuyo_range_data', sim.packFloatTable(distData))
sim.setStringSignal('hokuyo_angle_data', sim.packFloatTable(angleData))
```

**NEW CODE (Modern API):**
```lua
-- Use buffer properties instead of deprecated string signals
-- This avoids UTF-8 encoding errors with binary data in ZMQ Remote API
sim.setBufferProperty(sim.handle_scene, 'signal.hokuyo_range_data', sim.packFloatTable(distData))
sim.setBufferProperty(sim.handle_scene, 'signal.hokuyo_angle_data', sim.packFloatTable(angleData))
```

### Step 4: Save and Test

1. Save the script in CoppeliaSim
2. Save the scene (`cena-tp3-estatico.ttt`)
3. Restart the simulation
4. Run the Python notebook to test

## Python Code Update

The Python code in `kobuki_controller.py` has been updated to use:

```python
# Modern buffer property API (CoppeliaSim 4.10+)
ranges_packed = self.sim.getBufferProperty(
    self.sim.handle_scene, 
    'signal.hokuyo_range_data',
    {'noError': True}
)
angles_packed = self.sim.getBufferProperty(
    self.sim.handle_scene,
    'signal.hokuyo_angle_data', 
    {'noError': True}
)
```

Instead of the deprecated:
```python
# Deprecated (causes UTF-8 errors with binary data)
ranges_packed = self.sim.getStringSignal('hokuyo_range_data')
angles_packed = self.sim.getStringSignal('hokuyo_angle_data')
```

## Alternative: Add a Callable Function (If Editing Lua is Difficult)

If you cannot easily modify the Lua script's signal publishing, you can add this function to the **end of fastHokuyo.lua**:

```lua
function getLaserData()
    -- Return the laser data arrays directly
    -- Called from Python using sim.callScriptFunction()
    if not distData or not angleData then
        return nil, nil
    end
    return sim.packFloatTable(distData), sim.packFloatTable(angleData)
end
```

Then update the Python code to call this function:

```python
def get_laser_data(self) -> Optional[np.ndarray]:
    try:
        # Get the fastHokuyo object handle
        hokuyo_handle = self.sim.getObject('/kobuki/fastHokuyo')
        
        # Get the script handle (simulation script)
        script_handle = self.sim.getScript(
            self.sim.scripttype_simulation, 
            hokuyo_handle
        )
        
        # Call the getLaserData function
        ranges_packed, angles_packed = self.sim.callScriptFunction(
            'getLaserData', 
            script_handle
        )
        
        if not ranges_packed or not angles_packed:
            return None
        
        # Unpack
        ranges = self.sim.unpackFloatTable(ranges_packed)
        angles = self.sim.unpackFloatTable(angles_packed)
        
        # Combine
        laser_data = np.column_stack((angles, ranges))
        return laser_data
        
    except Exception as e:
        print(f"⚠ Error: {e}")
        return None
```

## References

- [CoppeliaSim Properties Documentation](https://manual.coppeliarobotics.com/en/properties.htm)
- [sim.setBufferProperty](https://manual.coppeliarobotics.com/en/regularApi/simSetBufferProperty.htm)
- [sim.getBufferProperty](https://manual.coppeliarobotics.com/en/regularApi/simGetBufferProperty.htm)
- [Deprecated: sim.setStringSignal](https://manual.coppeliarobotics.com/en/regularApi/simSetStringSignal.htm)
- [Deprecated: sim.getStringSignal](https://manual.coppeliarobotics.com/en/regularApi/simGetStringSignal.htm)

## Why This Fixes the Problem

1. **String signals** are deprecated and expect UTF-8 text strings
2. **Packed float data** is binary, not UTF-8 text
3. The ZMQ Remote API's CBOR encoder fails when trying to encode binary data as UTF-8
4. **Buffer properties** are designed for binary data and work correctly with ZMQ Remote API

---

**Status**: ✅ Python code updated | ⏳ Lua script needs manual update in CoppeliaSim
