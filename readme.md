# tof-sensor

Time-of-Flight / laser distance sensor integration for FANUC robot controllers — analog input reading, linear calibration, rolling-window averaging, digital signal output, and coordinated part-surface scanning with CSV export.

---

## Overview

`tof-sensor` provides two GPP-instantiated Karel class templates that run as concurrent tasks on the FANUC controller:

**`tof_sensor`** reads an analog input pin, applies a linear calibration (slope + intercept), and continuously updates a rolling-window average in DRAM. It exposes in-range and zeroed boolean flags that can be written to virtual DOUT pins (computed by Karel) or read from physical DIN pins (external hardware output). Edge detection routines detect rising and falling signal transitions.

**`scan_part`** coordinates a surface scan. It fires on a DOUT interrupt trigger (a TA/TB instruction in the TP path program pulses DOUT), captures robot LPOS at that instant, converts position to cylindrical coordinates, offsets the radius by the current sensor reading, and streams fused `(theta, z, r)` points to a CSV file through a semaphore-protected pipe buffer.

Three hardware profiles are included for the Keyence IL-300, Keyence IL-065, and Panasonic MLDS sensors. Custom sensors require one `.klt` config file.

---

## Files

| File | Purpose |
|------|---------|
| `include/tof_sensor.c.klt` | Analog mode constants: `MODE_4_20MA`, `MODE_0_5V`, `MODE_5_5V` |
| `include/tof_sensor_type.klt` | GPP: rolling window defaults, signal/zero pin resolution |
| `include/tof_sensor.klt` | `TOF_T` structure and DRAM variable declarations |
| `include/tof_sensor.klh` | Public header — all `declare_member` stubs |
| `include/tof_sensor.private.klh` | Private header — internal routine stubs |
| `include/tof_sensor.klc` | Full class implementation; runs as a Karel task |
| `include/sensor_configs/keyence_IL300.klt` | Keyence IL-300 config (280 mm, ±5V, AI2, 30 ms) |
| `include/sensor_configs/keyence_IL065.klt` | Keyence IL-065 config (50 mm, ±5V, AI2, 10 ms) |
| `include/sensor_configs/panasonic_MLDS.klt` | Panasonic MLDS config (10 mm, 0–5V, AI3, 10 ms) |
| `include/sensor_configs/sensor_calib_test_settings.klt` | Test/sim config (0–5V, AI1, slope=0.1, intercept=0) |
| `include/scan_class/scan_part.klt` | Scan constants (pipe name, semaphore IDs, timing) |
| `include/scan_class/scan_part.klh` | Public header for scan_part (new, init, set_*) |
| `include/scan_class/scan_part.template.klt` | Helper macros: `sensor_object_define`, `define_scan_object` |
| `include/scan_class/scan_part_dyn.klc` | Dynamic scan: uses FK frame → `pose__cartesian_to_cylindrical` |
| `include/scan_class/scan_part_stat.klc` | Static scan: reads raw joint components (multi-group cell) |
| `tp/scn_new.kl` | TP-callable shim: calls `new` on the scan object |
| `test/test_tof.kl` | Unit test: averaging, signal detection, zero detection |
| `test/tst_sim_tof.kl` | Companion task: replays CSV values into simulated AI pin |
| `test/test_scan.kl` | Integration test for scan_part |
| `test/tof_rand.csv` | Fixture: random analog values |
| `test/tof_sign.csv` | Fixture: signal edge values |
| `test/tof_zero.csv` | Fixture: zeroed-state values |
| `test/config/defaultcell.klt` | Example cell topology config for tests |
| `test/config/defaultscan.klt` | Example scan object config for tests |
| `test/config/base.motion.impl.klt` | Motion data type bindings (XYZWPR / gp1rbt) |

---

## API Reference

### Analog Mode Constants (`tof_sensor.c.klt`)

```
MODE_4_20MA = 1   -- 4–20 mA current loop  (NOT IMPLEMENTED — causes ABORT)
MODE_0_5V   = 2   -- 0 to +5 V unipolar
MODE_5_5V   = 3   -- −5 to +5 V bipolar
```

### Sensor Config Parameters (define in a `.klt` before class expansion)

| Macro | Example | Description |
|-------|---------|-------------|
| `SENSOR_PIN` | `2` | Analog input number (`AIN[n]`) |
| `SENS_MODE` | `MODE_5_5V` | Analog signal type |
| `SENS_SLOPE` | `0.14` | Calibration slope: mm = slope × raw + intercept |
| `SENS_INTER` | `-573.44` | Calibration intercept |
| `SENS_MAX` | `1000` | Maximum valid raw analog count; above = out of range |
| `TOTAL_RANGE` | `280` | Total sensor range in mm (documentation) |
| `SEARCH_DIST_SENSOR` | `80` | Acceptable search distance for application logic |
| `SIGNAL_TYPE` | `VIRTUAL` | `VIRTUAL` = compute and write DOUT; `PHYSICAL` = read DIN |
| `SIGNAL_PIN` | `33` | DOUT/DIN pin for in-range flag |
| `ZERO_TYPE` | `VIRTUAL` | `VIRTUAL` = compute and write DOUT; `PHYSICAL` = read DIN |
| `ZERO_PIN` | `34` | DOUT/DIN pin for zeroed flag |
| `MAX_SAMPLES` | `4` | Rolling window depth |
| `SAMPLE_MS` | `30` | Delay between samples (ms) |

### `tof_sensor` Public Routines

```
-- Lifecycle
class_name__new
    Initialize sensor from compile-time config.
    Sets meas_type = 1 (single), last_index = 1, uninitializes measurement array.

class_name__delete
    Clear pin and uninitialize measurement array.

-- Configuration
class_name__set_measurement_type(meas_type : INTEGER)
    1 = single reading per cycle
    2 = rolling-window average per cycle
    Any other value = ABORT with INVALID_TYPE_CODE.

class_name__set_sample_time(smpl_tme : INTEGER)
    Override sample delay in ms.

class_name__set_sim_analog(meas_mm : REAL)
    Convert a distance in mm back to raw analog counts and write to the
    simulated AI pin. Requires SET_PORT_SIM to be called first.

-- Ranging
class_name__read_measurement : REAL
    Single calibrated reading. Returns UNINIT if analog is out of range.

class_name__read_range : REAL
    Blocks for MAX_SAMPLES cycles, returns rolling average.
    Prefer the background task + DRAM variable for non-blocking access.

-- Digital signals
class_name__read_signal : BOOLEAN
    Compute or read the in-range boolean. Updates is_signal and SIGNAL pin.

class_name__read_zero : BOOLEAN
    Compute or read the zeroed boolean (|range| < 0.25 mm). Updates zeroed.

-- Edge detection (requires background task running)
class_name__watch_rising_edge : BOOLEAN
    TRUE once after is_signal transitions FALSE → TRUE.

class_name__watch_falling_edge : BOOLEAN
    TRUE once after is_signal transitions TRUE → FALSE.

class_name__watch_change : BOOLEAN
    TRUE once after any change in is_signal.
```

DRAM variables accessed from other tasks via `FROM class_name`:

```
last_range FROM class_name : REAL     -- current averaged reading (mm), UNINIT = out of range
zeroed     FROM class_name : BOOLEAN  -- TRUE when |last_range| < 0.25 mm
is_signal  FROM class_name : BOOLEAN  -- TRUE when sensor has a valid in-range reading
```

### `scan_part` Public Routines

```
class_name__new
    Default init: PTH_CYLINDER coordinate system, Z_AXES, orientation (0,0,0).

class_name__init(crd_sys : INTEGER; crd_axs : INTEGER; orientation : VECTOR)
    Set coordinate system, axis selection, and tool orientation vector.

class_name__set_orientation(orientation : VECTOR)
    Update orientation vector only.

class_name__set_coord_sys(crd_sys : INTEGER; crd_axs : INTEGER)
    Update coordinate system type and axis.

class_name__set_scan_finished
    Set SCN_END_FLG flag to terminate the scan task loop.
    TP programs may set F[SCN_END_FLG] = ON directly instead.
```

### Scan Part Config Macros (`scan_part.template.klt`)

| Macro | Example | Description |
|-------|---------|-------------|
| `SCANNING_OBJECT_NAME` | `scnprt` | Karel program name for scan instance |
| `SCANNING_CNFG` | `defaultcell.klt` | Config file passed to `define_scan_object` |
| `SCN_PATH_FLG` | `113` | Flag to pause robot motion during CSV flush |
| `SCAN_TRIGGER_PIN` | `50` | DOUT that the TP path program pulses to trigger logging |
| `SCN_END_FLG` | `118` | Flag to terminate the scan loop |
| `SCAN_FILENAME` | `tofscan.csv` | Output filename written to `FR:` |
| `SENSOR_CLASS` | `tof1` | Name of the tof_sensor instance embedded in scan_part |
| `SENSOR_TEMPLATE` | `keyence_IL300.klt` | Sensor config file |

---

## Common Patterns

### Pattern 1 — Background ranging (most common)

```karel
-- 1. Expand the class with a sensor config
%class tof1('tof_sensor.klc','tof_sensor.klh','keyence_IL300.klt')

-- 2. Declare shared DRAM variable (access range from this task)
VAR
  last_range FROM tof1 : REAL
  tof_task   : BOOLEAN

-- 3. In BEGIN block:
tof1__new
tof1__set_measurement_type(2)   -- rolling average mode
-- give the task time to fill its rolling window before first use:
tof_task = task__thread('tof1')
IF NOT tof_task THEN
  karelError(RUN_TASK_FAILED, 'tof sensor did not start', ER_ABORT)
ENDIF
DELAY 200   -- wait ~4 samples at 30 ms each + margin

-- 4. Read the continuously-updated value:
IF NOT UNINIT(last_range) THEN
  -- use last_range ...
ENDIF

-- 5. Cleanup:
tof_task = task__abort('tof1')
tof1__delete
```

### Pattern 2 — Single blocking measurement

```karel
%class tof1('tof_sensor.klc','tof_sensor.klh','keyence_IL065.klt')
VAR
  dist_mm : REAL

tof1__new
dist_mm = tof1__read_measurement
IF UNINIT(dist_mm) THEN
  -- out of range: handle error or skip
ELSE
  -- use dist_mm
ENDIF
tof1__delete
```

### Pattern 3 — Simulation (RoboGuide / offline testing)

```karel
VAR
  status : INTEGER

-- Enable AI simulation before creating sensor:
SET_PORT_SIM(io_anin, SENSOR_PIN, 0, status)

%class tof1('tof_sensor.klc','tof_sensor.klh','sensor_calib_test_settings.klt')

tof1__new
tof1__set_sim_analog(75.0)   -- simulate 75 mm

tof_task = task__thread('tof1')
DELAY 100
-- last_range is now ~75 mm

tof_task = task__abort('tof1')
tof1__delete

CLR_PORT_SIM(io_anin, SENSOR_PIN, status)
```

### Pattern 4 — Workpiece edge detection

```karel
%class tof1('tof_sensor.klc','tof_sensor.klh','keyence_IL300.klt')
VAR
  tof_task : BOOLEAN

tof1__new
tof1__set_measurement_type(2)
tof_task = task__thread('tof1')
DELAY 200

WHILE NOT done DO
  IF tof1__watch_rising_edge THEN
    WRITE TPDISPLAY('Part entered range', CR)
    -- trigger action
  ENDIF
  IF tof1__watch_falling_edge THEN
    WRITE TPDISPLAY('Part left range', CR)
  ENDIF
  DELAY 50
ENDWHILE

tof_task = task__abort('tof1')
tof1__delete
```

### Pattern 5 — Dynamic part scan (6-axis robot + FK)

```karel
-- In defaultcell.klt, define:
--   SCANNING_OBJECT_NAME = scnprt
--   SCANNING_CNFG = defaultcell.klt
--   SCN_PATH_FLG = 113
--   SCAN_TRIGGER_PIN = 50
--   SCN_END_FLG = 118
--   SCAN_FILENAME = tofscan.csv
--   SENSOR_CLASS = tof1
--   SENSOR_TEMPLATE = keyence_IL300.klt
-- Also define cell topology macros (USER_FRAME_NO, TOOL_FRAME_NO, RBT_GRP, etc.)

%include defaultcell.klt
define_scan_object(SCANNING_OBJECT_NAME, SCANNING_CNFG)

VAR
  orient : VECTOR

orient = VEC(0, 0, 0)
`SCANNING_OBJECT_NAME`__init(PTH_CYLINDER, Z_AXES, orient)

IF NOT task__thread('SCANNING_OBJECT_NAME') THEN
  karelError(RUN_TASK_FAILED, 'scan task did not start', ER_ABORT)
ENDIF

-- Run TP motion program; TP program pulses DOUT[SCAN_TRIGGER_PIN]
-- via a TA instruction on each move to capture points.
-- When the path is complete:
`SCANNING_OBJECT_NAME`__set_scan_finished

-- Output file: FR:tofscan.csv with columns theta,r,z
```

### Pattern 6 — Custom sensor calibration

Create `my_sensor.klt`:

```
%define SENSOR_PIN  4
%define SENS_MODE   MODE_0_5V
%define SENS_SLOPE  0.08          -- mm per raw count
%define SENS_INTER  0.0
%define SENS_MAX    1023
%define TOTAL_RANGE 100
%define SEARCH_DIST_SENSOR 80
%define SIGNAL_TYPE VIRTUAL
%define SIGNAL_PIN  40
%define ZERO_TYPE   VIRTUAL
%define ZERO_PIN    41
%define MAX_SAMPLES 4
%define SAMPLE_MS   20
```

Then:

```karel
%class mysensor('tof_sensor.klc','tof_sensor.klh','my_sensor.klt')
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Reading `last_range` immediately after starting the sensor task | Always 0 or UNINIT even with a target in range | Add `DELAY sample_time * MAX_SAMPLES` after `task__thread` before first read |
| Using `watch_rising_edge` without the background task | Always returns FALSE; `prev_signal` never changes | Run the sensor as a background task; edge detection requires the main loop to update `prev_signal` |
| Setting `SIGNAL_TYPE PHYSICAL` without a wired DIN | `read_signal` returns noise/garbage | Use `VIRTUAL` in development; switch to `PHYSICAL` only with confirmed DIN wiring |
| Calling `set_sim_analog` without `SET_PORT_SIM` first | ABORT with `PORT_NOT_SIMULATED` | Call `SET_PORT_SIM(io_anin, SENSOR_PIN, 0, status)` before `tof1__new` |
| Setting `SENS_MODE MODE_4_20MA` | Immediate ABORT with `INVALID_TYPE_CODE` | Use `MODE_0_5V` or `MODE_5_5V`; 4–20 mA is unimplemented |
| Using `read_range` directly in the main task (not the background task) | Blocks for `MAX_SAMPLES × sample_time` ms per call | Let the background task maintain `last_range`; read the DRAM variable from your program |
| Not matching `SCANNING_CNFG` in `scan_part.template.klt` to the actual config file name | GPP expansion fails or wrong sensor/flags used | Ensure `%define SCANNING_CNFG` exactly matches the `.klt` filename passed to `define_scan_object` |
| Robot speed too high during scan | `CANT_WRITE` abort — pipe fills faster than CSV can flush | Reduce path speed so the pipe stays under `MAX_PIPE` (200) lines between flush cycles |

---

## Supported Sensors

| Model | Range | Sample Time | Mode | AI Port | Notes |
|-------|-------|-------------|------|---------|-------|
| Keyence IL-300 | 280 mm | 30 ms | ±5V (`MODE_5_5V`) | AI2 | Default for medium-range work |
| Keyence IL-065 | 50 mm | 10 ms | ±5V (`MODE_5_5V`) | AI2 | Short-range, faster sampling |
| Panasonic MLDS | ~10 mm | 10 ms | 0–5V (`MODE_0_5V`) | AI3 | Very short range proximity |

---

## Build Flow

### Standard Build (dynamic scan)

```shell
cd lib/sensors/tof
del /f robot.ini && setrobot
mkdir build && cd build
rossum .. -w -o
ninja
kpush
```

To include tests:

```shell
rossum .. -w -o -t
```

### Static Scan Build

See `README_FOR_STATIC_SCAN_BUILD.md`. Copy `package_static.json` to `package.json` before building.

### Selecting a `package.json` Variant

| File | Use When |
|------|----------|
| `package.json` | Default dynamic scan (no external cell config packages) |
| `package_default.json` | Explicit default; exposes test targets separately |
| `package_macros.json` | Cell config packages (`cell1_config`, `cell2_config`) exist in workspace |
| `package_static.json` | Building the static (multi-group turntable) scan variant |

### Dependencies

```
ktransw-macros
errors
math
Strings
systemlib
display
registers
pose
pathlib
pathmotion
csv
multitask
KUnit (tests only)
```
