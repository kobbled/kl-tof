# lib/sensors/tof — AI Context File

## Purpose

`tof` is a FANUC Karel library providing analog laser/Time-of-Flight (ToF) distance sensor integration for robot controllers. It exposes two GPP-instantiated class templates:

- **`tof_sensor`** — a self-contained ranging task that reads one analog input pin, applies a linear calibration (slope + intercept), maintains a rolling-window average over `MAX_SAMPLES` readings, and exports virtual or physical digital signals for `in-range` and `zeroed` states.
- **`scan_part`** (`scan_part_dyn` / `scan_part_stat`) — a coordinating scan task that combines `tof_sensor` ranging with robot position capture at interrupt-driven trigger points, converts positions to cylindrical or polar coordinates, and streams the fused `(theta, z, r)` data to a CSV file via a semaphore-protected pipe buffer.

The module runs on the FANUC controller as concurrent Karel tasks. The sensor task loops continuously updating `last_range` in DRAM while the scan task logs fused pose + range data on DOUT trigger interrupts.

---

## Repository Layout

```
lib/sensors/tof/
├── package.json                        # Default rossum manifest (dynamic scan)
├── package_default.json                # Manifest without cell config deps
├── package_macros.json                 # Manifest with cell1_config/cell2_config deps
├── package_static.json                 # Manifest for static (multi-group) scan build
├── README_FOR_STATIC_SCAN_BUILD.md    # Notes on building the static scan variant
├── LICENSE.txt
│
├── include/
│   ├── tof_sensor.c.klt                # Shared analog mode constants (MODE_*)
│   ├── tof_sensor_type.klt             # GPP: rolling window, SIGNAL_TYPE / ZERO_TYPE resolution
│   ├── tof_sensor.klt                  # TOF_T structure definition + VAR declarations
│   ├── tof_sensor.klh                  # Public header — declare_member stubs for all public routines
│   ├── tof_sensor.private.klh          # Private header — internal routine stubs (calib_, read_*_)
│   ├── tof_sensor.klc                  # Class body — all implementation (runs as Karel task)
│   │
│   ├── sensor_configs/
│   │   ├── keyence_IL300.klt           # Config for Keyence IL-300 (280 mm, ±5V, AI2, 30 ms)
│   │   ├── keyence_IL065.klt           # Config for Keyence IL-065 (50 mm, ±5V, AI2, 10 ms)
│   │   ├── panasonic_MLDS.klt          # Config for Panasonic MLDS (10 mm, 0–5V, AI3, 10 ms)
│   │   └── sensor_calib_test_settings.klt  # Test/simulation config (0–5V, AI1, slope=0.1)
│   │
│   └── scan_class/
│       ├── scan_part.klt               # Scan constants (PIPE_FILE, semaphore IDs, timing)
│       ├── scan_part.klh               # Public header for scan_part (new, init, set_*)
│       ├── scan_part.template.klt      # Macro definitions for sensor_object_define / define_scan_object
│       ├── scan_part_dyn.klc           # Dynamic scan: LPOS from robot frame transform
│       └── scan_part_stat.klc          # Static scan: LPOS from raw joint components
│
├── tp/
│   └── scn_new.kl                      # TP-callable shim: calls new on the scan object
│
└── test/
    ├── test_tof.kl                     # Standalone tof_sensor test (averaging, signal, zero)
    ├── tst_sim_tof.kl                  # Companion task: replays a CSV into the simulated AI pin
    ├── test_scan.kl                    # Integration test for scan_part (simulated AI)
    ├── tof_rand.csv                    # Test fixture: random analog values
    ├── tof_sign.csv                    # Test fixture: signal edge test values
    ├── tof_zero.csv                    # Test fixture: zeroed-state test values
    └── config/
        ├── defaultcell.klt             # Cell topology (frame nos, groups, scan macros) for tests
        ├── defaultscan.klt             # Scan object config (object name, flags, filename, sensor)
        └── base.motion.impl.klt        # Motion data type bindings (XYZWPR / gp1rbt)
```

---

## Full API Reference

### Constants — `tof_sensor.c.klt`

| Constant | Value | Meaning |
|----------|-------|---------|
| `MODE_4_20MA` | `1` | 4–20 mA current loop (not implemented — raises `INVALID_TYPE_CODE`) |
| `MODE_0_5V` | `2` | 0–5 V unipolar analog input |
| `MODE_5_5V` | `3` | −5 V to +5 V bipolar analog input |

### GPP Parameters — resolved before class expansion

These must be `%define`d in a sensor config `.klt` before `%class`-expanding `tof_sensor.klc`:

| Macro | Type | Description |
|-------|------|-------------|
| `SENSOR_PIN` | INTEGER | Analog input port number (`AIN[SENSOR_PIN]`) |
| `SENS_MODE` | `MODE_*` | Analog signal type |
| `SENS_SLOPE` | REAL | Linear calibration slope (mm per raw count) |
| `SENS_INTER` | REAL | Linear calibration intercept (mm) |
| `SENS_MAX` | INTEGER | Maximum valid raw analog count |
| `TOTAL_RANGE` | INTEGER | Sensor total range in mm (informational) |
| `SEARCH_DIST_SENSOR` | INTEGER | Acceptable search distance for the sensor in mm |
| `SIGNAL_TYPE` | `VIRTUAL` or `PHYSICAL` | How signal detection is exposed: compute via DOUT or read via DIN |
| `SIGNAL_PIN` | INTEGER | DOUT (VIRTUAL) or DIN (PHYSICAL) port number for in-range signal |
| `ZERO_TYPE` | `VIRTUAL` or `PHYSICAL` | How zero detection is exposed |
| `ZERO_PIN` | INTEGER | DOUT (VIRTUAL) or DIN (PHYSICAL) port number for zeroed signal |
| `MAX_SAMPLES` | INTEGER | Rolling window size (default `4`) |
| `SAMPLE_MS` | INTEGER | Delay between samples in milliseconds |

### `tof_sensor_type.klt` — derived constants

| Macro | Derived Value | Description |
|-------|---------------|-------------|
| `ZERO_TOLERANCE` | `0.25` | Absolute mm threshold for `read_zero` |
| `SIGNAL_TYPE_STR` | `DOUT[SIGNAL_PIN]` or `DIN[SIGNAL_PIN]` | Port reference resolved at compile time |
| `ZERO_TYPE_STR` | `DOUT[ZERO_PIN]` or `DIN[ZERO_PIN]` | Port reference resolved at compile time |

### `TOF_T` Structure — `tof_sensor.klt`

```
TOF_T FROM class_name = STRUCTURE
  pin          : BYTE     -- analog input pin number
  mode         : BYTE     -- SENS_MODE constant
  m            : REAL     -- calibration slope
  b            : REAL     -- calibration intercept
  max          : SHORT    -- max valid raw analog value
  sample_time  : SHORT    -- delay between samples (ms)
  meas_type    : BYTE     -- 1 = single, 2 = rolling average
  last_index   : INTEGER  -- current position in rolling window
  measuremnts  : ARRAY[MAX_SAMPLES] OF REAL
  time_out     : BOOLEAN
  digital      : BYTE
ENDSTRUCTURE
```

DRAM variables (shared across tasks via `FROM class_name`):
- `last_range` — most recent calibrated measurement (mm); UNINIT when out of range
- `zeroed` — TRUE when |last_range| < ZERO_TOLERANCE
- `is_signal` — TRUE when sensor is in range
- `prev_signal`, `seen` — edge-detection state

### `tof_sensor` Public API — `tof_sensor.klh`

| Routine | Signature | Description |
|---------|-----------|-------------|
| `new` | `()` | Initialize TOF_T from compile-time sensor config macros; sets meas_type=1 |
| `delete` | `()` | Uninitialize measurement array and clear pin |
| `set_measurement_type` | `(meas_type : INTEGER)` | 1 = single measurement, 2 = rolling average; ABORT on invalid |
| `set_sample_time` | `(smpl_tme : INTEGER)` | Override sample delay in ms |
| `set_sim_analog` | `(meas_mm : REAL)` | Convert mm to raw counts and write to simulated AI pin (requires `SET_PORT_SIM` first) |
| `read_measurement` | `() : REAL` | Take one calibrated reading; returns UNINIT if out of range |
| `read_range` | `() : REAL` | Take MAX_SAMPLES readings, return rolling average; returns 0 if all UNINIT |
| `read_signal` | `() : BOOLEAN` | Read/compute in-range flag; updates `is_signal` and SIGNAL pin |
| `read_zero` | `() : BOOLEAN` | Read/compute zeroed flag; updates `zeroed` and ZERO pin |
| `watch_rising_edge` | `() : BOOLEAN` | TRUE on first call after `is_signal` transitions FALSE→TRUE |
| `watch_falling_edge` | `() : BOOLEAN` | TRUE on first call after `is_signal` transitions TRUE→FALSE |
| `watch_change` | `() : BOOLEAN` | TRUE on first call after any change in `is_signal` |

**Task main body** (`BEGIN ... END class_name`): loops continuously — reads measurement or range based on `meas_type`, updates signal and zero flags, delays `sample_time` ms, updates `prev_signal` only when the previous change has been `seen` by the caller.

### Private Routines (not for external use)

| Routine | Description |
|---------|-------------|
| `calib_() : REAL` | Convert `AIN[pin]` to mm via `m*AIN + b`; applies mode-specific formula |
| `read_measurement_()` | Update `last_range`; set UNINIT if out of range |
| `read_range_()` | Append one calibrated reading to rolling window; call `math__average_real` |
| `read_signal_()` | Compute `is_signal` and update the signal port |
| `read_zero_()` | Compute `zeroed` and update the zero port |

---

### `scan_part` Constants — `scan_part.klt`

| Constant | Value | Description |
|----------|-------|-------------|
| `PIPE_FILE` | `'PIP:SCANDATA.DAT'` | Karel pipe for buffered CSV writing |
| `TOF_SEMA` | `1` | Semaphore ID for tof task coordination |
| `CSV_SEMA` | `2` | Semaphore ID for pipe/CSV write serialization |
| `INRPT_LOGPOS` | `1` | Interrupt condition number |
| `MAX_WAIT` | `250` | Max wait cycles for pipe write (unused in current code) |
| `CYCLE_TIME` | `100` | ms between pipe-flush checks |
| `MAX_DELAY` | `5000` | ms timeout for semaphore pend during flush |
| `MAX_PIPE` | `200` | Maximum pipe lines before flush to CSV |

### `scan_part` GPP Parameters — `scan_part.template.klt`

| Macro | Description |
|-------|-------------|
| `SCANNING_OBJECT_NAME` | Karel program name of the scan instance (e.g. `tstscn`) |
| `SCANNING_CNFG` | Config `.klt` file name to `%include` into the scan object |
| `SCN_PATH_FLG` | Flag number to pause robot motion during CSV write |
| `SCAN_TRIGGER_PIN` | DOUT number used as interrupt trigger for point capture |
| `SCN_END_FLG` | Flag number to signal end of scanning |
| `SCAN_FILENAME` | Output CSV filename on `FR:` |
| `scan_trigger_condition` | GPP macro — trigger expression (e.g. `DOUT[50]`) |
| `scan_trigger_reset` | GPP macro — resets trigger pin after logging |
| `SENSOR_CLASS` | Karel name of the tof_sensor instance (e.g. `tof1`) |
| `SENSOR_TEMPLATE` | Config `.klt` file for the sensor |
| `sensor_object_define(name, cfg)` | GPP macro: expands `%class name('tof_sensor.klc', ...)` |
| `define_scan_object(name, cfg)` | GPP macro: expands `%class name('scan_part_dyn.klc', ...)` or `scan_part_stat.klc` |

### `scan_part` Public API — `scan_part.klh`

| Routine | Signature | Description |
|---------|-----------|-------------|
| `new` | `()` | Initialize coord_sys to PTH_CYLINDER / Z_AXES; orient to (0,0,0) |
| `init` | `(crd_sys : INTEGER; crd_axs : INTEGER; orientation : VECTOR)` | Set coordinate system and tool orientation vector |
| `set_orientation` | `(orientation : VECTOR)` | Update orientation vector only |
| `set_coord_sys` | `(crd_sys : INTEGER; crd_axs : INTEGER)` | Update coordinate system and axis |
| `set_scan_finished` | `()` | Set `SCN_END_FLG` to 1 to terminate the scan loop |

**Task main body** (runs as Karel task): sets up interrupt on `scan_trigger_condition`, initializes csvpip pipe, starts sensor task, loops writing pipe to CSV when pipe reaches `MAX_PIPE` lines, terminates when `SCN_END_FLG > 0`.

**`log_pose` (internal)**: triggered by interrupt — captures robot LPOS, converts to cylindrical (theta,z,r), offsets radius by `last_range` if not UNINIT, writes vector to CSV pipe under semaphore.

---

### Cell Configuration Macros — `defaultcell.klt` (example)

| Macro | Example | Description |
|-------|---------|-------------|
| `CD_PAIR` | `1` | Coordinated motion pair number |
| `USER_FRAME_NO` | `2` | User frame to activate for LPOS |
| `TOOL_FRAME_NO` | `3` | Tool frame to activate for LPOS |
| `RBT_GRP` | `1` | Primary robot motion group |
| `ROT_GRP` | `2` | Rotary axis group (static scan) |
| `ROT_AXES` | `1` | Number of rotary joints |
| `ROTARY_JNT` | `1` | Joint index of the rotary axis |
| `ROT_DIRECTION` | `-1` | Sign convention for rotary angle |
| `TRACK_GRP` | `0` | Track group (0 = no track) |
| `MOTION_DATA_TYPE` | `XYZWPR` | Robot position type |
| `MOTION_DATA_FILE` | `gp1rbt` | Robot motion implementation program |

---

## Core Patterns

### Pattern 1 — Instantiate and run a tof_sensor as a background task

```karel
-- In program header:
%class tof1('tof_sensor.klc','tof_sensor.klh','keyence_IL300.klt')

VAR
  last_range FROM tof1 : REAL  -- shared DRAM variable
  tof_task : BOOLEAN

-- In BEGIN block:
tof1__new
tof1__set_measurement_type(2)   -- use rolling average
tof1__set_sample_time(30)       -- 30 ms per sample (IL-300 default)

tof_task = task__thread('tof1')
IF NOT tof_task THEN
  karelError(RUN_TASK_FAILED, 'tof sensor did not start', ER_ABORT)
ENDIF

-- Use last_range throughout; it updates automatically in background
-- Clean up:
tof_task = task__abort('tof1')
tof1__delete
```

### Pattern 2 — Read a single measurement (no background task)

```karel
%class tof1('tof_sensor.klc','tof_sensor.klh','keyence_IL065.klt')

VAR
  dist_mm : REAL

tof1__new
dist_mm = tof1__read_measurement
IF UNINIT(dist_mm) THEN
  -- sensor out of range; handle appropriately
ENDIF
tof1__delete
```

### Pattern 3 — Simulation mode for testing without hardware

```karel
-- Enable simulation on the AI port before creating the sensor:
SET_PORT_SIM(io_anin, SENSOR_PIN, 0, status)

tof1__new
-- Drive a specific distance into the sensor:
tof1__set_sim_analog(150.0)   -- simulate 150 mm reading

dist_mm = tof1__read_measurement
-- ...
CLR_PORT_SIM(io_anin, SENSOR_PIN, status)
```

### Pattern 4 — Edge detection for workpiece presence

```karel
-- Run sensor task in background (Pattern 1), then:
REPEAT
  IF tof1__watch_rising_edge THEN
    -- workpiece entered sensor range
    WRITE TPDISPLAY('Part detected', CR)
  ENDIF
  IF tof1__watch_falling_edge THEN
    -- workpiece left sensor range
    WRITE TPDISPLAY('Part removed', CR)
  ENDIF
  DELAY 50
UNTIL(done)
```

### Pattern 5 — Launch a dynamic scan_part scan

```karel
-- In config file (e.g. defaultcell.klt), define cell topology and:
--   define_scan_object(scnprt, defaultcell.klt)
-- In main program:

%include defaultcell.klt
define_scan_object(SCANNING_OBJECT_NAME, SCANNING_CNFG)

VAR
  orient : VECTOR

orient = VEC(0,0,0)
`SCANNING_OBJECT_NAME`__init(PTH_CYLINDER, Z_AXES, orient)

IF NOT task__thread('SCANNING_OBJECT_NAME') THEN
  karelError(RUN_TASK_FAILED, 'scan task did not start', ER_ABORT)
ENDIF

-- Run robot motion program; it pulses SCAN_TRIGGER_PIN DOUT to record points
-- When path is complete:
`SCANNING_OBJECT_NAME`__set_scan_finished
-- Wait for task to clean up
```

### Pattern 6 — Custom sensor configuration

```karel
-- Create mySensor.klt:
--   %define SENSOR_PIN  4
--   %define SENS_MODE   MODE_0_5V
--   %define SENS_SLOPE  0.08
--   %define SENS_INTER  0.0
--   %define SENS_MAX    1023
--   %define TOTAL_RANGE 100
--   %define SEARCH_DIST_SENSOR 80
--   %define SIGNAL_TYPE VIRTUAL
--   %define SIGNAL_PIN  40
--   %define ZERO_TYPE   VIRTUAL
--   %define ZERO_PIN    41
--   %define MAX_SAMPLES 4
--   %define SAMPLE_MS   20

%class mysensor('tof_sensor.klc','tof_sensor.klh','mySensor.klt')
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Reading `last_range` before the sensor task has completed at least one cycle | Always reads 0 or UNINIT even with a valid target | After `task__thread('tof1')`, add `DELAY sample_time * MAX_SAMPLES` before first read |
| Calling `watch_rising_edge` / `watch_falling_edge` without running the sensor task loop | `prev_signal` never updates; edge detection always returns FALSE | Always run the sensor as a background task before using edge detection; these routines depend on `prev_signal` being updated in the `BEGIN...END` loop |
| Setting `SIGNAL_TYPE PHYSICAL` without wiring a DI | Build succeeds but `read_signal` reads garbage | Use `VIRTUAL` during development; `PHYSICAL` only when a real DI is wired to the sensor's output |
| Forgetting `SET_PORT_SIM` before `set_sim_analog` | `PORT_NOT_SIMULATED` error abort | Call `SET_PORT_SIM(io_anin, SENSOR_PIN, 0, status)` before `tof1__new` in simulation |
| Using `read_range` directly (not via the task) on a slow loop | `read_range` blocks for `MAX_SAMPLES * sample_time` ms each call | Let the background task maintain `last_range`; read the DRAM variable directly |
| Not resetting `SCN_PATH_FLG` and `SCN_END_FLG` before starting a scan | Scan immediately terminates or motion never resumes | These are reset inside `scan_part` `BEGIN`, but if the robot was stopped mid-scan previously the flags may be stale; power-cycle or call `SET_PORT_VAL` manually |
| Using `MODE_4_20MA` | Immediate ABORT with `INVALID_TYPE_CODE` | 4–20 mA is not implemented; use MODE_0_5V or MODE_5_5V |

---

## Dependencies

### What `tof` Depends On

| Package | Used For |
|---------|----------|
| `errors` | `karelError`, `CHK_STAT`, `SET_UNINIT_R`, named error codes |
| `math` | `math__average_real` (rolling window average) |
| `display` | `display__clear`, `display__show`, `usrdis` log class in scan_part |
| `registers` | `get_string` in scan_part |
| `pose` | `pose__cartesian_to_cylindrical`, `pose__set_userframe/toolframe` in scan_part |
| `pathlib` | `PTH_CYLINDER`, `PTH_POLAR`, `Z_AXES`, `t_COORD_SYS` constants and types |
| `pathmotion` | Motion implementation interface (robot pose capture) |
| `csv` | `csvclass` pipe + file writing in scan_part |
| `multitask` | `task__thread`, `task__abort`, `task__is_task_running` |
| `systemlib` | `VEC`, `ZEROPOS` |
| `Strings` | `i_to_s`, `r_to_s`, `pose_to_s`, `vec_to_s` in scan_part |
| `ktransw-macros` | `declare_member`, `namespace.m`, `header_guard.m` |
| `KUnit` | Test framework (tests-depends only) |

### What Depends on `tof`

| Package | Role |
|---------|------|
| `lib/paths/pathforms` | Lists `tof-sensor` as a dependency for the TP forms system |

The module is otherwise consumed by end-user programs (scan orchestration scripts, cell-level Karel programs) rather than other Ka-Boost library packages.

---

## Build and Integration Notes

### Choosing a package.json variant

| File | When to use |
|------|-------------|
| `package.json` | Standard dynamic scan build (default) |
| `package_default.json` | When no cell config packages (`cell1_config`, `cell2_config`) exist |
| `package_macros.json` | When cell config packages provide cell topology macros |
| `package_static.json` | Static scan build using raw joint values instead of FK frame transforms |

Copy the desired variant to `package.json` before running rossum.

### Dynamic vs Static Scan

- **`scan_part_dyn.klc`** — Uses FK frame transforms via `MOTION_DATA_FILE__get_cart` to get Cartesian LPOS, then calls `pose__cartesian_to_cylindrical`. Use when the robot moves in Cartesian space.
- **`scan_part_stat.klc`** — Reads joint components directly (`get_component`) for multi-group cells where a rotary turntable group provides theta and a linear track group provides z. Use when a rotary positioner (not FK) provides the cylindrical coordinate.

### Class Instantiation

The `tof_sensor` and `scan_part` classes are GPP templates. They are instantiated with `%class`:

```
-- tof_sensor:
%class tof1('tof_sensor.klc','tof_sensor.klh','keyence_IL300.klt')

-- scan_part (dynamic):
%class scnprt('scan_part_dyn.klc','scan_part.klh','defaultcell.klt')

-- scan_part (static):
%class scnprt('scan_part_stat.klc','scan_part.klh','defaultcell.klt')
```

Or use the helper macros from `scan_part.template.klt`:

```
sensor_object_define(tof1, keyence_IL300.klt)
define_scan_object(scnprt, defaultcell.klt)
```

### `scn_new.kl` / TP Entry Point

`tp/scn_new.kl` is a thin Karel wrapper that calls `new` on the scan object. It can be invoked from a FANUC TP program to initialize the scan object before running the motion path.

### DEBUG_BUILD Macro

When `DEBUG_BUILD=TRUE` is passed to rossum macros, scan_part enables verbose debug logging via `usrdis`. The default in `package.json` is `DEBUG_BUILD=FALSE`.

### Signal/Zero Pin Behavior

With `VIRTUAL` signal type: the sensor task computes the boolean from the analog reading and **writes** to the DOUT pin. This allows TP logic to monitor the signal without polling Karel.

With `PHYSICAL` signal type: the sensor task **reads** from a DIN pin. The external sensor hardware must provide the digital output.

### Rolling Average Behavior

`read_range` takes `MAX_SAMPLES` successive readings each time it is called directly. The background task loop calls `read_range_` (private) once per `sample_time` ms, maintaining a continuous rolling window. Reading `last_range FROM class_name` from another task gives the most recent averaged value without blocking.

If all samples are UNINIT (e.g., sensor consistently out of range), `math__average_real` returns `0` — not UNINIT. Code consuming `last_range` should explicitly check for unexpected zero values in addition to `UNINIT`.
