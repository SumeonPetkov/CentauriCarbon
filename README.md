# CentauriCarbon
Learn more about ELEGOO Centauri Carbon 3D printer: https://www.elegoo.com/products/centauri-carbon

This 3D printer system utilizes the Allwinner R528 chip as its main control platform, integrating a DSP unit and an MCU to provide a complete 3D printing solution.

The firmware serves as the core control module, encapsulating all 3D printing control algorithms and logic, including G-code parsing, motion path planning, temperature control strategies, auto-leveling, and time-lapse photography.

The DSP and MCU together form the system's real-time control core, responsible for managing a wide range of input/output signals and peripheral devices, such as multi-axis control of the printer, temperature monitoring and regulation of the heated bed and extruder, limit switch status detection, and fan speed control.

## Multi-Bed Support Architecture

### Scope
- Multiple named bed surfaces (for example: `Glass`, `PEI`, `Textured Steel`)
- Per-surface calibration, including optional per-temperature variants
- Printer remains source of truth for surface and calibration state
- Slicer synchronization support (list/state sync and optional edits)
- Profile selection at print start for local and remote print workflows

### Canonical Data Model (Firmware-owned)
- `BedSurface`
  - `id` (stable unique ID)
  - `name` (user-facing label)
  - `updated_at` (monotonic timestamp)
  - `revision_hash` (for fast sync)
- `CalibrationProfile`
  - `surface_id`
  - `default_calibration` (required baseline, e.g. Z offset)
  - `temperature_map` (optional: bed temp -> calibration payload)
  - `updated_at`, `revision_hash`
- `ActiveSelection`
  - `active_surface_id`
  - `active_temp_key` (optional)
  - `selection_source` (`local_ui` or `remote_job`)

### Firmware / Backend Responsibilities
- Persist surface and calibration records in onboard storage (reboot-safe).
- Expose CRUD for surfaces and calibration data:
  - list/get/add/rename/delete surfaces
  - get/set calibration (default and per-temperature entries)
  - query/set active profile selection
- Validate print-start profile requests against known state.
- Apply selected calibration before print motion begins.
- Support local UI and remote job starts through one shared selection path.

### Printer UI Responsibilities
- Provide profile management flows:
  - add, rename, delete, and calibrate named surfaces
  - edit per-temperature calibration entries where enabled
- At print start, clearly show selected surface and effective calibration/temp.
- Allow user override when a remote job requests a different surface.
- Prevent ambiguity by always presenting name + temp context together.
- Integrate with the existing data-driven UI resource pipeline (`widget.csv`/bin, translations, widget mapping).

### Slicer Responsibilities
- Allow selecting target bed surface in print settings/job metadata.
- Sync with printer by querying surface list and revision metadata.
- Optionally push profile create/update requests when printer policy allows.
- Warn or block when requested surface/profile is unavailable or mismatched.
- Include selected surface/profile identity in remote print start request.

### API / Transport Responsibilities
- Surface management:
  - `ListSurfaces`, `GetSurface`, `CreateSurface`, `RenameSurface`, `DeleteSurface`
- Calibration management:
  - `GetCalibration(surface_id)`, `SetCalibration(surface_id, payload)`
  - optional temperature-keyed get/set operations
- Sync:
  - return `updated_at` and/or `revision_hash` per surface/profile
  - support incremental refresh (avoid full dumps when unchanged)
- Print start contract:
  - print header/request includes `surface_id` (+ optional temp key/hash)
  - printer responds `accepted`, `rejected`, or `fallback_applied`

### Print Start Flow (Local and Remote)
1. Determine requested profile:
   - local print: selected in printer UI
   - remote print: slicer-provided profile, optionally user-confirmed
2. Firmware validates profile existence and compatibility.
3. Firmware resolves effective calibration (default or temp-specific).
4. Firmware applies calibration and records active selection.
5. Print starts; active surface/calibration is queryable via API and visible in UI.

## Implemented Multi-Mesh Workflow (production path)

This repository now includes a real firmware-side mesh workflow using Klipper-compatible commands:

- `BED_MESH_PROFILE SAVE=<name>|LOAD=<name>|REMOVE=<name>`
- `SET_ACTIVE_MESH MESH=<name>`
- `CALIBRATE_BED_SURFACE BED=<bed> SURFACE=<surface> TEMP=<temp> [SOAK=<sec>] [HOME=1] [PROFILE=<name>]`

### Deterministic profile names

`CALIBRATE_BED_SURFACE` uses deterministic naming:

`<bed>_<surface>_<temp>`

Examples:

- `orig_smooth_60`
- `orig_textured_60`
- `aftermarket_smooth_70`

Names are normalized to lowercase `a-z`, `0-9`, and `_`.

### Print-start mesh precedence (local + remote)

`SDCARD_PRINT_FILE` now resolves mesh at print start in this order:

1. explicit `MESH=<name>` parameter on `SDCARD_PRINT_FILE`
2. active mesh stored in `[bed_mesh] active_mesh_profile`
3. fallback legacy profile (`default`) for the currently selected platform

If the selected/fallback profile does not exist, print start is aborted with an error.

### Practical ElegooSlicer usage

If your remote start path can pass `MESH` to `SDCARD_PRINT_FILE`, use:

- `SDCARD_PRINT_FILE FILESLICE=<...> FILENAME="<...>" MESH=orig_smooth_60`

If it cannot, use start G-code (or preset macros) to set active profile before print:

- `SET_ACTIVE_MESH MESH=orig_smooth_60`

Then start print normally; print-start fallback will use the active profile.
