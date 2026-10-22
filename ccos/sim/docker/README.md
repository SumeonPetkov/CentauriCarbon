# ccsim in Docker

Runs the emulator (ccos against a virtual Centauri Carbon) in a container and
shows its window in your web browser. Nothing else to install but Docker; no
X server or WSLg needed.

From the repository root:

```sh
docker compose -f ccos/sim/docker/compose.yaml up --build
```

Then open <http://localhost:6080>. Click the touchscreen on the left; the
right side is the bed seen from above (red cross = nozzle), the bottom is the
machine's state. It opens at its real pixel size; the buttons on top make it
smaller, bigger or fit the browser window (the choice is remembered). Ctrl+C
in the terminal stops it. The first build takes a
few minutes.

## Sending G-code and uploading files

Under the emulator the page has two panels. They talk to `ccsim --control`
on port 6081, which only the emulator has; ccos on a real printer has no
such port.

- **G-code console.** Type a command (`G28`, `BED_MESH_CALIBRATE S60 pla`, …)
  and press Enter; ↑/↓ go through your history. The box shows ccos's console:
  your commands in blue, then the firmware's messages and errors. Like on the
  printer, commands wait while a print runs, and commands that only answer
  their sender (`M114`, `GET_POSITION`) print nothing. The emulator's state
  lines show the position instead.
- **G-code files.** Drop `.gcode` files on the box, or click it to choose.
  Each file in the list has Print and Delete buttons. Files also appear on
  the touchscreen's Print screen. They are kept in `ccos/sim/docker/gcodes/`
  on your PC, so you can copy files there directly too.

A full 11×11 bed mesh takes about 6 minutes of probing; then look at
Control → Bed mesh.

**Power cut** (top bar) cuts the mains. During a print, ccos saves where it
was and stops. **Power on** boots it again, and the screen asks whether to
resume the print. The switch is System → "Resume after power loss".

## Watching a test script

```sh
docker compose -f ccos/sim/docker/compose.yaml run --rm --service-ports ccsim sim/scripts/10_bed_mesh.ccsim
```

Open <http://localhost:6080> while it runs. Screenshots the script takes land
in `ccos/sim/docker/out/`. The container stops when the script ends.

## Other options

Arguments after the service name go to ccsim, e.g. `--speed 10` (heat 10×
faster) or `--bootloader`; `ccsim --help` lists them all.
