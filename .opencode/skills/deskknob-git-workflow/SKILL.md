---
name: deskknob-git-workflow
description: Use for ANY code change in the DeskKnob ESP-IDF project before finishing a task. Enforces committing and pushing every completed change to the esp_idf_v1.0 branch on GitHub, and describes the build/flash/test loop.
---

# DeskKnob git workflow

This project MUST stay in sync with the remote repository. Every time you make
a code change as part of a task, before you report the task finished you MUST:

1. Build (`pio run`).
2. Flash + verify on the device when hardware is attached (see below).
3. `git add` the intended files, `git commit` with a clear message.
4. `git push origin esp_idf_v1.0`.

Do not leave finished work uncommitted. If a push fails, report the failure and
the exact error instead of silently stopping.

## Repository facts

- Remote: `https://github.com/TJiangF/DeskKnob`
- Required branch: **`esp_idf_v1.0`** (never push the ESP-IDF rewrite to `main`).
- Old Arduino firmware is preserved under `legacy_arduino/`.
- Project path: `/Users/tf/Documents/PlatformIO/Projects/Deskknob_ESPIDF`

## Commit rules

- Stage only the files relevant to the change. `managed_components/`,
  `dependencies.lock`, `build/`, `.pio/`, and `sdkconfig*` are gitignored —
  never force-add them.
- Commit messages: short imperative subject, optional body listing what changed
  and any hardware behaviour that was verified.
- One coherent change per commit; prefer several small commits over one giant
  one.

## Push

```sh
git push origin esp_idf_v1.0
```

If authentication fails, tell the user to re-auth `gh auth login` (or push
manually) and do not retry in a loop.

## Build / flash / test loop

Everything runs through PlatformIO with the local ESP-IDF 6.0.1 toolchain:

```sh
export PATH="$HOME/.platformio/penv/bin:$PATH"
pio run                       # build
pio run -t upload --upload-port /dev/cu.wchusbserial*   # flash (port changes!)
```

- The USB-serial port name changes on every re-enumeration
  (`/dev/cu.wchusbserial120`, `...1140`, `...1120`, ...). Always
  auto-detect it: `ls /dev/cu.wchusbserial*`.
- To read logs, open the port at 115200 with a short pyserial script under
  `$HOME/.platformio/penv/bin/python`.
- After flashing, the device may need a physical replug if the serial tool
  reports `No serial data received` or `Invalid argument`.
- When a new source file is added to a component, run `pio run -t clean` once
  so CMake reconfigure picks it up (otherwise link errors / missing headers).

## Project conventions

- Component layout lives in `components/deskknob_*`; app entry is `main/`.
- Never hardcode GPIO numbers outside `components/deskknob_board/include/board_pins.h`.
- UI colors must use the runtime theme variables (`C_BG`, `C_FG`, `C_MUTED`,
  `C_CARD`, `C_ACCENT`, `C_ACCENT2`), never literal `lv_color_black()` /
  `lv_color_white()`, so dark/light themes keep working.
- All LVGL access goes through `display_lock()` / `display_unlock()`.
