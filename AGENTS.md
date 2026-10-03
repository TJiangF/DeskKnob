# AGENTS.md — DeskKnob (ESP-IDF)

## Always commit AND push

Every completed change in this project must be committed and pushed to the
**`esp_idf_v1.0`** branch of `https://github.com/TJiangF/DeskKnob` before you
report the task as done:

```sh
git add <intended files>
git commit -m "<type>: <summary>"
git push origin esp_idf_v1.0
```

- Never push this ESP-IDF work to `main`.
- Do not commit `managed_components/`, `dependencies.lock`, `build/`, `.pio/`,
  or `sdkconfig*` (they are gitignored).
- If the push fails, report the error; ask the user to `gh auth login` if it is
  an authentication problem.

## Build / flash

```sh
export PATH="$HOME/.platformio/penv/bin:$PATH"
pio run
pio run -t upload --upload-port "$(ls /dev/cu.wchusbserial* | head -1)"
```

- The serial port name changes on every re-enumeration — always auto-detect it.
- After adding a new source file to a component, run `pio run -t clean` once.
- Monitor logs at 115200 baud.

## Conventions

- App entry: `main/`. Components: `components/deskknob_*`.
- GPIO numbers only in `components/deskknob_board/include/board_pins.h`.
- UI colors only via theme vars (`C_BG`, `C_FG`, `C_MUTED`, `C_CARD`,
  `C_ACCENT`, `C_ACCENT2`) so dark/light themes work.
- Wrap all LVGL calls with `display_lock()` / `display_unlock()`.

See also `.opencode/skills/deskknob-git-workflow/SKILL.md`.
