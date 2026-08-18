# TouchGFX preview build

The desktop preview is generated at:

```text
stm32f411_fw/TouchGFX/build/bin/simulator.exe
```

From the repository root, build it with:

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_touchgfx_preview.ps1
```

Build and start the simulator with:

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_touchgfx_preview.ps1 -Run
```

The default command uses the generated sources tracked by Git and only writes
to the ignored `TouchGFX/build/` directory. When the `.touchgfx`, text, font, or
image assets have intentionally changed, regenerate them explicitly:

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_touchgfx_preview.ps1 -RegenerateAssets
```

Requirements:

- TouchGFX Designer 4.26.1 installed under `C:\TouchGFX\4.26.1`
- The generated TouchGFX sources committed to this repository

The `TouchGFX/build/` directory is intentionally ignored by Git. Commit the
source project, generated source, and this reproducible build script instead of
committing simulator binaries or object files.

While the simulator window is focused, press `F3` to save a BMP screenshot in
`stm32f411_fw/TouchGFX/build/bin/screenshots/`, or `Shift+F3` to copy the
screen.
