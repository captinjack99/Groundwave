# DSCA-NG v2 — Windows Build Guide
## Visual Studio 2022 + vcpkg + Qt6

---

## Prerequisites

### 1. Visual Studio 2022

Download from https://visualstudio.microsoft.com/downloads/

During install, select the **"Desktop development with C++"** workload.
This gives you MSVC 19.x, CMake, and the Developer PowerShell.

Verify in Developer PowerShell:
```powershell
cl.exe
# Should print: Microsoft (R) C/C++ Optimizing Compiler Version 19.xx.xxxxx
cmake --version
# Should print: cmake version 3.28+ (bundled with VS)
```

### 2. vcpkg (Package Manager)

```powershell
# Clone vcpkg to a permanent location (e.g., C:\vcpkg)
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# Add to your PATH (or set VCPKG_ROOT)
[Environment]::SetEnvironmentVariable("VCPKG_ROOT", "C:\vcpkg", "User")
[Environment]::SetEnvironmentVariable("Path", "$env:Path;C:\vcpkg", "User")

# Restart your terminal after setting env vars
```

### 3. Install Dependencies via vcpkg

```powershell
cd C:\vcpkg

# Install Opus (fast, ~30 seconds)
.\vcpkg install opus:x64-windows

# Install Qt6 (this takes 20-40 minutes on first build — go get coffee)
.\vcpkg install qt6:x64-windows

# Verify both installed
.\vcpkg list | Select-String "opus|qt6-base"
# Should show:
#   opus:x64-windows       1.4+      ...
#   qt6-base:x64-windows   6.x.x     ...
```

**Alternative: Qt Online Installer** — If you already have Qt6 installed via
the Qt online installer (https://www.qt.io/download-qt-installer-oss), you
can skip the vcpkg Qt6 install. Set `CMAKE_PREFIX_PATH` to your Qt install:
```powershell
$env:CMAKE_PREFIX_PATH = "C:\Qt\6.7.0\msvc2022_64"
```

---

## Build Steps

Open **Developer PowerShell for VS 2022** (from Start menu or VS).

### Extract the project

```powershell
# Navigate to where you extracted the zip
cd C:\Users\YourName\Projects\dsca-ng-v2
```

### Configure

```powershell
# With vcpkg (recommended — finds both Qt6 and Opus automatically)
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake" `
    -DCMAKE_BUILD_TYPE=Release
```

You should see:
```
-- Found Opus via CMake config
-- Found Qt6 ...
-- Configuring done
-- Generating done
```

If you're using the Qt Online Installer instead of vcpkg Qt6:
```powershell
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake" `
    -DCMAKE_PREFIX_PATH="C:\Qt\6.7.0\msvc2022_64" `
    -DCMAKE_BUILD_TYPE=Release
```

### Build

```powershell
cmake --build build --config Release --parallel
```

### Run Tests

```powershell
cd build
ctest -C Release --output-on-failure
```

Expected output:
```
1/6 Test #1: loopback_test ....................   Passed
2/6 Test #2: frame_test .......................   Passed
3/6 Test #3: fec_test .........................   Passed
4/6 Test #4: codec_test .......................   Passed
5/6 Test #5: modem_test .......................   Passed
6/6 Test #6: gui_test .........................   Passed

100% tests passed, 0 tests failed out of 6
```

Individual test detail:
```powershell
.\Release\gui_test.exe
# --- Results: 269/269 passed ---
```

### Run the GUI

```powershell
.\Release\dsca_ng.exe
```

If Qt DLLs aren't found, run `windeployqt`:
```powershell
# Find windeployqt (vcpkg or Qt installer)
C:\vcpkg\installed\x64-windows\tools\Qt6\bin\windeployqt.exe .\Release\dsca_ng.exe
# OR from Qt installer:
C:\Qt\6.7.0\msvc2022_64\bin\windeployqt.exe .\Release\dsca_ng.exe
```

---

## Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_GUI` | `ON` | Build the Qt6 GUI executable |
| `BUILD_TESTS` | `ON` | Build test executables |
| `DSCA_ENABLE_AUDIO` | `OFF` | Enable real soundcard I/O via miniaudio |

Example with hardware audio enabled:
```powershell
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake" `
    -DDSCA_ENABLE_AUDIO=ON
```

---

## Troubleshooting

### "Could not find Qt6"
- Verify: `C:\vcpkg\installed\x64-windows\share\Qt6\Qt6Config.cmake` exists
- Or set `-DCMAKE_PREFIX_PATH=<your Qt6 path>`

### "Could not find Opus"
- Verify: `C:\vcpkg\installed\x64-windows\share\Opus\OpusConfig.cmake` exists
- The CMake output should say "Found Opus via CMake config"

### LNK2019 unresolved externals for Opus
- Make sure you're using `x64-windows` triplet, not `x86-windows`
- Rebuild with `-A x64` in the cmake configure step

### "Qt6::moc not found" or AUTOMOC errors
- Ensure `qt6-base` and `qt6-tools` are installed via vcpkg
- `.\vcpkg install qt6[tools]:x64-windows`

### Missing DLLs at runtime
- Copy DLLs: `windeployqt .\Release\dsca_ng.exe`
- Or add vcpkg bin to PATH: `$env:Path += ";C:\vcpkg\installed\x64-windows\bin"`

### Tests-only build (no GUI, no Qt needed)
```powershell
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake" `
    -DBUILD_GUI=OFF
cmake --build build --config Release --parallel
cd build
.\Release\gui_test.exe
```
Note: This still needs Qt6 for the `dsca_gui` library. For a pure
headless/test build without Qt, run only the core test executables:
```powershell
.\Release\loopback_test.exe
.\Release\frame_test.exe
.\Release\fec_test.exe
.\Release\codec_test.exe
.\Release\modem_test.exe
.\Release\gui_test.exe
```

---

## Quick Copy-Paste (full sequence)

```powershell
# One-time setup (skip if already done)
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg install opus:x64-windows qt6:x64-windows

# Build DSCA-NG
cd C:\Users\YourName\Projects\dsca-ng-v2
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake"
cmake --build build --config Release --parallel
cd build
ctest -C Release --output-on-failure
.\Release\gui_test.exe
.\Release\dsca_ng.exe
```
