# DSCA-NG

A software-defined **OFDM digital radio modem**. It carries multi-stream
Opus audio over a complex-baseband OFDM waveform with QC-LDPC forward error
correction, and ships with a Qt6 desktop application for live operation and
diagnostics plus a set of command-line tools. It runs entirely in software
loopback (no hardware required) and can also drive a real soundcard.

<!-- Replace OWNER/REPO after the first push to enable the badge. -->
![build](https://github.com/OWNER/REPO/actions/workflows/build.yml/badge.svg)

## Highlights

- **Complex-baseband OFDM** physical layer with configurable FFT size
  (64–16384), cyclic prefix, pilot spacing, and frequency-domain preamble
  for correct-by-construction channel estimation.
- **Modulation** from BPSK through QAM-4096, plus **hierarchical
  modulation** (HP/LP layers) for graceful-degradation stereo via a
  Mid/Side split.
- **FEC**: girth-conditioned **QC-LDPC** (protograph construction, 4-cycle
  free) over 11 code rates, decoded by **ORBGRAND**, layered min-sum BP, or
  iterative **BICM-ID**, with an optional **Reed-Solomon** outer code.
- **Audio**: per-stream **Opus** encode/decode (up to 8 streams), with
  optional **DRED** (Opus 1.5 Deep REDundancy) neural packet-loss recovery
  and inband-FEC fallback.
- **Adaptive link**: VCM/AMC ModCod scheduling, in-band PLS signalling,
  AFC/AGC, and sample-rate-offset (SFO) correction for clock drift.
- **Qt6 GUI**: live spectrum + waterfall, constellation, eye diagram,
  time-domain scope, channel response, link budget, per-stream mixer,
  alarms, and a preset system with JSON config persistence.
- **CLI tools**: `dsca_encode`, `dsca_decode`, and `dsca_modem` for
  headless / scripted use.

See [`ARCHITECTURE.md`](ARCHITECTURE.md) and
[`docs/ARCHITECTURE_MS_HIER.md`](docs/ARCHITECTURE_MS_HIER.md) for the design.

## Repository layout

| Path | Contents |
|------|----------|
| `include/`, `src/` | Core DSP library (`dsca_core`): OFDM, LDPC, framing, codecs, modem |
| `gui/`, `main.cpp` | Qt6 application (`dsca_ng`) |
| `cli/` | Command-line tools |
| `tests/` | Unit + integration tests (run via `ctest`) and the GUI walker |
| `vcpkg-overlay/` | Overlay port to build libopus with DRED enabled |
| `external/` | Vendored single-header dependencies (miniaudio) |

## Building

Requires **CMake ≥ 3.16**, a **C++17** compiler, **Qt 6**, and **libopus**.

### Windows (MSVC + vcpkg)

```powershell
vcpkg install opus:x64-windows
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
  -DBUILD_GUI=ON -DBUILD_TESTS=ON -DBUILD_CLI=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

See [`BUILD_WINDOWS.md`](BUILD_WINDOWS.md) for the detailed Windows setup
(Qt install, deployment, troubleshooting).

### Linux

```bash
sudo apt install build-essential cmake ninja-build libopus-dev pkg-config \
  qt6-base-dev qt6-base-private-dev libgl1-mesa-dev
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_GUI=ON -DBUILD_TESTS=ON -DBUILD_CLI=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

### macOS

```bash
brew install opus qt@6 cmake ninja
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)" \
  -DBUILD_GUI=ON -DBUILD_TESTS=ON -DBUILD_CLI=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

### Optional: DRED-enabled libopus

DRED neural packet-loss recovery requires a libopus built with deep-PLC.
The stock libopus builds fine — DRED is simply inert and the modem falls
back to inband FEC. To enable it, build opus from the overlay port:

```
vcpkg install "opus[core,dred]:x64-windows" --overlay-ports=vcpkg-overlay
```

See [`vcpkg-overlay/README.md`](vcpkg-overlay/README.md).

## Running

- **GUI**: launch `dsca_ng`. Pick a preset (F2–F8), press **TX**, and the
  built-in software loopback runs the full TX→RX chain; the diagnostics
  panels show the live constellation, spectrum, eye, and sync state.
- **CLI**: `dsca_encode` / `dsca_decode` move audio ↔ modem frames;
  `dsca_modem` runs a loopback or file-based modem session. Run any tool
  with `--help` for options.

## Tests

`ctest` runs the full suite (DSP primitives, OFDM chain, LDPC, Reed-Solomon,
PLS, SFO, DRED, hierarchical mod, and an end-to-end integration test that
drives the complete TX→RX engine). The GUI walker
([`tests/gui_walker.cpp`](tests/gui_walker.cpp)) is a separate manual QA
harness that exercises every GUI control with the engine running.

## License

See [`LICENSE`](LICENSE).
