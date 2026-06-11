# vcpkg overlay — Opus with DRED / deep PLC

The default vcpkg `opus` port (1.5.2) is built **without** Deep REDundancy
(DRED), so `OPUS_SET_DRED_DURATION` and `opus_dred_*` return
`OPUS_UNIMPLEMENTED` at runtime. That blocks SOTA-5 (DRED decoder-side
recovery).

This overlay differs from the upstream `opus` port in two ways:
  1. a **`dred` feature** that passes `-DOPUS_DRED=ON` to Opus's CMake build
     (which auto-enables deep PLC), and
  2. it sources the **official release tarball** (`opus-1.5.2.tar.gz` from
     downloads.xiph.org) instead of the GitHub source archive. With DRED on,
     the build needs the generated neural-model data files (`dnn/*_data.h`,
     ~2 MB) — these are too large for git and are absent from the GitHub
     archive; only the release tarball bundles them.

## Rebuild Opus with DRED (classic-mode vcpkg)

```powershell
# Run from the repository root.
& C:\vcpkg\vcpkg.exe remove opus:x64-windows
& C:\vcpkg\vcpkg.exe install "opus[core,dred]:x64-windows" `
    --overlay-ports=".\vcpkg-overlay"
```

Requires internet access during the build (for the model download). The new
`opus.dll`/`opus.lib` (~2 MB larger) replace the stubbed ones in
`C:\vcpkg\installed\x64-windows\`; the DSCA-NG build + deploy pick them up
automatically.

Pass `--overlay-ports` on every future `vcpkg install opus` so the DRED build
stays pinned (a plain `vcpkg upgrade` would otherwise revert to the stubbed
upstream port).
