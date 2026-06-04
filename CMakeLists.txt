cmake_minimum_required(VERSION 3.16)
project(DSCA-NG
    VERSION 2.0.0
    DESCRIPTION "Digital SCA Next Generation — OFDM Digital Radio Modem"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
elseif(MSVC)
    # /W4 with /WX = warnings-as-errors. Selective GLOBAL suppressions only
    # for warnings that fire in Qt-MOC-generated or template code and carry
    # no signal:
    #   C4127 — conditional expression is constant (Qt MOC + if-constexpr)
    #   C4458 — declaration hides class member (Qt MOC-generated code)
    #   C4505 — unreferenced local function (template instantiations)
    # C4244 (int→smaller) and C4267 (size_t→smaller) are NOT suppressed
    # globally anymore — those are the exact narrowing classes we want
    # flagged in the DSP core. They are scoped PRIVATE to the Qt GUI target
    # (dsca_gui) below, where Qt's int-sized APIs make them unavoidable
    # noise. (#25)
    add_compile_options(/W4 /WX /permissive- /wd4127 /wd4458 /wd4505)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -D_USE_MATH_DEFINES)
endif()

# Version info for about dialog
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M" UTC)
add_compile_definitions(
    DSCA_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    DSCA_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    DSCA_VERSION_PATCH=${PROJECT_VERSION_PATCH}
    DSCA_BUILD_DATE="${BUILD_TIMESTAMP}"
)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets)

# Optional FFTW3 backend — falls back to built-in radix-2 if not enabled.
# To enable: -DUSE_FFTW3=ON  (requires FFTW3F installed via vcpkg or system)
option(USE_FFTW3 "Use FFTW3 single-precision instead of built-in FFT" OFF)
if(USE_FFTW3)
    find_package(FFTW3f CONFIG QUIET)
    if(NOT FFTW3f_FOUND)
        find_library(FFTW3F_LIB NAMES fftw3f)
        find_path(FFTW3F_INC fftw3.h)
        if(FFTW3F_LIB AND FFTW3F_INC)
            add_library(FFTW3::fftw3f UNKNOWN IMPORTED)
            set_target_properties(FFTW3::fftw3f PROPERTIES
                IMPORTED_LOCATION "${FFTW3F_LIB}"
                INTERFACE_INCLUDE_DIRECTORIES "${FFTW3F_INC}")
        else()
            message(FATAL_ERROR "USE_FFTW3 requested but FFTW3f not found")
        endif()
    endif()
    add_compile_definitions(DSCA_USE_FFTW3)
    message(STATUS "FFT backend: FFTW3 single-precision")
else()
    message(STATUS "FFT backend: built-in radix-2")
endif()

# SIMD switch — enables AVX2 / NEON in hot loops (currently informational;
# implementations live in dsp/* internals once added).
option(ENABLE_SIMD "Enable AVX2 / NEON intrinsics in DSP hot loops" ON)
if(ENABLE_SIMD)
    add_compile_definitions(DSCA_ENABLE_SIMD)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
            add_compile_options(-mavx2 -mfma)
        endif()
    elseif(MSVC)
        add_compile_options(/arch:AVX2)
    endif()
endif()

# Find Opus: try CMake config first (vcpkg/Windows), fall back to pkg-config (Linux)
find_package(Opus CONFIG QUIET)
if(Opus_FOUND)
    # vcpkg / CMake config provides Opus::opus target
    set(OPUS_TARGET Opus::opus)
    message(STATUS "Found Opus via CMake config")
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OPUS REQUIRED opus)
    set(OPUS_TARGET "")  # Will use OPUS_LIBRARIES / OPUS_INCLUDE_DIRS instead
    message(STATUS "Found Opus via pkg-config: ${OPUS_LIBRARIES}")
endif()

# --- Core DSP (no Qt) ---
add_library(dsca_core STATIC
    src/fft_engine.cpp
    src/symbol_mapper.cpp
    src/ofdm.cpp
    src/frame.cpp
    src/ldpc.cpp
    src/interleaver.cpp
    src/opus_codec.cpp
    src/iq_converter.cpp
    src/soundcard_modem.cpp
    src/spectrum_analyzer.cpp
    src/orbgrand.cpp
    src/mmse_estimator.cpp
    src/hw_audio.cpp
    src/papr_reducer.cpp
    src/hierarchical_mod.cpp
    src/pls.cpp
    src/reed_solomon.cpp
    src/side_reconstructor.cpp
)
target_include_directories(dsca_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OPUS_INCLUDE_DIRS}
)
find_package(Threads REQUIRED)
if(OPUS_TARGET)
    target_link_libraries(dsca_core PUBLIC ${OPUS_TARGET} Threads::Threads)
else()
    target_link_libraries(dsca_core PUBLIC ${OPUS_LIBRARIES} Threads::Threads)
endif()
if(USE_FFTW3)
    target_link_libraries(dsca_core PUBLIC FFTW3::fftw3f)
endif()

# Hardware audio support (miniaudio: WASAPI on Windows, ALSA/PulseAudio on Linux,
# CoreAudio on macOS). Default ON now that virtual-cable loopback testing is the
# primary integration scenario.
option(DSCA_ENABLE_AUDIO "Enable hardware audio via miniaudio" ON)
if(DSCA_ENABLE_AUDIO)
    target_compile_definitions(dsca_core PUBLIC DSCA_ENABLE_AUDIO)
    if(UNIX AND NOT APPLE)
        target_link_libraries(dsca_core PUBLIC dl m pthread)
    elseif(APPLE)
        find_library(COREAUDIO_LIB CoreAudio)
        find_library(AUDIOUNIT_LIB AudioUnit)
        find_library(COREFOUNDATION_LIB CoreFoundation)
        target_link_libraries(dsca_core PUBLIC
            ${COREAUDIO_LIB} ${AUDIOUNIT_LIB} ${COREFOUNDATION_LIB})
    elseif(WIN32)
        # WASAPI requires ole32, mmdevapi, avrt (for AvSetMmThreadCharacteristics).
        target_link_libraries(dsca_core PUBLIC ole32 winmm)
    endif()
endif()

# --- GUI library ---
add_library(dsca_gui STATIC
    gui/data_bridge.cpp
    gui/audio_engine.cpp
    gui/widgets/spectrum_widget.cpp
    gui/widgets/constellation_widget.cpp
    gui/widgets/meter_widget.cpp
    gui/widgets/scope_widget.cpp
    gui/panels/tx_panel.cpp
    gui/panels/rx_panel.cpp
    gui/panels/alarm_panel.cpp
    gui/panels/info_panel.cpp
    gui/panels/stream_panel.cpp
    gui/panels/link_budget_panel.cpp
    gui/panels/tuning_panel.cpp
    gui/widgets/channel_response_widget.cpp
    gui/widgets/eye_diagram_widget.cpp
    gui/widgets/pls_status_widget.cpp
    gui/widgets/hier_flow_widget.cpp
    gui/dialogs/preset_dialog.cpp
    gui/dialogs/alarm_config_dialog.cpp
    gui/dialogs/device_dialog.cpp
    gui/dialogs/hierarchical_dialog.cpp
    gui/dialogs/help_dialog.cpp
    gui/main_window.cpp
)
target_include_directories(dsca_gui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/gui
)
target_link_libraries(dsca_gui PUBLIC
    dsca_core Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGLWidgets
)
# Qt's APIs take int sizes, so size_t→int (C4267) and int→smaller (C4244)
# narrowings are unavoidable noise in GUI/MOC code. Scope those two
# suppressions to the GUI target only — the DSP core (dsca_core) and CLI
# now compile WITHOUT them so genuine narrowing bugs there are caught
# by /WX. (#25)
if(MSVC)
    target_compile_options(dsca_gui PRIVATE /wd4244 /wd4267)
endif()

# --- Main executable ---
option(BUILD_GUI "Build Qt6 GUI" ON)
if(BUILD_GUI)
    add_executable(dsca_ng main.cpp)
    target_link_libraries(dsca_ng PRIVATE dsca_gui)
    # WIN32_EXECUTABLE TRUE = no attached console (cleaner shipping). Toggle
    # to FALSE during dev to see stderr.
    set_target_properties(dsca_ng PROPERTIES MACOSX_BUNDLE TRUE WIN32_EXECUTABLE TRUE)

    # gui_walker: automated GUI fuzzer that drives every control of the live
    # MainWindow and detects crashes/hangs/Qt-warnings. Console subsystem so
    # its diagnostics are visible. Dev/QA tool — not shipped.
    add_executable(gui_walker tests/gui_walker.cpp)
    target_link_libraries(gui_walker PRIVATE dsca_gui)
    target_include_directories(gui_walker PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# --- CLI tools (dsca_encode, dsca_decode, dsca_modem) ---
option(BUILD_CLI "Build command-line tools" ON)
if(BUILD_CLI)
    add_executable(dsca_encode cli/dsca_encode.cpp)
    target_link_libraries(dsca_encode PRIVATE dsca_core)
    target_include_directories(dsca_encode PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cli
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

    add_executable(dsca_decode cli/dsca_decode.cpp)
    target_link_libraries(dsca_decode PRIVATE dsca_core)
    target_include_directories(dsca_decode PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cli
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

    add_executable(dsca_modem cli/modem_cli.cpp)
    target_link_libraries(dsca_modem PRIVATE dsca_core)
    target_include_directories(dsca_modem PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cli
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

# --- Tests ---
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    foreach(T loopback_test frame_test fec_test codec_test modem_test)
        add_executable(${T} tests/${T}.cpp)
        target_link_libraries(${T} dsca_core)
    endforeach()

    add_executable(gui_test tests/gui_test.cpp)
    target_compile_definitions(gui_test PRIVATE DSCA_HEADLESS)
    target_include_directories(gui_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(gui_test dsca_core)

    # New-module unit tests (Phase 1-5 additions)
    add_executable(dsp_primitives_test tests/dsp_primitives_test.cpp)
    target_link_libraries(dsp_primitives_test dsca_core)

    # End-to-end integration test that drives the full TX→RX engine pipeline
    # in headless mode. This is the test that catches integration bugs the
    # isolated unit tests miss: FEC-change crashes, modcod-rebuild races,
    # tick-latency regressions, threshold-monotonicity bugs.
    add_executable(integration_test tests/integration_test.cpp)
    target_link_libraries(integration_test dsca_core)

    # Direct encoder-validity test: encode random info, verify Hc^T = 0.
    # Catches encoder bugs faster than full TX→RX cycles.
    add_executable(ldpc_encoder_test tests/ldpc_encoder_test.cpp)
    target_link_libraries(ldpc_encoder_test dsca_core)

    # Bisects the OFDM chain stage-by-stage to localize bit-loss bugs.
    add_executable(ofdm_chain_test tests/ofdm_chain_test.cpp)
    target_link_libraries(ofdm_chain_test dsca_core)

    # Reed-Solomon outer-code round-trip + error-correction validation.
    add_executable(reed_solomon_test tests/reed_solomon_test.cpp)
    target_link_libraries(reed_solomon_test dsca_core)

    # Side-channel reconstructor (predictive graceful-degradation).
    add_executable(side_reconstructor_test tests/side_reconstructor_test.cpp)
    target_link_libraries(side_reconstructor_test dsca_core)

    # OFDM CP windowing (Tukey taper) — OOB suppression + clean-channel
    # BER preservation + legacy-path equivalence when disabled.
    add_executable(cp_window_test tests/cp_window_test.cpp)
    target_link_libraries(cp_window_test dsca_core)

    # Asymmetric hierarchical modulation round-trip — bit alignment +
    # wide-layer (>8 bit) index handling.
    add_executable(hierarchical_test tests/hierarchical_test.cpp)
    target_link_libraries(hierarchical_test dsca_core)

    # QC-LDPC protograph (girth-conditioned) vs legacy base matrices:
    # encoder validity, 4-cycle (girth) count, and marginal-SNR AWGN BER
    # A/B at the high rates (#11 / SOTA-1).
    add_executable(ldpc_protograph_test tests/ldpc_protograph_test.cpp)
    target_link_libraries(ldpc_protograph_test dsca_core)

    # Sample-rate-offset (SFO) loop: arbitrary-ratio resampler + the SRO
    # control loop holding lock under an injected clock offset (SOTA-2).
    add_executable(sfo_test tests/sfo_test.cpp)
    target_link_libraries(sfo_test dsca_core)

    # PLS bootstrap robustness: Reed-Muller FEC + soft-combine vs the legacy
    # CRC + repetition, over an AWGN BPSK channel (#26).
    add_executable(pls_test tests/pls_test.cpp)
    target_link_libraries(pls_test dsca_core)

    # DRED decoder-side burst recovery vs PLC (SOTA-5). Skips cleanly if the
    # linked libopus lacks DRED.
    add_executable(dred_test tests/dred_test.cpp)
    target_link_libraries(dred_test dsca_core)

    # Value-sanity sweep of the GUI bitrate/subcarrier readouts: asserts the
    # displayed ComputedParams match the modem's real allocation and stay
    # physically sane across every modcod / FFT / sample-rate / signal-BW combo
    # (guards the size_t-underflow "bitrate through the roof" class of bug).
    add_executable(computed_params_test tests/computed_params_test.cpp)
    target_link_libraries(computed_params_test dsca_core)

    enable_testing()
    foreach(T loopback_test frame_test fec_test codec_test modem_test
              gui_test dsp_primitives_test integration_test ldpc_encoder_test
              reed_solomon_test side_reconstructor_test cp_window_test
              hierarchical_test ldpc_protograph_test sfo_test pls_test dred_test
              computed_params_test)
        add_test(NAME ${T} COMMAND ${T})
    endforeach()
endif()
