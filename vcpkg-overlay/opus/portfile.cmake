# Use the official RELEASE tarball, not the GitHub source archive: with
# OPUS_DRED=ON the CMake build needs the generated neural-model data files
# (dnn/*_data.h) which are too large for git and so are ABSENT from the
# GitHub archive — only the release tarball bundles them. (DSCA-NG SOTA-5)
vcpkg_download_distfile(ARCHIVE
    URLS
        "https://downloads.xiph.org/releases/opus/opus-${VERSION}.tar.gz"
        "https://ftp.osuosl.org/pub/xiph/releases/opus/opus-${VERSION}.tar.gz"
    FILENAME "opus-${VERSION}.tar.gz"
    SHA512 78d963cd56d5504611f111e2b3606e236189a3585d65fae1ecdbec9bf4545632b1956f11824328279a2d1ea2ecf441ebc11e455fb598d20a458df15185e95da4
)

vcpkg_extract_source_archive(SOURCE_PATH
    ARCHIVE "${ARCHIVE}"
    PATCHES fix-pkgconfig-version.patch
)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        avx2 AVX2_SUPPORTED
        dred OPUS_DRED          # DSCA-NG SOTA-5: enable DRED (auto-enables deep PLC)
)

set(STACK_PROTECTOR ON)
set(ADDITIONAL_OPUS_OPTIONS "")
if(VCPKG_TARGET_IS_MINGW)
    set(STACK_PROTECTOR OFF)
    string(APPEND VCPKG_C_FLAGS "-D_FORTIFY_SOURCE=0")
    string(APPEND VCPKG_CXX_FLAGS "-D_FORTIFY_SOURCE=0")
    if(VCPKG_TARGET_ARCHITECTURE MATCHES "^(ARM|arm)64$")
        list(APPEND ADDITIONAL_OPUS_OPTIONS "-DOPUS_USE_NEON=OFF") # for version 1.3.1 (remove for future Opus release)
        list(APPEND ADDITIONAL_OPUS_OPTIONS "-DOPUS_DISABLE_INTRINSICS=ON") # for HEAD (and future Opus release)
    endif()
elseif(VCPKG_TARGET_IS_WINDOWS)
    if(VCPKG_CRT_LINKAGE STREQUAL "static")
        list(APPEND ADDITIONAL_OPUS_OPTIONS "-DOPUS_STATIC_RUNTIME=ON")
    endif()
elseif(VCPKG_TARGET_IS_EMSCRIPTEN)
    set(STACK_PROTECTOR OFF)
endif()

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS ${FEATURE_OPTIONS}
        -DPACKAGE_VERSION=${VERSION}
        -DOPUS_STACK_PROTECTOR=${STACK_PROTECTOR}
        -DOPUS_INSTALL_PKG_CONFIG_MODULE=ON
        -DOPUS_INSTALL_CMAKE_CONFIG_MODULE=ON
        -DOPUS_BUILD_PROGRAMS=OFF
        -DOPUS_BUILD_TESTING=OFF
        ${ADDITIONAL_OPUS_OPTIONS}
    MAYBE_UNUSED_VARIABLES
        OPUS_USE_NEON
        OPUS_DISABLE_INTRINSICS
)
vcpkg_cmake_install()
vcpkg_copy_pdbs()

vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/Opus)
vcpkg_fixup_pkgconfig(SYSTEM_LIBRARIES m)


file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/lib/cmake"
                    "${CURRENT_PACKAGES_DIR}/lib/cmake"
                    "${CURRENT_PACKAGES_DIR}/debug/include")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/COPYING")
