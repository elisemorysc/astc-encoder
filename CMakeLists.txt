cmake_minimum_required(VERSION 3.0)

include(cmake/HunterGate.cmake)
HunterGate(
    URL "https://github.com/cpp-pm/hunter/archive/v0.23.275.tar.gz"
    SHA1 "0952a54dc983ecbea09dfe8c53ee65d1e0e91232"
)

project(astc-encoder VERSION 1.3.1)

hunter_add_package(stb)
find_package(stb CONFIG REQUIRED)

# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)

option(BUILD_SHARED_LIBS "Build shared libraries" FALSE)

if(MSVC AND BUILD_SHARED_LIBS)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

set(ASTC_ENCODER_SRC_LIST
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_averages_and_directions.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_block_sizes2.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_color_quantize.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_color_unquantize.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_compress_symbolic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_decompress_symbolic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_encoding_choice_error.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_find_best_partitioning.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_ideal_endpoints_and_weights.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_image_load_store.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_integer_sequence.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_ktx_dds.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_kmeans_partitioning.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_partition_tables.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_percentile_tables.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_pick_best_endpoint_format.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_quantization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_stb_tga.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_symbolic_physical.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_weight_align.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_weight_quant_xfer_tables.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_compute_variance.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_extras.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/mathlib.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/softfloat.cpp
)

set(ASTC_ENCODER_PRIVATE_HDR_LIST
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/astc_codec_internals.h
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/mathlib.h
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/softfloat.h
)

add_library(astcenc ${ASTC_ENCODER_SRC_LIST} ${ASTC_ENCODER_PRIVATE_HDR_LIST})
set(TARGETS astcenc)

target_link_libraries(astcenc PUBLIC stb::stb)

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")

set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(NAMESPACE "${PROJECT_NAME}::")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${VERSION_CONFIG}" COMPATIBILITY ExactVersion
)

configure_package_config_file(
    "cmake/Config.cmake.in"
    "${PROJECT_CONFIG}"
    INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
)

install(
    TARGETS ${TARGETS}
    EXPORT "${TARGETS_EXPORT_NAME}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)

install(
    FILES ${ASTC_ENCODER_PUBLIC_HDR_LIST}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(
    FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
    DESTINATION "${CONFIG_INSTALL_DIR}"
)

install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${NAMESPACE}"
    DESTINATION "${CONFIG_INSTALL_DIR}"
)