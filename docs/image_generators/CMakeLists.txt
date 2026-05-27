# Doc image generators for rayrai.
#
# Each binary renders a small headless scene and writes one or more PNGs into
# ${RAISIM_DOC_IMAGE_OUTPUT_DIR} (default: ${CMAKE_CURRENT_SOURCE_DIR}/../image/rayrai).
# The docs/CMakeLists.txt wires a custom target that runs every generator
# before sphinx-build, so a fresh doc build always uses up-to-date images.

cmake_minimum_required(VERSION 3.18)

if(NOT TARGET rayrai)
  find_package(rayrai CONFIG QUIET)
endif()
if(NOT TARGET rayrai)
  message(STATUS "rayrai not available; skipping doc image generators")
  return()
endif()

if(NOT TARGET raisim::raisim)
  find_package(raisim CONFIG QUIET)
endif()
if(NOT TARGET raisim::raisim)
  message(STATUS "raisim not available; skipping doc image generators")
  return()
endif()

find_package(SDL2 QUIET)
if(NOT TARGET SDL2::SDL2)
  message(STATUS "SDL2 not available; skipping doc image generators")
  return()
endif()

set(RAISIM_DOC_IMAGE_OUTPUT_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../image/rayrai"
    CACHE PATH "Directory where doc image generators write PNGs")
file(MAKE_DIRECTORY "${RAISIM_DOC_IMAGE_OUTPUT_DIR}")

# stb_image_write.h ships in the rayrai install tree under
# <rayrai_prefix>/include/stb. The rayrai CONFIG target already exposes that
# path on its INTERFACE_INCLUDE_DIRECTORIES, so we do not need to add it
# explicitly.

set(_doc_image_generators
    doc_image_minimal_usage
    doc_image_custom_visuals
    doc_image_quality_presets
    doc_image_tone_mapping
    doc_image_color_grading
    doc_image_weather_presets
    doc_image_material_factories
    doc_image_ssr_ssao
    doc_image_cinematic
    doc_image_volumetric_fog
    doc_image_wet_snow
    doc_image_lights
    doc_image_rolling_friction)

set(_doc_image_outputs "")
set(_doc_image_targets "")

# Collect the rayrai and raisim install lib directories so we can put them on
# LD_LIBRARY_PATH / DYLD_LIBRARY_PATH / PATH when invoking the generator
# binaries. RUNPATH on the executables covers direct dependencies, but
# transitive loads (e.g. libassimp from librayrai) still need a fallback.
#
# RAYRAI_OVERRIDE_LIB_DIR lets a developer point doc-image generation at a
# freshly-built librayrai.so before the installed package has been updated.
# When set, that directory goes to the front of the runtime search path so
# its rayrai library wins over the installed one.
set(RAYRAI_OVERRIDE_LIB_DIR "" CACHE PATH
    "Optional path to a librayrai.so that overrides the installed copy when running doc image generators")

set(_doc_image_lib_dirs "")
if(RAYRAI_OVERRIDE_LIB_DIR AND EXISTS "${RAYRAI_OVERRIDE_LIB_DIR}")
  list(APPEND _doc_image_lib_dirs "${RAYRAI_OVERRIDE_LIB_DIR}")
endif()
foreach(_dep_target rayrai raisim::raisim)
  if(TARGET ${_dep_target})
    foreach(_prop IMPORTED_LOCATION IMPORTED_LOCATION_RELEASE
            IMPORTED_LOCATION_DEBUG IMPORTED_LOCATION_NOCONFIG
            IMPORTED_LOCATION_RELWITHDEBINFO)
      get_target_property(_loc ${_dep_target} ${_prop})
      if(_loc AND EXISTS "${_loc}")
        get_filename_component(_loc_dir "${_loc}" DIRECTORY)
        list(APPEND _doc_image_lib_dirs "${_loc_dir}")
      endif()
    endforeach()
  endif()
endforeach()
list(REMOVE_DUPLICATES _doc_image_lib_dirs)

if(WIN32)
  set(_doc_image_runtime_env_var "PATH")
  set(_doc_image_path_sep ";")
elseif(APPLE)
  set(_doc_image_runtime_env_var "DYLD_LIBRARY_PATH")
  set(_doc_image_path_sep ":")
else()
  set(_doc_image_runtime_env_var "LD_LIBRARY_PATH")
  set(_doc_image_path_sep ":")
endif()
string(REPLACE ";" "${_doc_image_path_sep}" _doc_image_lib_dirs_str "${_doc_image_lib_dirs}")

add_executable(doc_image_shader_warmup
    "${CMAKE_CURRENT_SOURCE_DIR}/doc_image_shader_warmup.cpp")
target_link_libraries(doc_image_shader_warmup PRIVATE rayrai raisim::raisim SDL2::SDL2)
target_compile_features(doc_image_shader_warmup PRIVATE cxx_std_17)
if(MSVC)
  target_compile_definitions(doc_image_shader_warmup PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

set(_doc_image_shader_warmup_stamp
    "${CMAKE_CURRENT_BINARY_DIR}/doc_image_shader_warmup.stamp")
set(_doc_image_shader_warmup_env
    "${_doc_image_runtime_env_var}=${_doc_image_lib_dirs_str}$<$<BOOL:$ENV{${_doc_image_runtime_env_var}}>:${_doc_image_path_sep}$ENV{${_doc_image_runtime_env_var}}>")
add_custom_command(
    OUTPUT "${_doc_image_shader_warmup_stamp}"
    COMMAND ${CMAKE_COMMAND} -E env
            ${_doc_image_shader_warmup_env}
            $<TARGET_FILE:doc_image_shader_warmup>
    COMMAND ${CMAKE_COMMAND} -E touch "${_doc_image_shader_warmup_stamp}"
    DEPENDS doc_image_shader_warmup
            "${CMAKE_CURRENT_SOURCE_DIR}/doc_image_shader_warmup.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/doc_image_common.hpp"
    WORKING_DIRECTORY "${RAISIM_DOC_IMAGE_OUTPUT_DIR}"
    COMMENT "Prewarming rayrai shader cache for doc image generators"
    VERBATIM)

set(_rolling_frames_dir "${CMAKE_CURRENT_BINARY_DIR}/rolling_friction_frames")
set(_previous_stamp "${_doc_image_shader_warmup_stamp}")
list(APPEND _doc_image_outputs "${_doc_image_shader_warmup_stamp}")
foreach(_gen IN LISTS _doc_image_generators)
  add_executable(${_gen} "${CMAKE_CURRENT_SOURCE_DIR}/${_gen}.cpp")
  target_link_libraries(${_gen} PRIVATE rayrai raisim::raisim SDL2::SDL2)
  target_compile_features(${_gen} PRIVATE cxx_std_17)
  target_compile_definitions(${_gen} PRIVATE
      DOC_IMAGE_OUTPUT_DIR="${RAISIM_DOC_IMAGE_OUTPUT_DIR}")
  if(MSVC)
    target_compile_definitions(${_gen} PRIVATE _CRT_SECURE_NO_WARNINGS)
  endif()

  # Sentinel stamp file so CMake only re-runs the generator when its source
  # changes. The generator itself writes the PNGs; we touch the stamp on
  # success. Stamps are chained so that the binaries run one at a time even
  # under `make -jN` — concurrent SDL/GL initialization on the same display
  # is racy.
  set(_stamp "${CMAKE_CURRENT_BINARY_DIR}/${_gen}.stamp")
  set(_doc_image_generator_env
      "${_doc_image_runtime_env_var}=${_doc_image_lib_dirs_str}$<$<BOOL:$ENV{${_doc_image_runtime_env_var}}>:${_doc_image_path_sep}$ENV{${_doc_image_runtime_env_var}}>")
  set(_doc_image_prepare_command "")
  if(_gen STREQUAL "doc_image_rolling_friction")
    list(APPEND _doc_image_generator_env
        "RAISIM_DOC_ROLLING_FRICTION_FRAMES_DIR=${_rolling_frames_dir}")
    set(_doc_image_prepare_command
        COMMAND ${CMAKE_COMMAND} -E rm -rf "${_rolling_frames_dir}")
  endif()
  add_custom_command(
      OUTPUT "${_stamp}"
      ${_doc_image_prepare_command}
      COMMAND ${CMAKE_COMMAND} -E env
              ${_doc_image_generator_env}
              $<TARGET_FILE:${_gen}> "${RAISIM_DOC_IMAGE_OUTPUT_DIR}"
      COMMAND ${CMAKE_COMMAND} -E touch "${_stamp}"
      DEPENDS ${_gen} "${CMAKE_CURRENT_SOURCE_DIR}/${_gen}.cpp"
              "${CMAKE_CURRENT_SOURCE_DIR}/doc_image_common.hpp"
              ${_previous_stamp}
      WORKING_DIRECTORY "${RAISIM_DOC_IMAGE_OUTPUT_DIR}"
      COMMENT "Generating rayrai doc images: ${_gen}"
      VERBATIM)
  list(APPEND _doc_image_outputs "${_stamp}")
  list(APPEND _doc_image_targets "${_gen}")
  set(_previous_stamp "${_stamp}")
endforeach()

# Composite the rolling-friction frame sequence into a GIF + MP4 if ffmpeg
# is available. The frame sequence is produced by doc_image_rolling_friction;
# this rule depends on that generator's stamp so it always runs after the
# frames are refreshed.
find_program(FFMPEG_EXECUTABLE ffmpeg)
set(_rolling_gif "${RAISIM_DOC_IMAGE_OUTPUT_DIR}/rayrai_rolling_spinning_friction.gif")
if(FFMPEG_EXECUTABLE)
  set(_rolling_stamp "${CMAKE_CURRENT_BINARY_DIR}/doc_image_rolling_friction.stamp")
  add_custom_command(
      OUTPUT "${_rolling_gif}"
      # Generate an indexed-palette GIF so the file is small.
      COMMAND ${FFMPEG_EXECUTABLE} -y -loglevel error
              -framerate 25
              -i "${_rolling_frames_dir}/frame_%03d.png"
              -filter_complex "[0:v] palettegen=stats_mode=diff [p]; [0:v][p] paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle"
              -loop 0
              "${_rolling_gif}"
      DEPENDS "${_rolling_stamp}"
      WORKING_DIRECTORY "${RAISIM_DOC_IMAGE_OUTPUT_DIR}"
      COMMENT "Compositing rolling-friction frames into GIF"
      VERBATIM)
  list(APPEND _doc_image_outputs "${_rolling_gif}")
else()
  message(STATUS "ffmpeg not found; skipping rolling-friction GIF compositing")
endif()

add_custom_target(rayrai_doc_images
    DEPENDS ${_doc_image_outputs}
    COMMENT "All rayrai doc images regenerated")

# Re-export the list so the parent docs CMakeLists can attach Sphinx to it.
set(RAYRAI_DOC_IMAGE_TARGET rayrai_doc_images PARENT_SCOPE)
set(RAYRAI_DOC_IMAGE_OUTPUTS ${_doc_image_outputs} PARENT_SCOPE)
