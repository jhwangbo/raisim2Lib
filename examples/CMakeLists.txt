cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

#==
# Dependencies
#==

if(RAISIM_PREFIX)
    list(APPEND CMAKE_PREFIX_PATH "${RAISIM_PREFIX}")
endif()
if(RAYRAI_PREFIX)
    list(APPEND CMAKE_PREFIX_PATH "${RAYRAI_PREFIX}")
endif()

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(rayrai CONFIG REQUIRED)

if(TARGET glm::glm AND NOT TARGET glm)
    add_library(glm ALIAS glm::glm)
endif()

if(WIN32)
    # Keep example executables inside the build tree (not under the prebuilt package folders).
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION "${CMAKE_BINARY_DIR}/bin")

    function(copy_runtime_dlls_from _source_dir)
        if(EXISTS "${_source_dir}")
            file(GLOB _runtime_dlls "${_source_dir}/*.dll")
            if(_runtime_dlls)
                file(COPY ${_runtime_dlls} DESTINATION "${CMAKE_BINARY_DIR}/bin")
            endif()
        endif()
    endfunction()

    # Make examples runnable without sourcing env scripts by copying runtime DLLs next to executables.
    if(DEFINED RAISIM_PREFIX)
        copy_runtime_dlls_from("${RAISIM_PREFIX}")
        copy_runtime_dlls_from("${RAISIM_PREFIX}/bin")
        copy_runtime_dlls_from("${RAISIM_PREFIX}/lib")
    endif()
    if(DEFINED RAYRAI_PREFIX AND NOT RAYRAI_PREFIX STREQUAL "")
        copy_runtime_dlls_from("${RAYRAI_PREFIX}")
        copy_runtime_dlls_from("${RAYRAI_PREFIX}/bin")
        copy_runtime_dlls_from("${RAYRAI_PREFIX}/lib")
    endif()

    set(_vcpkg_root "$ENV{VCPKG_ROOT}")
    if(_vcpkg_root STREQUAL "" AND EXISTS "C:/vcpkg")
        set(_vcpkg_root "C:/vcpkg")
    endif()
    if(NOT _vcpkg_root STREQUAL "")
        copy_runtime_dlls_from("${_vcpkg_root}/installed/x64-windows/bin")
        copy_runtime_dlls_from("${_vcpkg_root}/installed/x64-windows/debug/bin")
    endif()
else()
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_BINARY_DIR}/examples)
endif()

include_directories ( ${EIGEN3_INCLUDE_DIRS} )

#==
# Build Targets
#==

function(create_executable app_name file_name)
    add_executable(${app_name} ${file_name})
    set_target_properties(${app_name} PROPERTIES MACOSX_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/mac/lib")
    if(WIN32)
        target_link_libraries(${app_name} PUBLIC raisim::raisim Ws2_32 Winmm)
        target_compile_definitions(${app_name} PRIVATE EIGEN_HAS_STD_RESULT_OF=0)
        target_compile_options(${app_name} PRIVATE "/MP")
    else()
        target_link_libraries(${app_name} PUBLIC raisim::raisim pthread)
        target_compile_options(${app_name} PRIVATE "$<$<CONFIG:RELEASE>:-O3>")
        target_compile_options(${app_name} PRIVATE "$<$<CONFIG:DEBUG>:-O0>")
        if(EXAMPLES_RPATH)
            set_target_properties(${app_name} PROPERTIES
                BUILD_RPATH "${EXAMPLES_RPATH}"
            )
        endif()
    endif()

    if(APPLE)
        execute_process(COMMAND sysctl -q hw.optional.arm64
                OUTPUT_VARIABLE _sysctl_stdout
                ERROR_VARIABLE _sysctl_stderr
                RESULT_VARIABLE _sysctl_result
                )
        if(_sysctl_result EQUAL 0 AND _sysctl_stdout MATCHES "hw.optional.arm64: 1")
            target_compile_options(${app_name} PRIVATE -mcpu=apple-m1)
        endif()
    endif()

    target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endfunction()

function(create_rayrai_executable app_name file_name)
    create_executable(${app_name} ${file_name})
    target_link_libraries(${app_name} PRIVATE rayrai)
endfunction()

function(create_executable_if_exists app_name file_name)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${file_name}")
        create_executable(${app_name} ${file_name})
    else()
        message(STATUS "Skipping ${app_name} example: ${file_name} is not present")
    endif()
endfunction()


function(rayrai_header_has header_path needle out_var)
    set(${out_var} FALSE PARENT_SCOPE)
    set(_rayrai_include_candidates "")
    if(DEFINED RAYRAI_PREFIX)
        list(APPEND _rayrai_include_candidates
            "${RAYRAI_PREFIX}/include"
            "${RAYRAI_PREFIX}/linux/include"
            "${RAYRAI_PREFIX}/mac/include"
            "${RAYRAI_PREFIX}/win32/include")
    endif()
    if(TARGET rayrai)
        get_target_property(_rayrai_target_includes rayrai INTERFACE_INCLUDE_DIRECTORIES)
        if(_rayrai_target_includes)
            list(APPEND _rayrai_include_candidates ${_rayrai_target_includes})
        endif()
    endif()
    foreach(_include_dir IN LISTS _rayrai_include_candidates)
        if(EXISTS "${_include_dir}/${header_path}")
            file(READ "${_include_dir}/${header_path}" _rayrai_header_contents)
            if(_rayrai_header_contents MATCHES "${needle}")
                set(${out_var} TRUE PARENT_SCOPE)
                return()
            endif()
        endif()
    endforeach()
endfunction()

function(raisim_header_has header_path needle out_var)
    set(${out_var} FALSE PARENT_SCOPE)
    set(_raisim_include_candidates "")
    if(DEFINED RAISIM_PREFIX)
        list(APPEND _raisim_include_candidates "${RAISIM_PREFIX}/include")
    endif()
    if(TARGET raisim::raisim)
        get_target_property(_raisim_target_includes raisim::raisim INTERFACE_INCLUDE_DIRECTORIES)
        if(_raisim_target_includes)
            list(APPEND _raisim_include_candidates ${_raisim_target_includes})
        endif()
    endif()
    foreach(_include_dir IN LISTS _raisim_include_candidates)
        if(EXISTS "${_include_dir}/${header_path}")
            file(READ "${_include_dir}/${header_path}" _raisim_header_contents)
            if(_raisim_header_contents MATCHES "${needle}")
                set(${out_var} TRUE PARENT_SCOPE)
                return()
            endif()
        endif()
    endforeach()
endfunction()

# server examples
# Basics and object lifecycle.
create_executable(primitive_grid src/server/basics/primitive_grid.cpp)
create_executable(mesh_stack src/server/basics/mesh_stack.cpp)
create_executable(compound_object src/server/basics/compound_object.cpp)
create_executable(dynamic_object_addition src/server/basics/dynamic_object_addition.cpp)
create_executable(ycb_objects src/server/basics/ycb_objects.cpp)

# Asset import/export pipeline. This requires the asset-pipeline APIs added to
# raisim::Mesh and raisim::World, so keep older installed packages buildable.
if(DEFINED RAISIM_PREFIX
   AND EXISTS "${RAISIM_PREFIX}/include/raisim/object/singleBodies/Mesh.hpp"
   AND EXISTS "${RAISIM_PREFIX}/include/raisim/World.hpp")
    file(READ "${RAISIM_PREFIX}/include/raisim/object/singleBodies/Mesh.hpp" _raisim_mesh_header)
    file(READ "${RAISIM_PREFIX}/include/raisim/World.hpp" _raisim_world_header)
    if(_raisim_mesh_header MATCHES "preprocessMesh" AND _raisim_world_header MATCHES "exportMeshAssetsToObj")
        create_executable(model_asset_pipeline src/server/assets/model_asset_pipeline.cpp)
    else()
        message(STATUS "Skipping model_asset_pipeline example: installed raisim package does not expose asset-pipeline APIs")
    endif()
endif()

# Materials and terrain.
create_executable(material_restitution src/server/materials/material_restitution.cpp)
create_executable(material_static_friction src/server/materials/material_static_friction.cpp)
create_executable(procedural_heightmap src/server/terrain/procedural_heightmap.cpp)
create_executable(heightmap_from_png src/server/terrain/heightmap_from_png.cpp)
create_executable(dynamic_heightmap src/server/terrain/dynamic_heightmap.cpp)

# Sensors.
create_executable(ray_casting src/server/sensors/ray_casting.cpp)
create_executable(ray_scan_lidar src/server/sensors/ray_scan_lidar.cpp)

# Robots and control.
create_executable(wheeled_robot_force_control src/server/robots/wheeled_robot_force_control.cpp)
create_executable(templated_tracked_robot src/server/robots/templated_tracked_robot.cpp)
create_executable(minitaur_pd src/server/robots/minitaur_pd.cpp)
create_executable(inverse_dynamics src/server/robots/inverse_dynamics.cpp)
raisim_header_has("raisim/object/granular/GranularSystem.hpp" "GranularSystem" _raisim_has_granular)

# Dynamics and constraints.
create_executable(length_constraints_newtons_cradle src/server/dynamics/length_constraints_newtons_cradle.cpp)
create_executable(spring_damper_joints src/server/dynamics/spring_damper_joints.cpp)
create_executable(kinematic_platform src/server/dynamics/kinematic_platform.cpp)
create_executable(dzhanibekov_effect src/server/dynamics/dzhanibekov_effect.cpp)

# Visualization.
create_executable(visual_objects_showcase src/server/visualization/visual_objects_showcase.cpp)

# Performance and optional deformables.
create_executable(object_lifecycle_stress src/server/performance/object_lifecycle_stress.cpp)
create_executable(island_sleep_benchmark src/server/performance/island_sleep_benchmark.cpp)
if(EXISTS "${RAISIM_PREFIX}/include/raisim/object/deformable/DeformableObject.hpp")
    create_executable(deformable_objects src/server/deformable/deformable_objects.cpp)
else()
    message(STATUS "Skipping deformable_objects example: installed raisim package does not expose DeformableObject")
endif()

# benchmark examples
create_executable(articulated_system_benchmark src/benchmark/articulated_system_benchmark.cpp)
create_executable(anymal_standing_benchmark src/benchmark/anymal_standing_benchmark.cpp)
if(_raisim_has_granular)
    create_executable(granular_media src/benchmark/granular_media.cpp)
else()
    message(STATUS "Skipping granular_media example: installed raisim package does not expose GranularSystem")
endif()

# xml reader
create_executable(xml_world_loader src/xml/xml_world_loader.cpp)
create_executable(xml_templated_world src/xml/xml_templated_world.cpp)
create_executable(mjcf_gymnasium_hopper src/server/mjcf/mjcf_gymnasium_hopper.cpp)
create_executable(mjcf_gymnasium_walker2d src/server/mjcf/mjcf_gymnasium_walker2d.cpp)
create_executable(mjcf_gymnasium_humanoid src/server/mjcf/mjcf_gymnasium_humanoid.cpp)

# World and terrain scenes. Some source distributions omit these heavier examples.
create_executable_if_exists(mountain1_heightmap src/worlds/mountain1_heightmap.cpp)
create_executable_if_exists(office1_scene src/worlds/office1_scene.cpp)
create_executable_if_exists(lake1_heightmap src/worlds/lake1_heightmap.cpp)
create_executable_if_exists(atlas src/worlds/atlas.cpp)
create_executable_if_exists(anymal_pair src/worlds/anymal_pair.cpp)
create_executable_if_exists(hill1_heightmap src/worlds/hill1_heightmap.cpp)
create_executable_if_exists(kinova_arm src/worlds/kinova_arm.cpp)

# rayrai examples
# Entry points.
create_rayrai_executable(rayrai_basic_scene src/rayrai/getting_started/rayrai_basic_scene.cpp)
create_rayrai_executable(rayrai_complete_showcase src/rayrai/getting_started/rayrai_complete_showcase.cpp)

# Sensors.
create_rayrai_executable(rayrai_rgb_camera src/rayrai/sensors/rayrai_rgb_camera.cpp)
create_rayrai_executable(rayrai_depth_camera src/rayrai/sensors/rayrai_depth_camera.cpp)
create_rayrai_executable(rayrai_lidar_pointcloud src/rayrai/sensors/rayrai_lidar_pointcloud.cpp)
create_rayrai_executable(rayrai_aruco_marker src/rayrai/sensors/rayrai_aruco_marker.cpp)

# Visual-only overlays and GPU visual primitives.
create_rayrai_executable(rayrai_custom_visuals src/rayrai/visuals/rayrai_custom_visuals.cpp)
create_rayrai_executable(rayrai_instancing_grid src/rayrai/visuals/rayrai_instancing_grid.cpp)
create_rayrai_executable(rayrai_pointcloud_animation src/rayrai/visuals/rayrai_pointcloud_animation.cpp)

# Materials and assets.
create_rayrai_executable(rayrai_pbr_material_grid src/rayrai/materials/rayrai_pbr_material_grid.cpp)
create_rayrai_executable(rayrai_pbr_texture_maps src/rayrai/materials/rayrai_pbr_texture_maps.cpp)
rayrai_header_has("rayrai/Light.hpp" "SPOT" _rayrai_has_spot_lights)
rayrai_header_has("rayrai/RayraiWindow.hpp" "fxaaEnabled" _rayrai_has_fxaa_quality)
rayrai_header_has("rayrai/RayraiWindow.hpp" "depthOfFieldEnabled" _rayrai_has_quality_dof)
if(_rayrai_has_spot_lights AND _rayrai_has_fxaa_quality AND _rayrai_has_quality_dof)
    create_rayrai_executable(rayrai_quality_lighting src/rayrai/materials/rayrai_quality_lighting.cpp)
else()
    message(STATUS "Skipping rayrai_quality_lighting example: installed rayrai package does not expose spotlight/area-light, FXAA, and DoF quality APIs")
endif()
create_rayrai_executable(rayrai_coacd_mesh_approximation src/rayrai/assets/rayrai_coacd_mesh_approximation.cpp)
create_rayrai_executable(rayrai_visual_asset_support src/rayrai/assets/rayrai_visual_asset_support.cpp)

# Runtime scene editing and collision detection features added after older binary packages.
raisim_header_has("raisim/World.hpp" "cloneSingleBodyObject" _raisim_has_runtime_scene_editing)
if(_raisim_has_runtime_scene_editing)
    create_rayrai_executable(rayrai_runtime_scene_editing src/rayrai/runtime/rayrai_runtime_scene_editing.cpp)
else()
    message(STATUS "Skipping rayrai_runtime_scene_editing example: installed raisim package does not expose runtime scene editing APIs")
endif()

raisim_header_has("raisim/contact_engine/contact_engine.h" "sweptCcdEnabled" _raisim_has_swept_ccd)
if(_raisim_has_swept_ccd)
    create_rayrai_executable(rayrai_swept_ccd src/rayrai/collision/rayrai_swept_ccd.cpp)
else()
    message(STATUS "Skipping rayrai_swept_ccd example: installed raisim package does not expose swept CCD settings")
endif()

# Tools.
create_rayrai_executable(rayrai_tcp_viewer src/rayrai/tools/rayrai_tcp_viewer.cpp)
