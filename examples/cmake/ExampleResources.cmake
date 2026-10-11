# Refresh resource directories even when only assets changed since configuration.
if(CMAKE_SCRIPT_MODE_FILE)
    if(NOT IS_DIRECTORY "${RESOURCE_SOURCE}" OR "${RESOURCE_DESTINATION}" STREQUAL "")
        message(FATAL_ERROR "Example resources require a source and destination directory")
    endif()
    # COPY preserves timestamps and skips unchanged files. Discover new files at build time.
    file(COPY "${RESOURCE_SOURCE}/" DESTINATION "${RESOURCE_DESTINATION}")
else()
    set(_raisim_example_resources_script "${CMAKE_CURRENT_LIST_FILE}")
    function(sync_example_resources target source destination)
        add_custom_target(${target}
            COMMAND "${CMAKE_COMMAND}"
                "-DRESOURCE_SOURCE=${source}"
                "-DRESOURCE_DESTINATION=${destination}"
                -P "${_raisim_example_resources_script}"
            COMMENT "Refreshing ${target}"
            VERBATIM)
    endfunction()
endif()
