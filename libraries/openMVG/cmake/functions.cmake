include(FetchContent)

function(fetch_project)
    cmake_parse_arguments(FETCH_SOURCE "" "NAME;URL;CMAKE_ARGS" "" ${ARGN})
    FetchContent_Declare(${FETCH_SOURCE_NAME}
        URL ${FETCH_SOURCE_URL}
    )

    # if(${FETCH_SOURCE_CMAKE_ARGS})
    # set_target_properties(${FETCH_SOURCE_NAME} PROPERTY
    # WITH_WERROR OFF
    # )
    # endif()
    FetchContent_MakeAvailable(${FETCH_SOURCE_NAME})
endfunction()