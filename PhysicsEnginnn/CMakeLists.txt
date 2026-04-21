# CMakeList.txt : CMake project for PhysicsEnginnn

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PHYSICSENGINNN_ENABLE_SDL2 "Enable SDL2 debug rendering" ON)

set(PHYSICS_ENGINE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/math/Vec2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/math/Transform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/World.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/RigidBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/forces/ForceRegistry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/Shapes.cpp
)

set(PHYSICS_APP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/main.cpp
)

add_library(physics_engine STATIC ${PHYSICS_ENGINE_SOURCES})

target_include_directories(physics_engine
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(PhysicsEnginnn ${PHYSICS_APP_SOURCES})

target_link_libraries(PhysicsEnginnn
    PRIVATE
        physics_engine
)

if (PHYSICSENGINNN_ENABLE_SDL2)
    find_package(SDL2 CONFIG QUIET)

    if (TARGET SDL2::SDL2 OR TARGET SDL2::SDL2-static)
        target_sources(physics_engine
            PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/DebugRenderer.cpp
        )

        target_compile_definitions(physics_engine PUBLIC PHYSICSENGINNN_HAS_SDL2=1)

        if (TARGET SDL2::SDL2)
            target_link_libraries(physics_engine PRIVATE SDL2::SDL2)
        else()
            target_link_libraries(physics_engine PRIVATE SDL2::SDL2-static)
        endif()

        if (TARGET SDL2::SDL2main)
            target_link_libraries(PhysicsEnginnn PRIVATE SDL2::SDL2main)
        endif()
    else()
        message(STATUS "SDL2 not available for current compiler/triplet; building without renderer.")
    endif()
endif()
