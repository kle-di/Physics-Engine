# CMakeList.txt : CMake project for Physics-Engine

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PHYSICSENGINE_ENABLE_SDL2 "Enable SDL2 debug rendering" ON)

set(PHYSICS_ENGINE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/math/Vec2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/math/Transform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core/World.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/RigidBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/CollisionResolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/forces/ForceRegistry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/Shapes.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/Collision.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/broadphase/UniformGridBroadPhase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ccd/Ccd.cpp
)

set(PHYSICS_APP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/MouseInteractionController.cpp
)

add_library(physics_engine STATIC ${PHYSICS_ENGINE_SOURCES})

target_include_directories(physics_engine
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (MSVC)
    target_link_directories(physics_engine
        PUBLIC
            "$ENV{VCToolsInstallDir}/lib/x64"
            "$ENV{WindowsSdkDir}/Lib/$ENV{WindowsSDKVersion}/ucrt/x64"
            "$ENV{WindowsSdkDir}/Lib/$ENV{WindowsSDKVersion}/um/x64"
    )
endif()

add_executable(Physics-Engine ${PHYSICS_APP_SOURCES})

target_link_libraries(Physics-Engine
    PRIVATE
        physics_engine
)

if (PHYSICSENGINE_ENABLE_SDL2)
    find_package(SDL2 CONFIG QUIET)

    if (TARGET SDL2::SDL2 OR TARGET SDL2::SDL2-static)
        target_sources(physics_engine
            PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/DebugRenderer.cpp
        )

        target_compile_definitions(physics_engine PUBLIC PHYSICSENGINE_HAS_SDL2=1)

        if (TARGET SDL2::SDL2)
            target_link_libraries(physics_engine PRIVATE SDL2::SDL2)
            target_link_libraries(Physics-Engine PRIVATE SDL2::SDL2)
        else()
            target_link_libraries(physics_engine PRIVATE SDL2::SDL2-static)
            target_link_libraries(Physics-Engine PRIVATE SDL2::SDL2-static)
        endif()
    else()
        message(STATUS "SDL2 not available for current compiler/triplet; building without renderer.")
    endif()
endif()
