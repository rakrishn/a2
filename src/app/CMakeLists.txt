cmake_minimum_required(VERSION 3.5)

project(app)

find_package(Json REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(${PROJECT_NAME}
    main.cpp mechanisms.h
)
target_link_libraries(${PROJECT_NAME}
    optLib
    rbSimLib
    guiLib
)

target_compile_definitions(${PROJECT_NAME} PRIVATE DATA_FOLDER=${DATA_FOLDER})
