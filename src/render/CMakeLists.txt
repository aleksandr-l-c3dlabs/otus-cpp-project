add_library(rtr-render STATIC 
    framebuffer.cpp
    raytracer.cpp
    renderer.cpp
)

set_target_properties(rtr-render PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(rtr-render
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" 
    PRIVATE "${CMAKE_CURRENT_BINARY_DIR}"
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(rtr-render
    PUBLIC
        rtr-utils
        rtr-model
        rtr-camera
    PRIVATE 
        Eigen3::Eigen
)