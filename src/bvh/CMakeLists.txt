add_library(rtr-bvh STATIC 
    bvh.cpp
)

set_target_properties(rtr-bvh PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(rtr-bvh
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" 
    PRIVATE "${CMAKE_CURRENT_BINARY_DIR}"
)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(rtr-bvh
    PUBLIC
        rtr-utils
    PRIVATE 
        Eigen3::Eigen
)