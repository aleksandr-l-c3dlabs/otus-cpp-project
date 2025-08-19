include(FetchContent)

FetchContent_Declare(
    stb
    URL https://raw.githubusercontent.com/nothings/stb/master/stb_image.h
    DOWNLOAD_NO_EXTRACT TRUE
)

FetchContent_MakeAvailable(stb)

add_library(stb INTERFACE)
target_include_directories(stb INTERFACE 
    ${stb_SOURCE_DIR}
    ${stb_BINARY_DIR}
)
target_compile_definitions(stb INTERFACE STB_IMAGE_IMPLEMENTATION)

###################################

add_library(rtr-model STATIC 
    model.cpp
)

set_target_properties(rtr-model PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(rtr-model
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" 
    PRIVATE "${CMAKE_CURRENT_BINARY_DIR}"
)


find_package(tinyobjloader CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(rtr-model
    PUBLIC
        rtr-utils
        rtr-bvh
    PRIVATE 
        tinyobjloader::tinyobjloader
        Eigen3::Eigen
        stb
)