add_library(rtr-utils INTERFACE)

target_include_directories(rtr-utils INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/")
target_compile_features(rtr-utils INTERFACE cxx_std_20)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(rtr-utils INTERFACE Eigen3::Eigen)