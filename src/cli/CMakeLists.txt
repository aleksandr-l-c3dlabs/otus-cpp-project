add_executable(rtr-cli
    ppm.cpp
    main.cpp
)

set_target_properties(rtr-cli PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(rtr-cli
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" 
    PRIVATE "${CMAKE_CURRENT_BINARY_DIR}"
)

find_package(Boost REQUIRED COMPONENTS program_options)

target_link_libraries(rtr-cli
    PRIVATE 
        Boost::program_options
        rtr-render
)

install(TARGETS rtr-cli RUNTIME DESTINATION bin)