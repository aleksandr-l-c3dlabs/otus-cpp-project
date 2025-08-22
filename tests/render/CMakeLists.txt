add_executable(test_render 
    test_calculate_lighting.cpp
    test_reflect.cpp
    test_hit_triangle.cpp
)
target_link_libraries(test_render 
    PRIVATE 
        rtr-render
        GTest::gtest_main
)

add_test(NAME RenderTest COMMAND test_render)