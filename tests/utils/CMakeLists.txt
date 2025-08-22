add_executable(test_utils test_image.cpp test_material.cpp)
target_link_libraries(test_utils 
    PRIVATE 
        rtr-utils
        GTest::gtest_main
)

add_test(NAME UtilsTest COMMAND test_utils)