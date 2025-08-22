add_executable(test_bvh test_aabb_intersect.cpp test_bvh.cpp)
target_link_libraries(test_bvh 
    PRIVATE 
        rtr-bvh
        GTest::gtest_main
)

add_test(NAME BVHTest COMMAND test_bvh)