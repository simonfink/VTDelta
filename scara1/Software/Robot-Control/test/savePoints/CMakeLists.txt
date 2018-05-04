add_executable(savePointsTest savePointsTest.cpp)
target_link_libraries(savePointsTest ${EEROS_LIBRARY})

# add_test(savePoints-test savePointsTest "${TEST_DATA_DIR}/refSystemOutTest.txt")