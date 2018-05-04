add_executable(refSystemTest refSystemTest.cpp)
target_link_libraries(refSystemTest ${EEROS_LIBRARY})

add_test(refSystems-test refSystemTest "${TEST_DATA_DIR}/refSystemOutTest.txt")
# add_test(refSystems-test refSystemsTest "${TEST_DATA_DIR}/refSystemOutTest2.txt")
# add_test(refSystems-test refSystemsTest "${TEST_DATA_DIR}/refSystemOutTest3.txt")
# add_test(refSystems-test refSystemsTest "${TEST_DATA_DIR}/refSystemOutTest4.txt")
# add_test(refSystems-test refSystemsTest "${TEST_DATA_DIR}/refSystemOutTest5.txt")