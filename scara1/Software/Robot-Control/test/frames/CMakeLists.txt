add_executable(framesTest framesTest.cpp)
target_link_libraries(framesTest ${EEROS_LIBRARY})

add_test(frames-test framesTest "${TEST_DATA_DIR}/outputDataFramesTest.txt")
# add_test(frames-test framesTest "${TEST_DATA_DIR}/outputDataFramesTest2.txt")
# add_test(frames-test framesTest "${TEST_DATA_DIR}/outputDataFramesTest3.txt")
# add_test(frames-test framesTest "${TEST_DATA_DIR}/outputDataFramesTest4.txt")
# add_test(frames-test framesTest "${TEST_DATA_DIR}/outputDataFramesTest5.txt")