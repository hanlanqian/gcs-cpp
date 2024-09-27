add_executable(test_gcs gcs_test.cpp)
target_link_libraries(test_gcs gcs helpers drake::drake)