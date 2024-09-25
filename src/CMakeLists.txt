# find_package(gflags REQUIRED)
add_subdirectory(gcs)
add_subdirectory(test)

add_library(helpers SHARED "${PROJECT_SOURCE_DIR}/include/helpers.h" helpers.cpp)
target_include_directories(helpers PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(helpers drake::drake)

add_executable(bimanual_example bimanual_example.cpp)
target_link_libraries(bimanual_example PRIVATE drake::drake gcs helpers)