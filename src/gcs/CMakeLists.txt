add_library(gcs_helpers SHARED "${PROJECT_SOURCE_DIR}/include/gcs_helpers.h" gcs_helpers.cpp)
target_include_directories(gcs_helpers PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(gcs_helpers drake::drake)

add_library(gcs SHARED "${PROJECT_SOURCE_DIR}/include/gcs.h" gcs.cpp)
target_include_directories(gcs PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(gcs drake::drake gcs_helpers)