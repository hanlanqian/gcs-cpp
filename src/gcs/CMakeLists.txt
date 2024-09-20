add_library(gcs SHARED "${PROJECT_SOURCE_DIR}/include/gcs.h" gcs.cpp)
target_include_directories(gcs PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(gcs drake::drake)