find_package(CUDA REQUIRED)
find_package(GSLIC REQUIRED)
find_package(OpenCV2 REQUIRED)
find_package(HAL 0.1 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Calibu 0.1 REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${GSLIC_INCLUDE_DIRS})
include_directories(${OpenCV2_INCLUDE_DIR})
include_directories(${HAL_INCLUDE_DIRS})
include_directories(${PROTOBUF_INCLUDE_DIRS})

list(APPEND HDRS )
list(APPEND SRCS main.cpp)

add_executable(gslic-app ${HDRS} ${SRCS})

target_link_libraries(gslic-app ${CUDA_LIBRARIES})
target_link_libraries(gslic-app ${GSLIC_LIBRARIES})
target_link_libraries(gslic-app ${OpenCV2_LIBRARIES})
target_link_libraries(gslic-app ${HAL_LIBRARIES})
target_link_libraries(gslic-app ${PROTOBUF_LIBRARIES})
