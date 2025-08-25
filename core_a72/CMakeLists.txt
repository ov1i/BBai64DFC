cmake_minimum_required(VERSION 3.5)
project(A72Core)

add_executable(telemetry_tx $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/telemetry/tele_tx.cpp>)
add_executable(image_tx $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/isp/img_tx.cpp>)
add_executable(isp $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/isp/isp.cpp>)

target_include_directories(telemetry_tx PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interface>
)

target_include_directories(image_tx PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interface>
)

target_include_directories(isp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interface>
)

install(TARGETS telemetry_tx DESTINATION bin)
install(TARGETS image_tx DESTINATION bin)
install(TARGETS isp DESTINATION bin)

