cmake_minimum_required(VERSION 3.16)

set(LIBRARY_NAME
    modbus_lib
)

set(GLOBAL_NAMESPACE
    xitren
)

project(${LIBRARY_NAME}
	DESCRIPTION "Modbus library"
    VERSION 1.0.0
	LANGUAGES CXX
)

add_library(${LIBRARY_NAME} INTERFACE)
add_library(${GLOBAL_NAMESPACE}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

target_include_directories(
        ${LIBRARY_NAME} INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>)

message(STATUS "Adding library project \"${LIBRARY_NAME}\"")

enable_testing()
add_subdirectory(tests)