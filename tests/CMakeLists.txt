cmake_minimum_required(VERSION 3.16)

include(FetchContent)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        release-1.11.0)
FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

include(GoogleTest)

file(GLOB TESTS *.cpp)

foreach (file ${TESTS})
    get_filename_component(tgt ${file} NAME_WE)
    message(STATUS "Adding test \"${tgt}\"")
    add_executable(${tgt} ${file})
    target_compile_features(${tgt} PUBLIC cxx_std_20)
    if (NOT ${CMAKE_HOST_SYSTEM_NAME} MATCHES "Windows")
        target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
                -Woverloaded-virtual -Wsuggest-override)
    endif ()
    target_link_libraries(${tgt} PRIVATE ${LIBRARY_NAME} GTest::gtest GTest::GTest -pthread)
    gtest_discover_tests(${tgt} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach ()
