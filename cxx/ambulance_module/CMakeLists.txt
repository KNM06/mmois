file(GLOB SOURCES CONFIGURE_DEPENDS
    "*.cpp" "*.hpp"
    "agents/*.cpp" "agents/*.hpp"
    "keynodes/*.hpp"
)

add_library(ambulance_module SHARED ${SOURCES})
target_link_libraries(ambulance_module
    LINK_PUBLIC sc-machine::sc-memory
    LINK_PUBLIC sc-machine::sc-agents-common
    LINK_PUBLIC scl-machine::inference
    LINK_PUBLIC ps-common-lib::common-utils
)
target_include_directories(ambulance_module
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
set_target_properties(ambulance_module PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/extensions)

if(${SC_CLANG_FORMAT_CODE})
    target_clangformat_setup(ambulance_module)
endif()
find_package(GTest REQUIRED)
include(GoogleTest)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    "test/*.cpp"
)
add_executable(ambulance_module_tests ${TEST_SOURCES})
target_link_libraries(ambulance_module_tests
    LINK_PRIVATE GTest::gtest_main
    LINK_PRIVATE ambulance_module
)
target_include_directories(ambulance_module_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
gtest_discover_tests(ambulance_module_tests
    TEST_LIST ${TEST_SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
)

