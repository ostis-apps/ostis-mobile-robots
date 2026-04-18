file(GLOB SOURCES CONFIGURE_DEPENDS
    "units/*.cpp"
    "units/*.hpp"
)

add_executable(transport-module-tests ${SOURCES})
target_link_libraries(transport-module-tests
    LINK_PRIVATE GTest::gtest_main
    LINK_PRIVATE transport-module
)
target_include_directories(transport-module-tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/units
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..
)
gtest_discover_tests(transport-module-tests WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/units)

if(${SC_CLANG_FORMAT_CODE})
    target_clangformat_setup(transport-module-tests)
endif()
