file(GLOB SOURCES CONFIGURE_DEPENDS
    "*.cpp" "*.hpp"
    "*/*.cpp" "*/*.hpp"
)

add_library(transport-module SHARED ${SOURCES})
target_link_libraries(transport-module
    LINK_PUBLIC sc-machine::sc-memory
)
target_include_directories(transport-module
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
set_target_properties(transport-module PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/extensions)

if(${SC_CLANG_FORMAT_CODE})
    target_clangformat_setup(transport-module)
endif()

if(${SC_BUILD_TESTS})
    add_subdirectory(test)
endif()
