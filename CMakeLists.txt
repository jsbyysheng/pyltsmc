cmake_minimum_required(VERSION 3.4...3.18)
project(pyltsmc)
add_compile_options(-std=c++14)

set(SMC604_LIBRARIES ${PROJECT_SOURCE_DIR}/libs/libLTSMC.so)

include_directories(
  include
)

add_subdirectory(pybind11)
pybind11_add_module(pyltsmc src/main.cpp)

target_compile_definitions(pyltsmc
                           PRIVATE VERSION_INFO=${VERSION_INFO})

set_target_properties(pyltsmc PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
target_link_libraries(pyltsmc PRIVATE ${SMC604_LIBRARIES})

set_target_properties(pyltsmc PROPERTIES SKIP_BUILD_RPATH FALSE)
set_target_properties(pyltsmc PROPERTIES INSTALL_RPATH "$ORIGIN/libs")
set_target_properties(pyltsmc PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

file(COPY ${PROJECT_SOURCE_DIR}/libs DESTINATION .)