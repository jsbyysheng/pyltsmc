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

target_link_libraries(pyltsmc 
                      PRIVATE ${SMC604_LIBRARIES})
