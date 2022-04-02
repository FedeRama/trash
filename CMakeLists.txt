cmake_minimum_required(VERSION 3.7)
project(TestFrenet LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2")
set(CMAKE_CXX_STANDARD_LIBRARIES -ldl)

include_directories(include)
link_directories(include)
set(LIBRARIES
)

add_library(support STATIC 
        src/polynomials.cpp
        src/cubic_spline_planner.cpp
        src/frenet_optimal_trajectory.cpp
        src/frenet_optimal_trajectory.cu
        include/polynomials.hpp
        include/cubic_spline_planner.hpp
        include/frenet_optimal_trajectory.hpp
        include/debug.h
        )
        
target_compile_features(support PUBLIC cxx_std_14)

set_target_properties(support
                        PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

set_target_properties(support 
                        PROPERTIES CUDA_ARCHITECTURES 72)

                        

add_executable(frenet_test demo/test.cpp 
                )
                
set_target_properties(frenet_test 
                        PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

set_target_properties(frenet_test 
                        PROPERTIES CUDA_ARCHITECTURES 72)
             
#find_package(PythonLibs 2.7)
find_package(Python COMPONENTS Development)
target_include_directories(frenet_test PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(frenet_test PRIVATE support ${LIBRARIES} ${Python_LIBRARIES})
