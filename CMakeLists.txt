cmake_minimum_required(VERSION 2.8)

add_subdirectory(src/)
option(COMPILE_TEST "Compile Tests" ON)
if(COMPILE_TEST) 
   add_subdirectory(test/)
endif(COMPILE_TEST)