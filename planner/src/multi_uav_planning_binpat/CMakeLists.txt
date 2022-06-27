cmake_minimum_required(VERSION 3.15.0 FATAL_ERROR)

project(planning_binpat VERSION 1.0.0)



add_library(planning_binpat STATIC planning_binpat/src/bare/lap.cpp)

target_compile_features(planning_binpat PUBLIC cxx_std_17)

target_include_directories(planning_binpat PUBLIC planning_binpat/include planning_binpat/include/assignment/bare)
