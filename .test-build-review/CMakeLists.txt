cmake_minimum_required(VERSION 3.20)
project(estimator_review CXX)
add_executable(review main.cpp)
target_include_directories(review PRIVATE ../include)
target_compile_features(review PRIVATE cxx_std_17)

