cmake_minimum_required(VERSION 3.1...3.19)

set(CMAKE_CXX_FLAGS_RELEASE "/O2")

project(test-isotropicremesher)

include_directories(
    ../src
    ./
)

add_executable(test-isotropicremesher 
    main.cpp
    
    ../src/vector3.h
    
    ../src/vector2.h
    
    ../src/double.h
    
    ../src/axisalignedboundingbox.h
    
    ../src/axisalignedboundingboxtree.cpp
    ../src/axisalignedboundingboxtree.h
    
    ../src/isotropichalfedgemesh.cpp
    ../src/isotropichalfedgemesh.h
    
    ../src/isotropicremesher.cpp
    ../src/isotropicremesher.h
)