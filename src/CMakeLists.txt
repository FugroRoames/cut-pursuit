project(CUT_PURSUIT_SEG)

cmake_minimum_required(VERSION 2.8.12)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -fopenmp -O3")

# --------------------

include_directories(../include)

find_package(Boost 1.58.0 COMPONENTS graph numpy REQUIRED) #system filesystem thread serialization
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(PythonLibs)
message("PYTHON LIBRARIES ARE " ${PYTHON_LIBRARIES})
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

file(GLOB CP_HEADERS include/*.h)

set(CMAKE_LD_FLAG "${CMAKE_LD_FLAGS} -shared -Wl -fPIC --export-dynamic -O3 -Wall -std=c++11")

# ---- julia
find_package(JlCxx REQUIRED)
set(CMAKE_INSTALL_RPATH "${JlCxx_DIR}/../")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_MACOSX_RPATH 1)

# --- build

add_library(cpjl SHARED cutpursuit_julia.cpp ${CP_HEADERS})
target_link_libraries(cpjl
                    ${Boost_LIBRARIES}
                    ${PYTHON_LIBRARIES}
                    JlCxx::cxxwrap_julia)


#cmake . -DPYTHON_LIBRARY=$CONDAENV/lib/libpython3.6m.so -DPYTHON_INCLUDE_DIR=$CONDAENV/include/python3.6m -DBOOST_INCLUDEDIR=$CONDAENV/include -DEIGEN3_INCLUDE_DIR=$CONDAENV/include/eigen3 -DJlCxx_DIR=/home/josh/.julia/packages/CxxWrap/KcmSi/deps/usr/lib/cmake/JlCxx/ && make -j 8