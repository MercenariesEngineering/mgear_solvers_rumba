cmake_minimum_required(VERSION 3.10)
project( mgear_solvers )

file( GLOB_RECURSE MGEAR_SOLVERS_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" )
file( GLOB_RECURSE MGEAR_SOLVERS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" )

include_directories( ${RUMBA_SDK}/include )
link_directories( ${RUMBA_SDK}/lib )
 
add_library( mgear_solvers SHARED ${MGEAR_SOLVERS_HEADERS} ${MGEAR_SOLVERS_SOURCES} ) 
target_link_libraries( mgear_solvers PUBLIC kernel Iex Imath )
target_include_directories( mgear_solvers PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" )
