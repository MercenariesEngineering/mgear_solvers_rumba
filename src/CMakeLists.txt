cmake_minimum_required(VERSION 3.10)
project( mgear_solvers_plugin )

file( GLOB_RECURSE MGEAR_SOLVERS_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h" )
file( GLOB_RECURSE MGEAR_SOLVERS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" )

find_package(rumba REQUIRED)
include_directories(${rumba_INCLUDE_DIRS})
link_directories(${rumba_LIBRARY_DIRS})

add_library( mgear_solvers_plugin SHARED ${MGEAR_SOLVERS_HEADERS} ${MGEAR_SOLVERS_SOURCES} )
target_link_libraries( mgear_solvers_plugin PUBLIC ${rumba_LIBRARIES} )
target_include_directories( mgear_solvers_plugin PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" )

# On visual, use the release dll runtime (MD) in Debug
if (MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "/MD /Zi /Ob0 /Od /RTC1")
endif()

# Optional output directory
foreach(BUILD_TYPE RELEASE DEBUG DEV CI PROFILE MINSIZEREL RELWITHDEBINFO)
	if(DEFINED rumba_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE})
		set_target_properties(mgear_solvers_plugin PROPERTIES
			RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE} "${rumba_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE}}"
			ARCHIVE_OUTPUT_DIRECTORY_${BUILD_TYPE} "${rumba_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE}}"
			LIBRARY_OUTPUT_DIRECTORY_${BUILD_TYPE} "${rumba_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE}}"
		)
	endif()
endforeach()
if(DEFINED rumba_RUNTIME_OUTPUT_DIRECTORY)
	set_target_properties(mgear_solvers_plugin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${rumba_RUNTIME_OUTPUT_DIRECTORY}")
endif()
