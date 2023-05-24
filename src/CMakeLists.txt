project(${CMAKE_PROJECT_NAME})

set(SOURCES	
	llcm.cpp
)

add_library( llcm OBJECT 
	${SOURCES}
)

target_include_directories(llcm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(llcm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(llcm PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)
