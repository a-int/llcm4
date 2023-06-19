project(${CMAKE_PROJECT_NAME})

set(SOURCES	
	${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/startup_stm32f411xe.s
	${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/system_stm32f4xx.c
	${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/segger_fault_handler.c
)

add_library( llcm OBJECT 
	${SOURCES}
	llcm.cpp
)

target_include_directories(llcm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(llcm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(llcm PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)
