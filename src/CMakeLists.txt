project(${CMAKE_PROJECT_NAME})


add_library( llcm_core OBJECT
	${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/startup_stm32f411xe.s
	${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/system_stm32f4xx.c
	${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/segger_fault_handler.c
)

target_include_directories(llcm_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(llcm_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(llcm_core PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

add_library( llcm OBJECT 
	llcm.c
)

target_include_directories(llcm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(llcm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(llcm PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

add_library( usart OBJECT 
	usart.c
)

target_include_directories(usart PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(usart PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(usart PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

add_library( flash OBJECT 
	flash.c
)

target_include_directories(flash PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(flash PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(flash PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

add_library( spi OBJECT 
	spi.c
)

target_include_directories(spi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(spi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(spi PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)
