project(${CMAKE_PROJECT_NAME})

add_executable(gpio.elf 
	gpio.c
)
target_include_directories(gpio.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(gpio.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(gpio.elf PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

target_link_libraries(gpio.elf
	llcm_core
	llcm
)

add_executable(exti.elf 
	exti.c
)
target_include_directories(exti.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(exti.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(exti.elf PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

target_link_libraries(exti.elf
	llcm_core
	llcm
)

add_executable(usart.elf 
	usart.c
)
target_include_directories(usart.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(usart.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(usart.elf PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

target_link_libraries(usart.elf
	llcm_core
	llcm
	usart
)

add_executable(flash.elf 
	flash.c
)
target_include_directories(flash.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(flash.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(flash.elf PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

target_link_libraries(flash.elf
	llcm_core
	llcm
	usart
	flash
)

add_executable(dma.elf 
	dma.c
)
target_include_directories(dma.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(dma.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(dma.elf PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

target_link_libraries(dma.elf
	llcm_core
	llcm
	usart
)
