project(${CMAKE_PROJECT_NAME})

add_executable(test.elf 
	test.cpp
)
target_include_directories(test.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/vendor/)
target_include_directories(test.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include/llcm/)
target_link_options(test.elf PRIVATE 
    ${arch_flags} 
    ${common_flags}
    ${link_flags}
)

target_link_libraries(test.elf
	llcm
)