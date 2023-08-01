# llcm4
Low level cortex-M4 library for STM32F411

# Build
* Make sure you have installed arm toolcahin, cmake.
    ```
    $ arm-none-eabi-gcc -v
    $ cmake --version
    ```
 * if none of previous commands led to errors then you can try to compile library using the cmake template.
	```
	$ cd <path-of-this-repo>
	$ mkdir build && cd build
	$ cmake -Darm-toolchain-path=<path-to-arm-toolcahin> -DCMAKE_TOOLCHAIN_FILE=../arm-toolchain.cmake -DBUILD_EXAMPLES=On ..
	$ cmake --build .
	```
