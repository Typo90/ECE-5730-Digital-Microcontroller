# build the a concrete examples
add_executable(led_cube)


# must match with pio filename and executable name from above
pico_generate_pio_header(led_cube ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(led_cube ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(led_cube ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

#set(rp2040_shift_register rp2040_shift_register.h)

# must match with executable name and source file names
target_sources(led_cube PRIVATE led_cube.c rp2040_shift_register.c vga_graphics.c)


# define where to find the files for the lib <pico-shift-register>
target_link_libraries(led_cube pico_stdlib pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pio hardware_dma hardware_adc hardware_irq)

pico_add_extra_outputs(led_cube)