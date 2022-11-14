# build the a concrete examples
add_executable(8_led_blink)


#set(rp2040_shift_register rp2040_shift_register.h)

# must match with executable name and source file names
target_sources(8_led_blink PRIVATE 8_led_blink.c rp2040_shift_register.c)



# define where to find the files for the lib <pico-shift-register>
#target_link_libraries(8_led_blink pico_stdlib pico-shift-register)
target_link_libraries(8_led_blink pico_stdlib  )

pico_add_extra_outputs(8_led_blink)