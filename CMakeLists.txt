
  cmake_minimum_required(VERSION 3.13)

  set(CMAKE_C_STANDARD 11)
  set(CMAKE_CXX_STANDARD 17)

  include(pico_sdk_import.cmake)

  project(PLynx C CXX ASM)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_CXX_STANDARD 17)

  pico_sdk_init()
  #add display drivers
  add_subdirectory(ScrKbd-drivers display-drv-build)

  add_executable(PLynx
	Lynx.c 
#        ide.c 
        z80dis.c 
#        acia.c
        hw_config.c
        dictionary.c
	iniparser.c
	ScrKbd-drivers/lscreen.c
        ScrKbd-drivers/keyboard.c


  )




  add_subdirectory(FatFs_SPI build)


  add_subdirectory(libz80)
  link_directories(libz80)


  # Add the standard library and FatFS/SPI to the build
  target_link_libraries(PLynx 
    pico_multicore
    hardware_gpio 
    hardware_pwm
    pico_stdlib
    hardware_pio 
    hardware_dma 
    hardware_irq
    hardware_i2c
    FatFs_SPI
    libz80
#display
    ili9341
    gfx

  )

  pico_enable_stdio_usb(PLynx 1)
  pico_enable_stdio_uart(PLynx 1)

  pico_add_extra_outputs(PLynx)

  add_compile_definitions(PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1)
  add_compile_definitions(PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=0)


