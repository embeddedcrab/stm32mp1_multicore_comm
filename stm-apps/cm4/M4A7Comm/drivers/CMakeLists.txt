
# CMSIS and BSP CMake

cmake_minimum_required(VERSION 3.15)


set(Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/STM32MP15xx_DISCO/stm32mp15xx_disco_bus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/STM32MP15xx_DISCO/stm32mp15xx_disco_stpmic1.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/STM32MP15xx_DISCO/stm32mp15xx_disco.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Core_A/Source/irq_ctrl_gic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_crc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cryp_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cryp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dac_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dac.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dcmi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dfsdm_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dfsdm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_hash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_hash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_hsem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_i2c_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_ipcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_lptim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_mdios.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_mdma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_msp_template.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_qspi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rtc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rtc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_sai_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_sai.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_sd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_sd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_smartcard_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_smartcard.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_smbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_spdifrx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_spi_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_sram.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_timebase_tim_template.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_uart_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_usart_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_wwdg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_delayblock.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_fmc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_lptim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_rtc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_sdmmc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_ll_utils.c
)

set_source_files_properties(${Drivers_Src}
    PROPERTIES COMPILE_FLAGS ${STM32_DEFINES}
)

add_library(bsp_drivers STATIC ${Drivers_Src})

target_include_directories(bsp_drivers
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/STM32MP15xx_DISCO
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Core_A/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Device/ST/STM32MP1xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
    #${CMAKE_CURRENT_SOURCE_DIR}/RTOS2/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Inc
    ${PROJECT_SOURCE_DIR}/inc
# where top-level project will look for the library's public headers
PUBLIC
    $<BUILD_INTERFACE:
        ${CMAKE_CURRENT_SOURCE_DIR}/BSP/STM32MP15xx_DISCO
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Core/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Core_A/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Device/ST/STM32MP1xx/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Legacy
        ${CMAKE_CURRENT_SOURCE_DIR}/STM32MP1xx_HAL_Driver/Inc>
    # where external projects will look for the library's public headers
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(bsp_drivers PROPERTIES LINKER_LANGUAGE C)


# Create package of drivers
install(TARGETS bsp_drivers DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN *.h
  PATTERN *.inl
  PATTERN *.hpp
  PATTERN *.ih
)