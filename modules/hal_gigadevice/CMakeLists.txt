# Copyright (c) 2021 ATL-Electronics
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_HAS_GD32_HAL)

string(TOUPPER ${CONFIG_SOC} gd32_soc_uc)

set(gd32_soc_dir     ${ZEPHYR_HAL_GIGADEVICE_MODULE_DIR}/${CONFIG_SOC_SERIES})
set(gd32_cmsis_dir   ${gd32_soc_dir}/cmsis/gd/${CONFIG_SOC_SERIES})
set(gd32_std_dir     ${gd32_soc_dir}/standard_peripheral)
set(gd32_std_src_dir ${gd32_std_dir}/source)

zephyr_library_named(hal_gigadevice)

zephyr_compile_definitions(${gd32_soc_uc})

# Global includes to be used outside hal_gigadevice
zephyr_include_directories(${gd32_cmsis_dir}/include)
zephyr_include_directories(${gd32_std_dir}/include)
zephyr_include_directories(${ZEPHYR_HAL_GIGADEVICE_MODULE_DIR}/include)

zephyr_library_sources(${gd32_cmsis_dir}/source/system_${CONFIG_SOC_SERIES}.c)

zephyr_library_sources_ifdef(CONFIG_USE_GD32_ADC    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_adc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_BKP    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_bkp.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_CAN    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_can.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_CRC    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_crc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_CTC    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_ctc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_DAC    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_dac.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_DBG    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_dbg.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_DCI    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_dci.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_DMA    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_dma.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_ENET   ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_enet.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_EXMC   ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_exmc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_EXTI   ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_exti.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_FMC    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_fmc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_FWDGT  ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_fwdgt.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_GPIO   ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_gpio.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_I2C    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_i2c.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_IPA    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_ipa.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_IREF   ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_iref.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_MISC   ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_misc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_PMU    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_pmu.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_RCU    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_rcu.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_RTC    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_rtc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_SDIO   ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_sdio.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_SPI    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_spi.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_SYSCFG ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_syscfg.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_TIMER  ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_timer.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_TLI    ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_tli.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_TRNG   ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_trng.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_USART  ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_usart.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_WWDGT  ${gd32_std_src_dir}/${CONFIG_SOC_SERIES}_wwdgt.c)

endif()
