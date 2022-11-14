cmake_minimum_required(VERSION 3.16)

project(project_name LANGUAGES C ASM)

set(NRF5_SDK_VERSION "17.0.2")
set(NRF5_TARGET "nrf52832_xxaa" CACHE STRING "TARGET")
set(NRF5_SOFTDEVICE_VARIANT "s132" CACHE STRING "SOFTDEVICE_VARIANT")
set(NRF5_LINKER_SCRIPT "${PROJECT_SOURCE_DIR}/linker.ld" CACHE STRING "LINKER_SCRIPT")
set(NRF5_SDKCONFIG_PATH "${PROJECT_SOURCE_DIR}/config")

add_definitions(
  # -DNDEBUG
  -DUSE_APP_CONFIG
  -DCONFIG_GPIO_AS_PINRESET
  -DFLOAT_ABI_HARD
  -DNRF52_PAN_74
  -DSOFTDEVICE_PRESENT
)

list(APPEND CMAKE_MODULE_PATH "./cmake")
include("nrf5")

include_directories("." "./config" "./bluetooth/service" "./bluetooth" "./module" "./module/display")

# file(GLOB_RECURSE USER_SOURCES Src/*.c)
set(SOURCE_FILES
  "main.c"
  "bluetooth/bluetooth.c"
  "bluetooth/bt_air.c"
  "bluetooth/service/ble_user_srv.c"
  "module/uevent.c"
  "module/platform.c"
  "module/timedate.c"
  "module/led_drv.c"
)
add_executable(${CMAKE_PROJECT_NAME} ${SOURCE_FILES})

nrf5_target(${CMAKE_PROJECT_NAME})
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
  # Common
  nrf5_soc
  nrf5_mdk
  nrf5_section
  nrf5_strerror
  nrf5_atomic
  nrf5_atfifo
  nrf5_atflags
  nrf5_memobj
  nrf5_balloc
  nrf5_ringbuf
  nrf5_pwr_mgmt
  # External
  nrf5_ext_fprintf
  # Drivers
  nrf5_nrfx_common
  nrf5_drv_rng
  nrf5_queue
  nrf5_nrfx_rng
  nrf5_nrfx_hal
  nrf5_nrfx_prs
  nrf5_nrfx_gpiote
  nrf5_nrfx_saadc
  nrf5_nrfx_spi
  nrf5_drv_spi
  nrf5_nrfx_clock
  nrf5_drv_clock
  nrf5_nrfx_rtc
  # nrf5_nrfx_uarte
  # nrf5_nrfx_uart
  # nrf5_drv_uart
  # Logger
  nrf5_log
  nrf5_log_backend_rtt
  nrf5_log_backend_serial
  nrf5_ext_fprintf
  nrf5_ext_segger_rtt
  # nrf5_log_backend_uart
  nrf5_log_default_backends
  # Application
  nrf5_config
  nrf5_delay
  nrf5_app_scheduler
  nrf5_app_error
  nrf5_app_util_platform
  nrf5_app_timer
  nrf5_app_button
  # BLE
  nrf5_sdh
  nrf5_ble_common
  nrf5_ble_advertising
  nrf5_ble_link_ctx_manager
  nrf5_ble_db_discovery
  nrf5_ble_gq
  nrf5_ble_gatt
  nrf5_ble_qwr
  nrf5_ble_scan
)
