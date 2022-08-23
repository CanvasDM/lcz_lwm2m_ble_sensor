if(CONFIG_LCZ_LWM2M_BLE_SENSOR)
    zephyr_include_directories(include)
    zephyr_sources(source/lcz_lwm2m_ble_sensor.c)
endif()