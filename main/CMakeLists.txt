file(GLOB_RECURSE SOURCES "../src/*.cpp")

idf_component_register(SRCS "main.cpp" ${SOURCES}
                    INCLUDE_DIRS "." "../include"
                    REQUIRES driver esp_system esp_wifi esp_event esp_netif lwip nvs_flash esp_eth esp_http_server json)