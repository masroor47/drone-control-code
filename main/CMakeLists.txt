file(GLOB_RECURSE SOURCES "../src/*.cpp")

idf_component_register(SRCS "main.cpp" ${SOURCES}
                    INCLUDE_DIRS "." "../include"
                    REQUIRES driver esp_system)