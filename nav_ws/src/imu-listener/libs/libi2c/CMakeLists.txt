add_library(i2c SHARED
    src/i2c.c)

target_include_directories(i2c PRIVATE include)

install(TARGETS i2c
    LIBRARY DESTINATION lib/${PROJECT_NAME}/i2c
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
