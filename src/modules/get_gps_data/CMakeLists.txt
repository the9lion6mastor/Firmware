set(MODULE_CFLAGS)
px4_add_module(
        MODULE modules__get_gps_data
        MAIN get_gps_data
    COMPILE_FLAGS
        -Os
    SRCS
                get_gps_data.c
    DEPENDS
        platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
