set(MODULE_CFLAGS)
px4_add_module(
        MODULE modules__read_point
        MAIN read_point
    COMPILE_FLAGS
        -Os
    SRCS
                read_point.c
    DEPENDS
        platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 

