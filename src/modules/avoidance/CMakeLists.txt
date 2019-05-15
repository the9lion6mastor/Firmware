set(MODULE_CFLAGS)
px4_add_module(
        MODULE modules__avoidance
        MAIN avoidance
    COMPILE_FLAGS
        -Os
    SRCS
                avoidance.c
    DEPENDS
        platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
