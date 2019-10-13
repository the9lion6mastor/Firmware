set(MODULE_CFLAGS)
px4_add_module(
        MODULE modules__radar
        MAIN radar
    COMPILE_FLAGS
        -Os
    SRCS
                radar.c
    DEPENDS
        #platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
