set(MODULE_CFLAGS)
px4_add_module(
        MODULE modules__read_uart
        MAIN read_uart
    COMPILE_FLAGS
        -Os
    SRCS
                read_uart.c
    DEPENDS
        #platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 

