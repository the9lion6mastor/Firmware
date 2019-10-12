set(MODULE_CFLAGS)
px4_add_module(
        MODULE modules__offboard_pub
        MAIN offboard_pub
    COMPILE_FLAGS
        -Os
    SRCS
                offboard_pub.c
    DEPENDS
        #platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
