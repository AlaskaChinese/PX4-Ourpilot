px4_add_module(
 	MODULE examples__rw_uart
 	MAIN rw_uart
 	STACK_MAIN 2000
 	SRCS
		rw_uart.c
 	DEPENDS
 	)
