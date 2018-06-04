#include <sys/types.h>
#include "Compiler.h"
#include "HardwareProfile.h"

#include "uart1.h"

extern int _end;
#define	STACK_TOP	0xa0002000

#if	0
/* BSD internals */
typedef	u_int64_t	u_quad_t;	/* quads */
typedef	int64_t		quad_t;
typedef	quad_t *	qaddr_t;

typedef	char *		caddr_t;	/* core address */
#endif

/*
 * sbrk -- changes heap size size. Get nbytes more
 *         RAM. We just increment a pointer in what's
 *         left of memory on the board.
 */
caddr_t sbrk(int nbytes)
{
    static caddr_t heap_ptr = NULL;
    caddr_t        base;

    if (heap_ptr == NULL) {
        heap_ptr = (caddr_t)&_end;
//		UART1PrintString( "sbrk heap=" );
//		UART1PutHex4( &_end );
    }

    if ((STACK_TOP >= (unsigned int) heap_ptr) ) {
        base = heap_ptr;
        heap_ptr += nbytes;
#if	0
			UART1PrintString( "sbrk" );
			UART1PutHex4( nbytes );
			UART1PrintString( "=" );
			UART1PutHex4( base );
			UART1PrintString( "\n" );
#endif
        return (base);
    } else {
//        uart_send("heap full!\r\n");
        return ((caddr_t)-1);
    }
}

