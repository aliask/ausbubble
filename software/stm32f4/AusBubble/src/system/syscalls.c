#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "usbd_cdc_vcp.h"

#undef errno
extern int errno;
extern int  _end;

caddr_t _sbrk ( int incr )
{
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL)
	{
		heap = (unsigned char *)&_end;
	}
	prev_heap = heap;

	heap += incr;

	return (caddr_t) prev_heap;
}

int link(char *old, char *new)
{
	return -1;
}

int _close(int file)
{
	return -1;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _read(int file, char *ptr, int len)
{
    if (file != 0)
    {
        return 0;
    }

    // Use USB CDC Port for stdin
    while(!VCP_get_char((uint8_t*)ptr)){};

    // Echo typed characters
    VCP_put_char((uint8_t)*ptr);

    return 1;
}

int _write(int file, char *ptr, int len)
{
    VCP_send_buffer((uint8_t*)ptr, len);
    return len;
}

void abort(void)
{
	while(1);
}
