#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
extern "C"
{
    #include "usbd_cdc_vcp.h"
}

#undef errno
extern int errno;
extern int _end;

/*
 * The default pulls in 70K of garbage
 */
namespace __gnu_cxx
{
    void __verbose_terminate_handler()
    {
        for(;;);
    }
}

/*
 * The default pulls in about 12K of garbage
 */
extern "C" void __cxa_pure_virtual()
{
    for(;;);
}

/*
 * Implement C++ new/delete operators using the heap
 */
void *operator new(size_t size)
{
    return malloc(size);
}

void *operator new[](size_t size)
{
    return malloc(size);
}

void operator delete(void *p)
{
    free(p);
}

void operator delete[](void *p)
{
    free(p);
}

extern "C"
{
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
}

extern "C"
{
    int link(char *old_, char *new_)
    {
        return -1;
    }
}

extern "C"
{
    int _close(int file)
    {
        return -1;
    }
}

extern "C"
{
    int _fstat(int file, struct stat *st)
    {
        st->st_mode = S_IFCHR;
        return 0;
    }
}

extern "C"
{
    int _isatty(int file)
    {
        return 1;
    }
}

extern "C"
{
    int _lseek(int file, int ptr, int dir)
    {
        return 0;
    }
}

extern "C"
{
    int _read(int file, char *ptr, int len)
    {
        if (file != 0)
        {
            return 0;
        }

        /* Use USB CDC Port for stdin */
        while(!VCP_get_char((uint8_t*)ptr)){};

        /* Echo typed characters */
        VCP_put_char((uint8_t)*ptr);

        return 1;
    }
}

extern "C"
{
    int _write(int file, char *ptr, int len)
    {
        VCP_send_buffer((uint8_t*)ptr, len);
        return len;
    }
}

extern "C"
{
    void abort(void)
    {
        while(1);
    }
}
