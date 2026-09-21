#include <sys/stat.h>
#include <errno.h>
#include <stdint.h>
int _write(int file, char *data, int count)
{
    (void)file;
    (void)data;
    return count;
}
int _read(int file, char *data, int count)
{
    (void)file;
    (void)data;
    (void)count;
    return 0;
}
int _close(int file)
{
    (void)file;
    return -1;
}
int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}
int _isatty(int file)
{
    (void)file;
    return 1;
}
int _lseek(int file, int pos, int dir)
{
    (void)file;
    (void)pos;
    (void)dir;
    return 0;
}
int _getpid(void)
{
    return 1;
}
int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}
void *_sbrk(int incr)
{
    (void)incr;
    errno = ENOMEM;
    return (void *)(intptr_t)-1;
}
__attribute__((noreturn)) void _exit(int code)
{
    (void)code;
    for (;;)
    {
    }
}
