/*
 * Минимальные заглушки для newlib. Нужны только потому, что мы пользуемся
 * стандартным snprintf: он тянет за собой _sbrk, а тот — границы кучи.
 * Файлового ввода-вывода в прошивке нет, поэтому остальные вызовы — пустышки.
 */
#include <errno.h>
#include <sys/stat.h>

extern char _sheap, _eheap;

void *_sbrk(int incr)
{
    static char *brk = &_sheap;
    char *prev = brk;
    if (brk + incr > &_eheap) {
        errno = ENOMEM;
        return (void *)-1;
    }
    brk += incr;
    return prev;
}

int _close(int fd)                        { (void)fd; return -1; }
int _fstat(int fd, struct stat *st)       { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                       { (void)fd; return 1; }
int _lseek(int fd, int off, int whence)   { (void)fd; (void)off; (void)whence; return -1; }
int _read(int fd, char *p, int len)       { (void)fd; (void)p; (void)len; return -1; }
int _write(int fd, const char *p, int len){ (void)fd; (void)p; return len; }
void _exit(int code)                      { (void)code; while (1) { } }
void _kill(int pid, int sig)              { (void)pid; (void)sig; }
int _getpid(void)                         { return 1; }
