/**************************************************************************//**
 * @file     syscalls_stub.c
 * @brief    Minimal GCC newlib/newlib-nano syscall stubs for bare-metal projects
 *
 * @details
 * This file provides minimal implementations for system calls referenced by
 * newlib/newlib-nano when the project does not use retarget.c.
 *
 * These stubs are intended for projects that do not require console or file I/O.
 *****************************************************************************/

#if defined(__GNUC__) && !defined(__ARMCC_VERSION)

int _close(int file)
{
    (void)file;

    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;

    return 0;
}

int _read(int fd, char *ptr, int len)
{
    (void)fd;
    (void)ptr;
    (void)len;

    return -1;
}

int _write(int fd, char *ptr, int len)
{
    (void)fd;
    (void)ptr;
    (void)len;

    return -1;
}

#endif /* defined(__GNUC__) && !defined(__ARMCC_VERSION) */
