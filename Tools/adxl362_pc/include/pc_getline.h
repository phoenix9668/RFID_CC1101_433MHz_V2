#ifndef ADXL362_PC_GETLINE_H
#define ADXL362_PC_GETLINE_H

#ifndef _WIN32
#define pc_getline getline
#else

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

/* MinGW runtimes do not all provide the POSIX getline extension. */
static ssize_t pc_getline(char **line, size_t *capacity, FILE *input)
{
    size_t length = 0;
    int ch;

    while ((ch = fgetc(input)) != EOF)
    {
        if (*line == NULL || length + 1 >= *capacity)
        {
            if (*capacity > (size_t)PTRDIFF_MAX / 2)
            {
                errno = ERANGE;
                return -1;
            }
            size_t next_capacity = *capacity == 0 ? 256 : *capacity * 2;
            char *next = realloc(*line, next_capacity);
            if (next == NULL)
                return -1;
            *line = next;
            *capacity = next_capacity;
        }
        (*line)[length++] = (char)ch;
        if (ch == '\n')
            break;
    }

    if (ferror(input) || (ch == EOF && length == 0))
        return -1;

    (*line)[length] = '\0';
    return (ssize_t)length;
}

#endif
#endif
