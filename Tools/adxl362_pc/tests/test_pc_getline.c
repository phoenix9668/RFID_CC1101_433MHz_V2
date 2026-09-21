#define _POSIX_C_SOURCE 200809L

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pc_getline.h"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s temporary-file-path\n", argv[0]);
        return 2;
    }
    char *line = NULL;
    size_t capacity = 0;
    FILE *input = fopen(argv[1], "w+b");
    if (input == NULL)
    {
        perror("cannot create test input");
        return 1;
    }
    assert(pc_getline(&line, &capacity, input) == -1);

    rewind(input);
    assert(fputs("\r\n\n", input) >= 0);
    for (size_t i = 0; i < 4096; i++)
        assert(fputc('x', input) != EOF);
    assert(fputs("\ntail", input) >= 0);
    rewind(input);

    assert(pc_getline(&line, &capacity, input) == 2);
    assert(strcmp(line, "\r\n") == 0);
    assert(pc_getline(&line, &capacity, input) == 1);
    assert(strcmp(line, "\n") == 0);

    assert(pc_getline(&line, &capacity, input) == 4097);
    for (size_t i = 0; i < 4096; i++)
        assert(line[i] == 'x');
    assert(line[4096] == '\n' && line[4097] == '\0');

    assert(pc_getline(&line, &capacity, input) == 4);
    assert(strcmp(line, "tail") == 0);
    assert(pc_getline(&line, &capacity, input) == -1);
    assert(pc_getline(&line, &capacity, input) == -1);

    free(line);
    assert(fclose(input) == 0);
    assert(remove(argv[1]) == 0);
    puts("pc_getline: empty input, CRLF, blank line, growth and EOF passed");
    return 0;
}
