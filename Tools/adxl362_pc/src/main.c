#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adxl362.h"

#define SAMPLE_RATE_HZ 25
#define SAMPLES_PER_FIFO (_FIFO_SAMPLES_LEN / 6)
#define FIFO_OUTPUT_COUNT (SAMPLES_PER_FIFO / SAMPLE_RATE_HZ)
#define MAX_COLUMNS 256

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    char *line;
} csv_sample_t;

extern int pc_verbose;

static void print_usage(const char *program)
{
    fprintf(stderr, "Usage: %s [--verbose] input.csv [output.csv]\n", program);
}

static char *trim_line_end(char *line)
{
    size_t length = strlen(line);
    while (length > 0 && (line[length - 1] == '\n' || line[length - 1] == '\r'))
        line[--length] = '\0';
    return line;
}

static int split_columns(char *line, char **columns, int capacity)
{
    int count = 0;
    char *cursor = line;

    while (count < capacity)
    {
        columns[count++] = cursor;
        char *comma = strchr(cursor, ',');
        if (comma == NULL)
            break;
        *comma = '\0';
        cursor = comma + 1;
    }

    return count;
}

static int find_column(char **columns, int count, const char *name)
{
    for (int i = 0; i < count; i++)
    {
        if (strcmp(columns[i], name) == 0)
            return i;
    }
    return -1;
}

static int parse_axis(const char *text, int16_t *value, size_t row, const char *axis)
{
    errno = 0;
    char *end = NULL;
    long parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed < -2048 || parsed > 2047)
    {
        fprintf(stderr,
                "row %zu: %s value '%s' is not an ADXL362 12-bit value (-2048..2047)\n",
                row,
                axis,
                text);
        return 0;
    }

    *value = (int16_t)parsed;
    return 1;
}

static int append_sample(csv_sample_t **samples,
                         size_t *count,
                         size_t *capacity,
                         const csv_sample_t *sample)
{
    if (*count == *capacity)
    {
        size_t next_capacity = *capacity == 0 ? 1024 : *capacity * 2;
        csv_sample_t *next = realloc(*samples, next_capacity * sizeof(**samples));
        if (next == NULL)
        {
            fprintf(stderr, "failed to allocate CSV sample buffer\n");
            return 0;
        }
        *samples = next;
        *capacity = next_capacity;
    }

    (*samples)[(*count)++] = *sample;
    return 1;
}

static int read_csv(const char *path,
                    char **header,
                    csv_sample_t **samples,
                    size_t *sample_count)
{
    FILE *input = fopen(path, "rb");
    if (input == NULL)
    {
        fprintf(stderr, "cannot open '%s': %s\n", path, strerror(errno));
        return 0;
    }

    char *line = NULL;
    size_t line_capacity = 0;
    ssize_t line_length = getline(&line, &line_capacity, input);
    if (line_length < 0)
    {
        fprintf(stderr, "'%s' has no header row\n", path);
        fclose(input);
        free(line);
        return 0;
    }

    trim_line_end(line);
    *header = strdup(line);
    char *header_copy = strdup(line);
    if (*header == NULL || header_copy == NULL)
    {
        fprintf(stderr, "failed to allocate CSV header\n");
        fclose(input);
        free(line);
        free(*header);
        free(header_copy);
        return 0;
    }

    char *header_columns[MAX_COLUMNS];
    int header_count = split_columns(header_copy, header_columns, MAX_COLUMNS);
    int x_column = find_column(header_columns, header_count, "x");
    int y_column = find_column(header_columns, header_count, "y");
    int z_column = find_column(header_columns, header_count, "z");
    if (x_column < 0 || y_column < 0 || z_column < 0)
    {
        fprintf(stderr, "'%s' must contain x,y,z columns\n", path);
        fclose(input);
        free(line);
        free(*header);
        free(header_copy);
        return 0;
    }

    size_t count = 0;
    size_t capacity = 0;
    size_t row = 1;
    while ((line_length = getline(&line, &line_capacity, input)) >= 0)
    {
        row++;
        trim_line_end(line);
        char *original_line = strdup(line);
        char *parse_line = strdup(line);
        if (original_line == NULL || parse_line == NULL)
        {
            fprintf(stderr, "failed to allocate CSV row\n");
            free(original_line);
            free(parse_line);
            goto fail;
        }

        char *columns[MAX_COLUMNS];
        int column_count = split_columns(parse_line, columns, MAX_COLUMNS);
        int required_column = x_column;
        if (y_column > required_column)
            required_column = y_column;
        if (z_column > required_column)
            required_column = z_column;
        if (column_count <= required_column)
        {
            fprintf(stderr, "row %zu: not enough columns\n", row);
            free(original_line);
            free(parse_line);
            goto fail;
        }

        csv_sample_t sample = {.line = original_line};
        int valid = parse_axis(columns[x_column], &sample.x, row, "x") &&
                    parse_axis(columns[y_column], &sample.y, row, "y") &&
                    parse_axis(columns[z_column], &sample.z, row, "z");
        free(parse_line);
        if (!valid || !append_sample(samples, &count, &capacity, &sample))
        {
            free(original_line);
            goto fail;
        }
    }

    free(header_copy);
    free(line);
    fclose(input);
    *sample_count = count;
    return 1;

fail:
    for (size_t i = 0; i < count; i++)
        free((*samples)[i].line);
    free(*samples);
    *samples = NULL;
    free(*header);
    *header = NULL;
    free(header_copy);
    free(line);
    fclose(input);
    return 0;
}

static void encode_fifo_word(int16_t value, uint8_t axis, uint8_t *output)
{
    uint16_t raw = (uint16_t)value & 0x0FFFu;
    output[0] = (uint8_t)(raw & 0xFFu);
    output[1] = (uint8_t)(((raw >> 8) & UINT16_C(0x000F)) |
                          ((uint16_t)axis << UINT16_C(6)));
}

static void load_fifo(const csv_sample_t *samples)
{
    memset(fifo, 0, _FIFO_LEN);
    for (size_t i = 0; i < SAMPLES_PER_FIFO; i++)
    {
        size_t offset = i * 6;
        encode_fifo_word(samples[i].x, 0, &fifo[offset]);
        encode_fifo_word(samples[i].y, 1, &fifo[offset + 2]);
        encode_fifo_word(samples[i].z, 2, &fifo[offset + 4]);
    }
}

static int write_output(const char *path,
                        const char *header,
                        const csv_sample_t *samples,
                        const uint8_t *behaviors,
                        size_t count)
{
    FILE *output = fopen(path, "wb");
    if (output == NULL)
    {
        fprintf(stderr, "cannot create '%s': %s\n", path, strerror(errno));
        return 0;
    }

    fprintf(output, "%s,behavior\n", header);
    for (size_t i = 0; i < count; i++)
        fprintf(output, "%s,%u\n", samples[i].line, (unsigned)behaviors[i]);

    if (fclose(output) != 0)
    {
        fprintf(stderr, "failed to close '%s': %s\n", path, strerror(errno));
        return 0;
    }
    return 1;
}

static const char *action_label(uint8_t action)
{
    static const char *labels[] = {
        "initial_delay", "rest", "ingestion", "movement",
        "climb", "ruminate", "other", "breath"};
    return action < sizeof(labels) / sizeof(labels[0]) ? labels[action] : "unknown";
}

int main(int argc, char **argv)
{
    int argument = 1;
    if (argument < argc && strcmp(argv[argument], "--verbose") == 0)
    {
        pc_verbose = 1;
        argument++;
    }
    if (argument >= argc || argument + 2 < argc)
    {
        print_usage(argv[0]);
        return 2;
    }

    const char *input_path = argv[argument++];
    const char *output_path = argument < argc ? argv[argument] : "adxl362_pc_output.csv";

    char *header = NULL;
    csv_sample_t *samples = NULL;
    size_t input_count = 0;
    if (!read_csv(input_path, &header, &samples, &input_count))
        return 1;

    size_t output_count = (input_count / SAMPLES_PER_FIFO) * SAMPLES_PER_FIFO;
    uint8_t *behaviors = calloc(output_count, sizeof(*behaviors));
    if (behaviors == NULL && output_count != 0)
    {
        fprintf(stderr, "failed to allocate behavior output buffer\n");
        for (size_t i = 0; i < input_count; i++)
            free(samples[i].line);
        free(samples);
        free(header);
        return 1;
    }

    uint32_t counts[8] = {0};
    for (size_t start = 0; start < output_count; start += SAMPLES_PER_FIFO)
    {
        load_fifo(&samples[start]);
        ADXL362FifoProcess();

        for (size_t second = 0; second < FIFO_OUTPUT_COUNT; second++)
        {
            uint8_t behavior = action_classify_array[second];
            for (size_t sample = 0; sample < SAMPLE_RATE_HZ; sample++)
            {
                size_t output_index = start + second * SAMPLE_RATE_HZ + sample;
                behaviors[output_index] = behavior;
                if (behavior < sizeof(counts) / sizeof(counts[0]))
                    counts[behavior]++;
            }
        }
    }

    int success = write_output(output_path, header, samples, behaviors, output_count);
    if (success)
    {
        printf("input_rows=%zu\n", input_count);
        printf("output_rows=%zu\n", output_count);
        printf("discarded_tail_rows=%zu\n", input_count - output_count);
        printf("output=%s\n", output_path);
        printf("behavior_counts:\n");
        for (uint8_t i = 0; i < 8; i++)
            printf("  %u (%s): %u\n", i, action_label(i), counts[i]);
    }

    for (size_t i = 0; i < input_count; i++)
        free(samples[i].line);
    free(samples);
    free(behaviors);
    free(header);
    return success ? 0 : 1;
}
