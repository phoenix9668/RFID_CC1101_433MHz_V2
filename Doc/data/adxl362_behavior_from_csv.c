#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _FIFO_SAMPLES_LEN 900
#define _AXIS_LEN 170
#define _DIFF_CNT 2
#define _MEM_ROWS 18
#define _MEM_COLS 4

#define SAMPLE_RATE_HZ 25
#define SAMPLES_PER_FIFO (_FIFO_SAMPLES_LEN / 6)
#define SECONDS_PER_FIFO 6

#define SINE_WAVE_MIN_FREQ 1.0f
#define SINE_WAVE_MAX_FREQ 1.7f
#define SINE_WAVE_MIN_AMPLITUDE 30
#define PLATEAU_THRESHOLD 5
#define MIN_PLATEAU_COUNT 3
#define MAX_PLATEAU_WINDOW 3
#define MAX_PEAK_COUNT 20
#define MAX_VALLEY_COUNT 20
#define MAX_PERIOD_COUNT 11
#define STD_DEV_THRESHOLD 0.3f
#define SMOOTH_WINDOW_SIZE 5
#define EXTREMA_WINDOW_SIZE 3

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} axis_info_int16_t;

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} axis_info_int32_t;

typedef struct
{
    uint16_t low;
    uint16_t normal;
    uint16_t abovenormal;
    uint16_t high;
} threshold_judge_t;

typedef struct
{
    uint16_t rest;
    uint16_t ingestion;
    uint16_t movement;
    uint16_t climb;
    uint16_t ruminate;
    uint16_t other;
} action_classify_t;

typedef struct
{
    int16_t value;
    uint16_t index;
} extreme_point_t;

typedef struct
{
    extreme_point_t peaks[MAX_PEAK_COUNT];
    extreme_point_t valleys[MAX_VALLEY_COUNT];
    uint16_t peak_count;
    uint16_t valley_count;
    uint16_t periods[MAX_PERIOD_COUNT];
    uint8_t period_count;
    uint8_t is_rising;
    uint8_t is_sine_wave;
    float frequency;
    int16_t amplitude;
} sine_wave_detection_t;

typedef struct
{
    int packet;
    int x;
    int y;
    int z;
} csv_sample_t;

static axis_info_int16_t three_axis_info[_AXIS_LEN];
static axis_info_int16_t diff_three_axis_info[_DIFF_CNT];
static axis_info_int32_t three_axis_average_info;
static axis_info_int32_t sum_info;
static threshold_judge_t threshold_judge;
static action_classify_t action_classify;
static uint8_t action_classify_array[SECONDS_PER_FIFO];
static uint8_t action;

static int32_t memory_array[_MEM_ROWS][_MEM_COLS];
static uint8_t memory_index = 0;
static uint8_t memory_index_o;
static uint8_t mid_index = 0;

static uint8_t movement_cnt = 0;
static uint8_t climb_cnt = 0;
static uint8_t rest_cnt1 = 0;
static uint8_t rest_cnt2 = 0;
static uint8_t ingestion_cnt = 0;
static uint8_t rest_cnt = 0;
static uint8_t deta_a_cnt = 0;
static uint8_t jicha_cnt = 0;
static int16_t eighteen_average = 0;
static uint16_t sum_eighteen_average = 0;

static int16_t x_max_val = -2000;
static int16_t x_min_val = 2000;
static int16_t y_max_val = -2000;
static int16_t y_min_val = 2000;
static int16_t z_max_val = -2000;
static int16_t z_min_val = 2000;

static sine_wave_detection_t sine_detection;
static int16_t smoothed_x[_AXIS_LEN];

static const char *action_label(uint8_t value)
{
    switch (value)
    {
    case 0:
        return "initial_delay";
    case 1:
        return "rest";
    case 2:
        return "ingestion";
    case 3:
        return "movement";
    case 4:
        return "climb";
    case 5:
        return "ruminate";
    case 6:
        return "other";
    case 7:
        return "breath";
    default:
        return "unknown";
    }
}

static void sort_extreme_points(extreme_point_t *points, uint16_t count)
{
    for (uint16_t i = 0; i < count - 1; i++)
    {
        for (uint16_t j = 0; j < count - i - 1; j++)
        {
            if (points[j].index > points[j + 1].index)
            {
                extreme_point_t temp = points[j];
                points[j] = points[j + 1];
                points[j + 1] = temp;
            }
        }
    }
}

static void detect_sine_wave(void)
{
    memset(&sine_detection, 0, sizeof(sine_detection));
    memset(smoothed_x, 0, sizeof(smoothed_x));
    x_max_val = -2000;
    x_min_val = 2000;
    y_max_val = -2000;
    y_min_val = 2000;
    z_max_val = -2000;
    z_min_val = 2000;

    for (uint16_t i = 0; i < SAMPLES_PER_FIFO; i++)
    {
        int16_t current_x = three_axis_info[i].x;
        int16_t current_y = three_axis_info[i].y;
        if (current_x > x_max_val)
            x_max_val = current_x;
        if (current_x < x_min_val)
            x_min_val = current_x;

        if (current_y > y_max_val)
            y_max_val = current_y;
        if (current_y < y_min_val)
            y_min_val = current_y;
    }

    if (abs(x_max_val - x_min_val) > 2 * abs(y_max_val - y_min_val))
    {
        for (uint16_t i = 0; i < SAMPLES_PER_FIFO; i++)
        {
            int32_t sum = 0;
            int16_t count = 0;

            for (int16_t j = -SMOOTH_WINDOW_SIZE / 2; j <= SMOOTH_WINDOW_SIZE / 2; j++)
            {
                int16_t idx = (int16_t)i + j;
                if (idx >= 0 && idx < SAMPLES_PER_FIFO)
                {
                    sum += three_axis_info[idx].x;
                    count++;
                }
            }

            smoothed_x[i] = (int16_t)(sum / count);
        }

        for (uint16_t i = 0; i < SAMPLES_PER_FIFO; i++)
        {
            if (i == 0)
            {
                continue;
            }

            if (i >= MAX_PLATEAU_WINDOW)
            {
                uint8_t is_plateau = 1;
                int16_t plateau_value_sum = 0;
                uint8_t plateau_count = 0;

                for (int16_t j = -MAX_PLATEAU_WINDOW; j < 0; j++)
                {
                    if (abs(smoothed_x[i + j] - smoothed_x[i + j + 1]) > PLATEAU_THRESHOLD)
                    {
                        is_plateau = 0;
                        break;
                    }
                    plateau_value_sum += smoothed_x[i + j];
                    plateau_count++;
                }

                if (is_plateau && plateau_count >= MIN_PLATEAU_COUNT)
                {
                    int16_t plateau_value = plateau_value_sum / plateau_count;
                    int16_t before_plateau = 0;
                    int16_t after_plateau = 0;

                    if (i > MAX_PLATEAU_WINDOW + 2)
                    {
                        before_plateau = smoothed_x[i - MAX_PLATEAU_WINDOW - 2];
                    }

                    after_plateau = smoothed_x[i];

                    if (before_plateau < plateau_value && after_plateau < plateau_value)
                    {
                        if (sine_detection.is_rising && sine_detection.peak_count < MAX_PEAK_COUNT)
                        {
                            sine_detection.peaks[sine_detection.peak_count].value = plateau_value;
                            sine_detection.peaks[sine_detection.peak_count].index =
                                i - MAX_PLATEAU_WINDOW / 2;

                            if (sine_detection.peak_count >= 1 &&
                                sine_detection.period_count < MAX_PERIOD_COUNT)
                            {
                                sine_detection.periods[sine_detection.period_count] =
                                    sine_detection.peaks[sine_detection.peak_count].index -
                                    sine_detection.peaks[sine_detection.peak_count - 1].index;
                                sine_detection.period_count++;
                            }
                            sine_detection.peak_count++;
                            sine_detection.is_rising = 0;
                        }
                    }
                    else if (before_plateau > plateau_value && after_plateau > plateau_value)
                    {
                        if (!sine_detection.is_rising &&
                            sine_detection.valley_count < MAX_VALLEY_COUNT)
                        {
                            sine_detection.valleys[sine_detection.valley_count].value = plateau_value;
                            sine_detection.valleys[sine_detection.valley_count].index =
                                i - MAX_PLATEAU_WINDOW / 2;
                            sine_detection.valley_count++;
                            sine_detection.is_rising = 1;
                        }
                    }
                }
            }

            if (i >= EXTREMA_WINDOW_SIZE && i < SAMPLES_PER_FIFO - EXTREMA_WINDOW_SIZE)
            {
                uint8_t is_peak = 1;
                for (int16_t j = -EXTREMA_WINDOW_SIZE; j <= EXTREMA_WINDOW_SIZE; j++)
                {
                    if (j != 0 && smoothed_x[i + j] > smoothed_x[i])
                    {
                        is_peak = 0;
                        break;
                    }
                }

                if (is_peak && sine_detection.is_rising &&
                    sine_detection.peak_count < MAX_PEAK_COUNT)
                {
                    uint8_t too_close = 0;
                    for (uint16_t j = 0; j < sine_detection.peak_count; j++)
                    {
                        if (abs((int)i - (int)sine_detection.peaks[j].index) <
                            EXTREMA_WINDOW_SIZE)
                        {
                            too_close = 1;
                            if (smoothed_x[i] > sine_detection.peaks[j].value)
                            {
                                sine_detection.peaks[j].value = smoothed_x[i];
                                sine_detection.peaks[j].index = i;
                            }
                            break;
                        }
                    }

                    if (!too_close)
                    {
                        sine_detection.peaks[sine_detection.peak_count].value = smoothed_x[i];
                        sine_detection.peaks[sine_detection.peak_count].index = i;

                        if (sine_detection.peak_count >= 1 &&
                            sine_detection.period_count < MAX_PERIOD_COUNT)
                        {
                            sine_detection.periods[sine_detection.period_count] =
                                sine_detection.peaks[sine_detection.peak_count].index -
                                sine_detection.peaks[sine_detection.peak_count - 1].index;
                            sine_detection.period_count++;
                        }
                        sine_detection.peak_count++;
                    }
                    sine_detection.is_rising = 0;
                }

                uint8_t is_valley = 1;
                for (int16_t j = -EXTREMA_WINDOW_SIZE; j <= EXTREMA_WINDOW_SIZE; j++)
                {
                    if (j != 0 && smoothed_x[i + j] < smoothed_x[i])
                    {
                        is_valley = 0;
                        break;
                    }
                }

                if (is_valley && !sine_detection.is_rising &&
                    sine_detection.valley_count < MAX_VALLEY_COUNT)
                {
                    uint8_t too_close = 0;
                    for (uint16_t j = 0; j < sine_detection.valley_count; j++)
                    {
                        if (abs((int)i - (int)sine_detection.valleys[j].index) <
                            EXTREMA_WINDOW_SIZE)
                        {
                            too_close = 1;
                            if (smoothed_x[i] < sine_detection.valleys[j].value)
                            {
                                sine_detection.valleys[j].value = smoothed_x[i];
                                sine_detection.valleys[j].index = i;
                            }
                            break;
                        }
                    }

                    if (!too_close)
                    {
                        sine_detection.valleys[sine_detection.valley_count].value = smoothed_x[i];
                        sine_detection.valleys[sine_detection.valley_count].index = i;
                        sine_detection.valley_count++;
                    }
                    sine_detection.is_rising = 1;
                }
            }
        }

        sort_extreme_points(sine_detection.peaks, sine_detection.peak_count);
        sort_extreme_points(sine_detection.valleys, sine_detection.valley_count);

        sine_detection.period_count = 0;
        for (uint16_t i = 1; i < sine_detection.peak_count &&
                             sine_detection.period_count < MAX_PERIOD_COUNT;
             i++)
        {
            sine_detection.periods[sine_detection.period_count] =
                sine_detection.peaks[i].index - sine_detection.peaks[i - 1].index;
            sine_detection.period_count++;
        }

        if (sine_detection.peak_count >= 3 && sine_detection.valley_count >= 3)
        {
            float avg_period = 0;
            for (uint8_t j = 0; j < sine_detection.period_count; j++)
            {
                avg_period += sine_detection.periods[j];
            }
            avg_period /= sine_detection.period_count;

            sine_detection.frequency = 25.0f / avg_period;

            int32_t peak_sum = 0, valley_sum = 0;
            for (uint8_t j = 0; j < sine_detection.peak_count; j++)
            {
                peak_sum += sine_detection.peaks[j].value;
            }
            for (uint8_t j = 0; j < sine_detection.valley_count; j++)
            {
                valley_sum += sine_detection.valleys[j].value;
            }
            int16_t avg_peak = peak_sum / sine_detection.peak_count;
            int16_t avg_valley = valley_sum / sine_detection.valley_count;
            sine_detection.amplitude = avg_peak - avg_valley;

            if (sine_detection.frequency >= SINE_WAVE_MIN_FREQ &&
                sine_detection.frequency <= SINE_WAVE_MAX_FREQ &&
                sine_detection.amplitude > SINE_WAVE_MIN_AMPLITUDE)
            {
                float peak_std = 0, valley_std = 0;
                for (uint8_t j = 0; j < sine_detection.peak_count; j++)
                {
                    peak_std += (sine_detection.peaks[j].value - avg_peak) *
                                (sine_detection.peaks[j].value - avg_peak);
                }
                for (uint8_t j = 0; j < sine_detection.valley_count; j++)
                {
                    valley_std += (sine_detection.valleys[j].value - avg_valley) *
                                  (sine_detection.valleys[j].value - avg_valley);
                }
                peak_std = sqrtf(peak_std / sine_detection.peak_count);
                valley_std = sqrtf(valley_std / sine_detection.valley_count);

                if (peak_std < sine_detection.amplitude * STD_DEV_THRESHOLD &&
                    valley_std < sine_detection.amplitude * STD_DEV_THRESHOLD)
                {
                    sine_detection.is_sine_wave = 1;
                }
            }
        }
    }
}

static void classify_fifo_samples(void)
{
    detect_sine_wave();

    x_max_val = -2000;
    x_min_val = 2000;
    y_max_val = -2000;
    y_min_val = 2000;
    z_max_val = -2000;
    z_min_val = 2000;

    for (uint16_t i = 0; i < SAMPLES_PER_FIFO; i++)
    {
        diff_three_axis_info[1] = diff_three_axis_info[0];
        diff_three_axis_info[0] = three_axis_info[i];

        if (three_axis_info[i].x > x_max_val)
            x_max_val = three_axis_info[i].x;

        if (three_axis_info[i].x < x_min_val)
            x_min_val = three_axis_info[i].x;

        if (three_axis_info[i].y > y_max_val)
            y_max_val = three_axis_info[i].y;

        if (three_axis_info[i].y < y_min_val)
            y_min_val = three_axis_info[i].y;

        if (three_axis_info[i].z > z_max_val)
            z_max_val = three_axis_info[i].z;

        if (three_axis_info[i].z < z_min_val)
            z_min_val = three_axis_info[i].z;

        three_axis_average_info.x += three_axis_info[i].x;

        three_axis_info[i].x = diff_three_axis_info[0].x - diff_three_axis_info[1].x;
        three_axis_info[i].y = diff_three_axis_info[0].y - diff_three_axis_info[1].y;
        three_axis_info[i].z = diff_three_axis_info[0].z - diff_three_axis_info[1].z;

        sum_info.x += abs(three_axis_info[i].x);
        sum_info.y += abs(three_axis_info[i].y);
        sum_info.z += abs(three_axis_info[i].z);

        if (abs(three_axis_info[i].x) <= 10)
            threshold_judge.low++;
        else if (abs(three_axis_info[i].x) <= 100)
            threshold_judge.normal++;
        else if (abs(three_axis_info[i].x) <= 200)
            threshold_judge.abovenormal++;
        else
            threshold_judge.high++;

        if (((i + 1) / 25) != 0 && ((i + 1) % 25) == 0)
        {
            three_axis_average_info.x = three_axis_average_info.x / 25;

            if (sine_detection.is_sine_wave == 1)
                action = 7;
            else if (threshold_judge.low >= 24)
                action = 1;
            else if ((threshold_judge.normal + threshold_judge.abovenormal) > 11 &&
                     threshold_judge.high == 0 &&
                     three_axis_average_info.x >= 200)
                action = 2;
            else if (three_axis_average_info.x > -200 && three_axis_average_info.x < 100 &&
                     (sum_info.x + sum_info.y + sum_info.z) / 3 > 400 &&
                     (((x_max_val - x_min_val) + (y_max_val - y_min_val) +
                       (z_max_val - z_min_val)) /
                          3) > 150)
                action = 3;
            else if (threshold_judge.high > 0 && three_axis_average_info.x <= -200)
                action = 4;
            else
                action = 6;

            memory_array[memory_index][0] = action;
            memory_array[memory_index][1] = three_axis_average_info.x;
            memory_array[memory_index][2] = (sum_info.x + sum_info.y + sum_info.z) / 3;
            memory_array[memory_index][3] =
                ((x_max_val - x_min_val) + (y_max_val - y_min_val) +
                 (z_max_val - z_min_val)) /
                3;

            movement_cnt = 0;
            rest_cnt1 = 0;
            rest_cnt2 = 0;

            if (memory_index >= 9)
                mid_index = memory_index - 9;
            else
                mid_index = memory_index + 9;

            if (memory_array[mid_index][0] == 3)
            {
                for (uint8_t j = 0; j < _MEM_ROWS; j++)
                    if (memory_array[j][0] == 3)
                        movement_cnt += 1;

                if (movement_cnt == 1)
                {
                    if (mid_index < memory_index)
                    {
                        for (uint8_t j = mid_index; j < memory_index; j++)
                            if (memory_array[j][0] == 1)
                                rest_cnt1 += 1;

                        for (uint8_t j = 0; j < mid_index; j++)
                            if (memory_array[j][0] == 1)
                                rest_cnt2 += 1;

                        for (uint8_t j = memory_index; j < _MEM_ROWS; j++)
                            if (memory_array[j][0] == 1)
                                rest_cnt2 += 1;
                    }
                    else if (memory_index < mid_index)
                    {
                        for (uint8_t j = memory_index; j < mid_index; j++)
                            if (memory_array[j][0] == 1)
                                rest_cnt2 += 1;

                        for (uint8_t j = 0; j < memory_index; j++)
                            if (memory_array[j][0] == 1)
                                rest_cnt1 += 1;

                        for (uint8_t j = mid_index; j < _MEM_ROWS; j++)
                            if (memory_array[j][0] == 1)
                                rest_cnt1 += 1;
                    }

                    if (rest_cnt1 >= 4 && rest_cnt2 >= 4)
                        memory_array[mid_index][0] = 1;
                }
            }

            movement_cnt = 0;
            climb_cnt = 0;

            if (memory_array[memory_index][0] == 4)
            {
                for (uint8_t j = 0; j < _MEM_ROWS; j++)
                {
                    if (memory_array[j][0] == 3)
                        movement_cnt += 1;

                    if (memory_array[j][0] == 4)
                        climb_cnt += 1;
                }

                if (movement_cnt < 4)
                    memory_array[memory_index][0] = 6;
                else if (climb_cnt >= 2)
                    memory_array[memory_index][0] = 3;
            }

            if (memory_index >= 17)
                memory_index = 0;
            else
                memory_index += 1;

            ingestion_cnt = 0;

            for (uint8_t j = 0; j < _MEM_ROWS; j++)
                if (memory_array[j][0] == 2)
                    ingestion_cnt += 1;

            if (ingestion_cnt >= 2)
                for (uint8_t j = 0; j < _MEM_ROWS; j++)
                    if (memory_array[j][0] == 3)
                        memory_array[j][0] = 6;

            rest_cnt = 0;
            deta_a_cnt = 0;
            jicha_cnt = 0;
            eighteen_average = 0;
            sum_eighteen_average = 0;

            for (uint8_t j = 0; j < _MEM_ROWS; j++)
                if (memory_array[j][0] == 1)
                    rest_cnt += 1;

            if (rest_cnt <= 4 && sine_detection.is_sine_wave == 0)
            {
                for (uint8_t j = 0; j < _MEM_ROWS; j++)
                {
                    if (memory_array[j][2] > 130 && memory_array[j][2] < 700)
                        deta_a_cnt += 1;

                    if (memory_array[j][3] < 120)
                        jicha_cnt += 1;
                }

                if (deta_a_cnt >= 14 && jicha_cnt >= 14)
                {
                    for (uint8_t j = 0; j < _MEM_ROWS; j++)
                        eighteen_average += memory_array[j][1];

                    eighteen_average = eighteen_average / _MEM_ROWS;

                    for (uint8_t j = 0; j < _MEM_ROWS; j++)
                        sum_eighteen_average += abs(eighteen_average - memory_array[j][1]);

                    if (sum_eighteen_average <= 400 && eighteen_average < 150)
                        for (uint8_t j = 0; j < _MEM_ROWS; j++)
                            memory_array[j][0] = 5;
                }
            }

            if (memory_index == 17)
                memory_index_o = 0;
            else
                memory_index_o = memory_index + 1;

            action_classify_array[((i + 1) / 25) - 1] = memory_array[memory_index_o][0];

            if (memory_array[memory_index_o][0] == 7)
                action_classify.other++;
            else if (memory_array[memory_index_o][0] == 1)
                action_classify.rest++;
            else if (memory_array[memory_index_o][0] == 2)
                action_classify.ingestion++;
            else if (memory_array[memory_index_o][0] == 3)
                action_classify.movement++;
            else if (memory_array[memory_index_o][0] == 4)
                action_classify.climb++;
            else if (memory_array[memory_index_o][0] == 5)
                action_classify.ruminate++;

            memset(&threshold_judge, 0, sizeof(threshold_judge));
            memset(&three_axis_average_info, 0, sizeof(three_axis_average_info));
            memset(&sum_info, 0, sizeof(sum_info));
            x_max_val = -2000;
            x_min_val = 2000;
            y_max_val = -2000;
            y_min_val = 2000;
            z_max_val = -2000;
            z_min_val = 2000;
        }
    }
}

static int parse_sample_line(char *line, csv_sample_t *sample)
{
    char *token = strtok(line, ",\r\n");
    if (token == NULL)
        return 0;
    sample->packet = (int)strtol(token, NULL, 10);

    token = strtok(NULL, ",\r\n");
    if (token == NULL)
        return 0;
    sample->x = (int)strtol(token, NULL, 10);

    token = strtok(NULL, ",\r\n");
    if (token == NULL)
        return 0;
    sample->y = (int)strtol(token, NULL, 10);

    token = strtok(NULL, ",\r\n");
    if (token == NULL)
        return 0;
    sample->z = (int)strtol(token, NULL, 10);

    return 1;
}

static int read_csv(const char *path, csv_sample_t **samples_out, size_t *count_out)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL)
    {
        fprintf(stderr, "failed to open input %s: %s\n", path, strerror(errno));
        return 0;
    }

    size_t capacity = 2048;
    size_t count = 0;
    csv_sample_t *samples = (csv_sample_t *)malloc(capacity * sizeof(csv_sample_t));
    if (samples == NULL)
    {
        fprintf(stderr, "failed to allocate sample buffer\n");
        fclose(file);
        return 0;
    }

    char line[512];
    if (fgets(line, sizeof(line), file) == NULL)
    {
        fprintf(stderr, "input is empty: %s\n", path);
        free(samples);
        fclose(file);
        return 0;
    }

    while (fgets(line, sizeof(line), file) != NULL)
    {
        if (count == capacity)
        {
            capacity *= 2;
            csv_sample_t *next =
                (csv_sample_t *)realloc(samples, capacity * sizeof(csv_sample_t));
            if (next == NULL)
            {
                fprintf(stderr, "failed to grow sample buffer\n");
                free(samples);
                fclose(file);
                return 0;
            }
            samples = next;
        }

        if (!parse_sample_line(line, &samples[count]))
        {
            fprintf(stderr, "failed to parse input row %zu\n", count + 2);
            free(samples);
            fclose(file);
            return 0;
        }
        count++;
    }

    fclose(file);
    *samples_out = samples;
    *count_out = count;
    return 1;
}

static int write_output_csv(
    const char *path,
    const csv_sample_t *samples,
    const uint8_t *behaviors,
    size_t count)
{
    FILE *file = fopen(path, "wb");
    if (file == NULL)
    {
        fprintf(stderr, "failed to open output %s: %s\n", path, strerror(errno));
        return 0;
    }

    fprintf(file, "\xB0\xFC\xCD\xB7,x,y,z,behavior\n");
    for (size_t i = 0; i < count; i++)
    {
        fprintf(
            file,
            "%d,%d,%d,%d,%u\n",
            samples[i].packet,
            samples[i].x,
            samples[i].y,
            samples[i].z,
            (unsigned)behaviors[i]);
    }

    fclose(file);
    return 1;
}

int main(int argc, char **argv)
{
    const char *input_path = "chuanxi.csv";
    const char *output_path = "chuanxi_c_with_behavior.csv";
    if (argc >= 2)
        input_path = argv[1];
    if (argc >= 3)
        output_path = argv[2];

    csv_sample_t *samples = NULL;
    size_t input_count = 0;
    if (!read_csv(input_path, &samples, &input_count))
        return 1;

    size_t output_count = (input_count / SAMPLES_PER_FIFO) * SAMPLES_PER_FIFO;
    uint8_t *behaviors = (uint8_t *)calloc(output_count, sizeof(uint8_t));
    if (behaviors == NULL)
    {
        fprintf(stderr, "failed to allocate behavior buffer\n");
        free(samples);
        return 1;
    }

    uint32_t behavior_counts[8] = {0};
    size_t behavior_index = 0;

    memset(memory_array, 0, sizeof(memory_array));
    memset(diff_three_axis_info, 0, sizeof(diff_three_axis_info));
    memset(&action_classify, 0, sizeof(action_classify));

    for (size_t start = 0; start < output_count; start += SAMPLES_PER_FIFO)
    {
        memset(three_axis_info, 0, sizeof(three_axis_info));
        memset(action_classify_array, 0, sizeof(action_classify_array));

        for (uint16_t i = 0; i < SAMPLES_PER_FIFO; i++)
        {
            three_axis_info[i].x = (int16_t)samples[start + i].x;
            three_axis_info[i].y = (int16_t)samples[start + i].y;
            three_axis_info[i].z = (int16_t)samples[start + i].z;
        }

        classify_fifo_samples();

        for (uint8_t second = 0; second < SECONDS_PER_FIFO; second++)
        {
            uint8_t behavior = action_classify_array[second];
            for (uint8_t sample = 0; sample < SAMPLE_RATE_HZ; sample++)
            {
                behaviors[behavior_index++] = behavior;
                if (behavior < 8)
                    behavior_counts[behavior]++;
            }
        }
    }

    if (!write_output_csv(output_path, samples, behaviors, output_count))
    {
        free(behaviors);
        free(samples);
        return 1;
    }

    printf("input_rows=%zu\n", input_count);
    printf("output_rows=%zu\n", output_count);
    printf("discarded_tail_rows=%zu\n", input_count - output_count);
    printf("output=%s\n", output_path);
    printf("behavior_counts:\n");
    for (uint8_t i = 0; i < 8; i++)
    {
        printf("  %u (%s): %lu\n", (unsigned)i, action_label(i), (unsigned long)behavior_counts[i]);
    }

    free(behaviors);
    free(samples);
    return 0;
}
