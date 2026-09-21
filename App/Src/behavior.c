/* Mechanically extracted from 34d57c2. Thresholds and integer arithmetic preserved. */
#include "behavior.h"
#include <stdlib.h>
#include <string.h>
#define _DIFF_CNT 2
#define _MEM_ROWS 18
#define _MEM_COLS 4
typedef accel_sample_t axis_info_int16_t;
typedef struct
{
    int32_t x, y, z;
} axis_info_int32_t;
typedef struct
{
    uint16_t low, normal, abovenormal, high;
} threshold_judge_t;
static axis_info_int16_t diff_three_axis_info[_DIFF_CNT];
static axis_info_int32_t three_axis_average_info;
static axis_info_int32_t sum_info;
static threshold_judge_t threshold_judge;
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

static uint8_t pending_samples;
bool behavior_push(accel_sample_t sample, uint8_t *result)
{

    diff_three_axis_info[1] = diff_three_axis_info[0];
    diff_three_axis_info[0] = sample;

    if (sample.x > x_max_val)
        x_max_val = sample.x;

    if (sample.x < x_min_val)
        x_min_val = sample.x;

    if (sample.y > y_max_val)
        y_max_val = sample.y;

    if (sample.y < y_min_val)
        y_min_val = sample.y;

    if (sample.z > z_max_val)
        z_max_val = sample.z;

    if (sample.z < z_min_val)
        z_min_val = sample.z;

    three_axis_average_info.x += sample.x;

    sample.x = diff_three_axis_info[0].x - diff_three_axis_info[1].x;
    sample.y = diff_three_axis_info[0].y - diff_three_axis_info[1].y;
    sample.z = diff_three_axis_info[0].z - diff_three_axis_info[1].z;

    sum_info.x += abs(sample.x);
    sum_info.y += abs(sample.y);
    sum_info.z += abs(sample.z);

    if (abs(sample.x) <= 10)
        threshold_judge.low++;
    else if (abs(sample.x) <= 100)
        threshold_judge.normal++;
    else if (abs(sample.x) <= 200)
        threshold_judge.abovenormal++;
    else
        threshold_judge.high++;

    if (++pending_samples == 25)
    {

        three_axis_average_info.x = three_axis_average_info.x / 25;

        if (threshold_judge.low >= 24)
            action = 1; // rest
        else if ((threshold_judge.normal + threshold_judge.abovenormal) > 11 &&
                 threshold_judge.high == 0 && three_axis_average_info.x >= 200)
            action = 2; // ingestion
        else if (three_axis_average_info.x > -200 && three_axis_average_info.x < 100 &&
                 (sum_info.x + sum_info.y + sum_info.z) / 3 > 400 &&
                 (((x_max_val - x_min_val) + (y_max_val - y_min_val) + (z_max_val - z_min_val)) /
                  3) > 150)
            action = 3; // movement
        else if (threshold_judge.high > 0 && three_axis_average_info.x <= -200)
            action = 4; // climb
        else
            action = 6; // other

        memory_array[memory_index][0] = action;
        memory_array[memory_index][1] = three_axis_average_info.x;
        memory_array[memory_index][2] = (sum_info.x + sum_info.y + sum_info.z) / 3;
        memory_array[memory_index][3] =
            ((x_max_val - x_min_val) + (y_max_val - y_min_val) + (z_max_val - z_min_val)) / 3;

        movement_cnt = 0;
        rest_cnt1 = 0;
        rest_cnt2 = 0;

        if (memory_index >= 9)
            mid_index = memory_index - 9;
        else
            mid_index = memory_index + 9;

        if (memory_array[mid_index][0] == 3)
        {
            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if (memory_array[i][0] == 3)
                    movement_cnt += 1;

            if (movement_cnt == 1)
            {
                if (mid_index < memory_index)
                {
                    for (uint8_t i = mid_index; i < memory_index; i++)
                        if (memory_array[i][0] == 1)
                            rest_cnt1 += 1;

                    for (uint8_t i = 0; i < mid_index; i++)
                        if (memory_array[i][0] == 1)
                            rest_cnt2 += 1;

                    for (uint8_t i = memory_index; i < _MEM_ROWS; i++)
                        if (memory_array[i][0] == 1)
                            rest_cnt2 += 1;
                }
                else if (memory_index < mid_index)
                {
                    for (uint8_t i = memory_index; i < mid_index; i++)
                        if (memory_array[i][0] == 1)
                            rest_cnt2 += 1;

                    for (uint8_t i = 0; i < memory_index; i++)
                        if (memory_array[i][0] == 1)
                            rest_cnt1 += 1;

                    for (uint8_t i = mid_index; i < _MEM_ROWS; i++)
                        if (memory_array[i][0] == 1)
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
            for (uint8_t i = 0; i < _MEM_ROWS; i++)
            {
                if (memory_array[i][0] == 3)
                    movement_cnt += 1;

                if (memory_array[i][0] == 4)
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

        for (uint8_t i = 0; i < _MEM_ROWS; i++)
            if (memory_array[i][0] == 2)
                ingestion_cnt += 1;

        if (ingestion_cnt >= 2)
            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if (memory_array[i][0] == 3)
                    memory_array[i][0] = 6;

        rest_cnt = 0;
        deta_a_cnt = 0;
        jicha_cnt = 0;
        eighteen_average = 0;
        sum_eighteen_average = 0;

        for (uint8_t i = 0; i < _MEM_ROWS; i++)
            if (memory_array[i][0] == 1)
                rest_cnt += 1;

        if (rest_cnt <= 4)
        {
            for (uint8_t i = 0; i < _MEM_ROWS; i++)
            {
                if (memory_array[i][2] > 130 && memory_array[i][2] < 700)
                    deta_a_cnt += 1;

                if (memory_array[i][3] < 120)
                    jicha_cnt += 1;
            }

            if (deta_a_cnt >= 14 && jicha_cnt >= 14)
            {
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    eighteen_average += memory_array[i][1];

                eighteen_average = eighteen_average / _MEM_ROWS;

                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    sum_eighteen_average += abs(eighteen_average - memory_array[i][1]);

                if (sum_eighteen_average <= 400 && eighteen_average < 150)
                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
                        memory_array[i][0] = 5;
            }
        }

        if (memory_index == 17)
            memory_index_o = 0;
        else
            memory_index_o = memory_index + 1;

        *result = (uint8_t)memory_array[memory_index_o][0];

        memset(&threshold_judge, 0, sizeof(threshold_judge));
        memset(&three_axis_average_info, 0, sizeof(three_axis_average_info));
        memset(&sum_info, 0, sizeof(sum_info));
        x_max_val = -2000;
        x_min_val = 2000;
        y_max_val = -2000;
        y_min_val = 2000;
        z_max_val = -2000;
        z_min_val = 2000;
        pending_samples = 0;
        return true;
    }

    return false;
}
void behavior_reset(void)
{
    memset(diff_three_axis_info, 0, sizeof(diff_three_axis_info));
    memset(&three_axis_average_info, 0, sizeof(three_axis_average_info));
    memset(&sum_info, 0, sizeof(sum_info));
    memset(&threshold_judge, 0, sizeof(threshold_judge));
    memset(memory_array, 0, sizeof(memory_array));
    memory_index = memory_index_o = pending_samples = 0;
    x_max_val = y_max_val = z_max_val = -2000;
    x_min_val = y_min_val = z_min_val = 2000;
}
