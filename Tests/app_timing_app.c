/* app.c with three collaborators renamed, so app_timing_test.c can count the
 * resampler and sample-clock status codes without changing application logic. */
#define resampler_push sim_resampler_push
#define sample_clock_batch sim_sample_clock_batch
#define behavior_push sim_behavior_push
#include "../App/Src/app.c"
