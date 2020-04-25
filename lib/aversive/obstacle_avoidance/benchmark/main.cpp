#include <benchmark/benchmark.h>
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>

static void BM_ObstacleAvoidance(benchmark::State& state)
{
    const point_t start = {.x = 1000, .y = 1000};
    const point_t end = {.x = 2000, .y = 1000};
    point_t* points;
    struct obstacle_avoidance oa;

    polygon_set_boundingbox(0, 0, 3000, 3000);
    oa_init(&oa);
    oa_start_end_points(&oa, start.x, start.y, end.x, end.y);

    for (int i = 0; i < state.range(0); i++) {
        auto obstacle = oa_new_poly(&oa, 4);
        oa_poly_set_point(&oa, obstacle, 50 + 100 * i, 900, 3);
        oa_poly_set_point(&oa, obstacle, 50 + 100 * i, 1300, 2);
        oa_poly_set_point(&oa, obstacle, 150 + 100 * i, 1300, 1);
        oa_poly_set_point(&oa, obstacle, 150 + 100 * i, 900, 0);
    }

    for (auto _ : state) {
        oa_process(&oa);

        auto point_cnt = oa_get_path(&oa, &points);
        benchmark::DoNotOptimize(point_cnt);
    }
}

BENCHMARK(BM_ObstacleAvoidance)->RangeMultiplier(2)->Range(1, 8);
BENCHMARK_MAIN();
