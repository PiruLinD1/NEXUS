#define main original_main
#include "../tests/controller_test.cpp"
#undef main
int main() {
    std::cout << std::fixed << std::setprecision(6);
    for (bool mismatch : {false, true}) for (double delay : {.01, .04, .08, .12})
        for (unsigned scenario = 0; scenario < 3; ++scenario) runQueuedLatency(delay, scenario, true, mismatch);
    for (double delay : {.025, .065, .105}) for (unsigned scenario = 3; scenario < 6; ++scenario)
        runQueuedLatency(delay, scenario, true, true, false, 0, true, true);
    for (double delay : {.01, .04}) for (double age : {.005, .01, .02}) for (bool compensate : {false,true})
        runQueuedLatency(delay, 0, true, false, true, age, compensate);
}
