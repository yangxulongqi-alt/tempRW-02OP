// shared_types.cpp
#include "shared_types.h"
#include "shared_types_cpp.h"
#include <unordered_map>
#include <mutex>


static std::unordered_map<uint16_t, MotorParams> g_motor_params;
static std::mutex g_motor_params_mutex;

void set_motor_params(uint16_t motor_id, const MotorParams& params) {
    std::lock_guard<std::mutex> lock(g_motor_params_mutex);
    g_motor_params[motor_id] = params;
}

extern "C" int get_motor_params(uint16_t motor_id, MotorParams* out_params) {
    std::lock_guard<std::mutex> lock(g_motor_params_mutex);
    auto it = g_motor_params.find(motor_id);
    if (it == g_motor_params.end()) return -1;
    *out_params = it->second;
    return 0;
}