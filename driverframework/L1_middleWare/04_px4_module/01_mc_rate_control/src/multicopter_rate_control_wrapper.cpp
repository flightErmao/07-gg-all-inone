/****************************************************************************
 *
 * C wrapper for MulticopterRateControl C++ class
 * This file provides C interface to be used in C task files
 *
 ****************************************************************************/

#include "MulticopterRateControl.hpp"
#include <rtthread.h>

extern "C" {
    // Global instance pointer
    static MulticopterRateControl* g_rate_control_instance = nullptr;

    /**
     * @brief Initialize MulticopterRateControl instance
     * @param vtol VTOL mode flag
     * @return 0 on success, -1 on failure
     */
    int multicopter_rate_control_init(int vtol) {
        if (g_rate_control_instance != nullptr) {
            // Already initialized
            return 0;
        }

        g_rate_control_instance = new MulticopterRateControl(vtol != 0);
        
        if (g_rate_control_instance == nullptr) {
            return -1;
        }

        if (!g_rate_control_instance->init()) {
            delete g_rate_control_instance;
            g_rate_control_instance = nullptr;
            return -1;
        }

        return 0;
    }

    /**
     * @brief Run one cycle of rate control
     * This calls the run() method of MulticopterRateControl
     */
    void multicopter_rate_control_run(void) {
        if (g_rate_control_instance != nullptr) {
            g_rate_control_instance->run();
        }
    }

    /**
     * @brief Cleanup MulticopterRateControl instance
     */
    void multicopter_rate_control_cleanup(void) {
        if (g_rate_control_instance != nullptr) {
            delete g_rate_control_instance;
            g_rate_control_instance = nullptr;
        }
    }
}

