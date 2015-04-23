#include <parameter/parameter.h>

#define SLAVE_CONFIG_COUNT 20

#ifdef __cplusplus
extern "C" {
#endif

struct pid_parameter_s {
    parameter_namespace_t root;
    parameter_t kp;
    parameter_t ki;
    parameter_t kd;
    parameter_t ilimit;
};

struct slave_config_s {
    parameter_namespace_t root;
    parameter_namespace_t pid_root;
    char name[4]; // "0"..."127"
    struct pid_parameter_s speed_pid;
    struct pid_parameter_s position_pid;
    struct pid_parameter_s current_pid;
};

extern parameter_namespace_t global_config;
extern struct slave_config_s slave_configs[SLAVE_CONFIG_COUNT];

/* Inits all the globally available objects. */
void config_init(void);

/** Shorthand to get a parameter via its name.
 *
 * @note Panics if the ID is unknown.
 */
float config_get_scalar(const char *id);

#ifdef __cplusplus
}
#endif
