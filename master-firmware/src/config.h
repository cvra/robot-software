#include <parameter/parameter.h>

#ifdef __cplusplus
extern "C" {
#endif


extern parameter_namespace_t global_config;
extern parameter_namespace_t actuator_config;
extern parameter_namespace_t master_config;


/* Inits all the globally available objects. */
void config_init(void);

/** Shorthand to get a parameter via its name.
 *
 * @note Panics if the ID is unknown.
 */
float config_get_scalar(const char *id);
int config_get_integer(const char *id);
bool config_get_boolean(const char *id);

/* Macro to easily find a parameter from path */
#define PARAMETER(s) parameter_find(&global_config, (s))

#ifdef __cplusplus
}
#endif
