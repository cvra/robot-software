#include <parameter/parameter.h>

#ifdef __cplusplus
extern "C" {
#endif


extern parameter_namespace_t global_config;
extern parameter_namespace_t motor_config;

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
