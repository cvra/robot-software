#ifndef ODOMETRY_PUBLISHER_H
#define ODOMETRY_PUBLISHER_H

#ifdef __cplusplus
extern "C" {
#endif

#define ODOMETRY_PUBLISHER_PORT 20000
#define ODOMETRY_PUBLISHER_HOST(server) LWIP_GATEWAY(server)
#define ODOMETRY_PUBLISHER_TIMESTEP_MS 20

void odometry_publisher_init(void);

#ifdef __cplusplus
}
#endif

#endif
