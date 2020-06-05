/*
 *  Copyright Droids Corporation (2007)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: blocking_detection_manager.h,v 1.1.2.6 2008-05-09 08:25:10 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */

/* blocking detection manager */

#ifndef BLOCKING_DETECTION_MANAGER_H_
#define BLOCKING_DETECTION_MANAGER_H_

#include <stdint.h>

/**@brief Detect blocking based on motor current.
 *
 * Triggers the blocking if:
 *   - the current in the motor is a above a threshold
 *     during n tests
 *   - the speed is below the threshold (if specified)
 *
 * We suppose that i = k1.V - k2.w
 * (V is the voltage applied on the motor, and w the current speed
 * of the motor)
 */

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Stores settings for blocking detection.
 *
 */
struct blocking_detection {
    uint16_t cpt_thres; /**< Number of err_thres surpasses to trigger blocking */
    uint16_t cpt; /**< Number of times that the current surpassed the threshold */
    uint32_t err_thres; /**< Current threshold */
    uint32_t err_max; /**< Highest current measured */
};

/** init module */
void bd_init(struct blocking_detection* bd);

void bd_set_thresholds(struct blocking_detection* bd, uint32_t err_thres, uint16_t cpt_thres);

/** reset the blocking */
void bd_reset(struct blocking_detection* bd);

/** function to be called periodically */
void bd_manage(struct blocking_detection* bd, uint32_t err);

/** get value of blocking detection */
uint8_t bd_get(struct blocking_detection* bd);

/** get value of blocking detection maximale value, reseted each time it's read*/
int32_t bd_get_max(struct blocking_detection* bd);

#ifdef __cplusplus
}
#endif

#endif
