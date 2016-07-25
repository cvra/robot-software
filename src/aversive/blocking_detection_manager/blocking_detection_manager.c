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
 *  Revision : $Id: blocking_detection_manager.c,v 1.1.2.7 2009-05-18 12:29:10 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org>
 */

/* blocking detection manager */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <error/error.h>
#include <blocking_detection_manager/blocking_detection_manager.h>


#define ABS(val) ({					\
			__typeof(val) __val = (val);	\
			if (__val < 0)			\
				__val = - __val;	\
			__val;				\
		})

/** init module, give the robot system to use as a parameter */
void bd_init(struct blocking_detection * bd, struct cs *cs)
{
    memset(bd, 0, sizeof(*bd));
    bd->cs = cs;
}


/** reset current blocking */
void bd_reset(struct blocking_detection * bd)
{
    bd->cpt = 0;
}

/**
 *
 */
void bd_set_thresholds(struct blocking_detection *bd, uint32_t err_thres, uint16_t cpt_thres) {
    bd->cpt_thres = cpt_thres;
    bd->err_thres = err_thres;
}



/** function to be called periodically */
void bd_manage(struct blocking_detection * bd)
{
	uint32_t err = ABS(cs_get_error(bd->cs));

	if(bd->err_thres == 0) return;

	if(bd->err_max < err)
		bd->err_max = err;

	if(err > bd->err_thres) {
		bd->cpt++;
	}
	else {
		bd->cpt=0;
    }
}

/** get value of blocking detection */
uint8_t bd_get(struct blocking_detection * bd)
{
    return (bd->cpt_thres && (bd->cpt >= bd->cpt_thres));
}

/** get value of blocking detection maximale value, reseted each time it's read*/
int32_t bd_get_max(struct blocking_detection * bd)
{
    int32_t ret;
    ret = bd->err_max;
    bd->err_max = 0;
    return ret;
}
