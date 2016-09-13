/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: general_errors.h,v 1.5.4.3 2009-01-23 23:54:15 zer0 Exp $
 *
 */


/** 
 * these are general errors. 
 */

/* Not module specific */

/* Operation not permitted */
#define EPERM_COMMENT   "Operation not permitted"

/* No such file or directory */
#define ENOENT_COMMENT   "No such file or directory"

/* I/O error */
#define EIO_COMMENT   "I/O error"

/* No such device or address */
#define ENXIO_COMMENT   "No such device or address"

/* Argument list too long */
#define E2BIG_COMMENT   "Argument list too long"

/* Try again */
#define EAGAIN_COMMENT   "Try again"

/* Out of memory */
#define ENOMEM_COMMENT   "Out of memory"

/* Bad address */
#define EFAULT_COMMENT   "Bad address"

/* Device or resource busy */
#define EBUSY_COMMENT   "Device or resource busy"

/* Invalid argument */
#define EINVAL_COMMENT   "Invalid argument"

/* Unkwow error */
#define EUNKNOW_COMMENT   "Unkwow error"


/* Module specific, from 129 to 192 */

#define E_UART 129
#define E_ROBOT_SYSTEM 130
#define E_MULTISERVO 131
#define E_TRAJECTORY 132
#define E_I2C 133
#define E_BLOCKING_DETECTION_MANAGER 134
#define E_OA 135
#define E_SPI 136
#define E_CC2420 137
#define E_TIME_EXT 138

/* ... etc TBD */

/* User specific, from > 192 */
#define E_ARM 193

#define E_STRAT 193

/* defined in user app */
