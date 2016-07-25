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
 *  Revision : $Id: error.c,v 1.10.4.3 2007-12-31 16:25:00 zer0 Exp $
 *
 */


#include <string.h>

#include <platform.h>
#include <aversive/error.h>

struct error_fct g_error_fct;

/** All fcts pointers to NULL */
void error_init(void)
{
	memset(&g_error_fct, 0, sizeof(g_error_fct));
}


struct error error_generate(uint8_t num, uint8_t severity, const char * t, 
			    const char * f, uint16_t l) {
	struct error e;      

	e.err_num = num;
	e.severity = severity;
#ifdef ERROR_DUMP_TEXTLOG
	e.text = t;
#else
	e.text = PSTR("");
#endif
#ifdef ERROR_DUMP_FILE_LINE
	e.file = f;
	e.line = l;
#else
	e.file = PSTR("");
	e.line = 0;
#endif
	return e;
}


/** Register log function for EMERG level */
void error_register_emerg(void (*f)(struct error *, ...))
{
	g_error_fct.emerg = f;
}

/** Register log function for ERROR level */
void error_register_error(void (*f)(struct error *, ...))
{
	g_error_fct.error = f;
}

/** Register log function for WARNING level */
void error_register_warning(void (*f)(struct error *, ...))
{
	g_error_fct.warning = f;
}

/** Register log function for NOTICE level */
void error_register_notice(void (*f)(struct error *, ...))
{
	g_error_fct.notice = f;
}

/** Register log function for DEBUG level */
void error_register_debug(void (*f)(struct error *, ...))
{
	g_error_fct.debug = f;
}

