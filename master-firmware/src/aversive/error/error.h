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
 *  Revision : $Id: error.h,v 1.11.4.3 2007-12-31 16:25:00 zer0 Exp $
 *
 */

#ifndef _ERROR_H_
#define _ERROR_H_

#include <stdint.h>
#include <error/general_errors.h>

#define ERROR_SEVERITY_EMERG    0
#define ERROR_SEVERITY_ERROR    1
#define ERROR_SEVERITY_WARNING  2
#define ERROR_SEVERITY_NOTICE   3
#define ERROR_SEVERITY_DEBUG    4


/** enable the dump of the comment */
#define ERROR_DUMP_TEXTLOG

/** enable the dump of filename and line number */
#define ERROR_DUMP_FILE_LINE

/** The error structure, which is given as a parameter in log funcs */
struct error {
	unsigned int err_num;        /**< Error number */
	unsigned int severity;       /**< Error severity */
	const char * text;      /**< Error text */
	const char * file;      /**< File in which the error occurred */
	unsigned int line;          /**< Line number in which the error occurred */
};

/** Structure of pointers to functions which are called on errors
 */
struct error_fct {
	void (*emerg)(struct error *, ...);     /**< Pointer to emergency func */
	void (*error)(struct error *, ...);     /**< Pointer to error func */
	void (*warning)(struct error *, ...);   /**< Pointer to warning func */
	void (*notice)(struct error *, ...);    /**< Pointer to notice func */
	void (*debug)(struct error *, ...);     /**< Pointer to debug func */
} ;

extern struct error_fct g_error_fct;


struct error error_generate(uint8_t num, uint8_t severity, const char * t, const char * f, uint16_t l);

/** Register log function for EMERG level */
void error_register_emerg(void (*f)(struct error *, ...));

/** Register log function for ERROR level */
void error_register_error(void (*f)(struct error *, ...));

/** Register log function for WARNING level */
void error_register_warning(void (*f)(struct error *, ...));

/** Register log function for NOTICE level */
void error_register_notice(void (*f)(struct error *, ...));

/** Register log function for DEBUG level */
void error_register_debug(void (*f)(struct error *, ...));




/** Call this macro to log EMERG events */
#define EMERG(num, text, ...)  do {                                            \
	if(g_error_fct.emerg) {                                                \
		struct error e = error_generate(num, ERROR_SEVERITY_EMERG,     \
								(text),    \
								(__FILE__),\
								__LINE__);     \
		g_error_fct.emerg(&e, ##__VA_ARGS__);                          \
	}                                                                      \
} while(0)

/** Call this macro to log ERROR events */
#define ERROR(num, text, ...)  do {                                            \
	if(g_error_fct.error) {                                                \
		struct error e = error_generate(num, ERROR_SEVERITY_ERROR,     \
								(text),    \
								(__FILE__),\
								__LINE__);     \
		g_error_fct.error(&e, ##__VA_ARGS__);                          \
	}                                                                      \
} while(0)

/** Call this macro to log WARNING events */
#define WARNING(num, text, ...)  do {                                          \
	if(g_error_fct.warning) {                                              \
		struct error e = error_generate(num, ERROR_SEVERITY_WARNING,   \
								(text),    \
								(__FILE__),\
								__LINE__);     \
		g_error_fct.warning(&e, ##__VA_ARGS__);                        \
	}                                                                      \
} while(0)

/** Call this macro to log NOTICE events */
#define NOTICE(num, text, ...)  do {                                           \
	if(g_error_fct.notice) {                                               \
		struct error e = error_generate(num, ERROR_SEVERITY_NOTICE,    \
								(text),    \
								(__FILE__),\
								__LINE__);     \
		g_error_fct.notice(&e, ##__VA_ARGS__);                         \
	}                                                                      \
} while(0)

/** Call this macro to log DEBUG events */
#define DEBUG(num, text, ...)  do {                                            \
	if(g_error_fct.debug) {                                                \
		struct error e = error_generate(num, ERROR_SEVERITY_DEBUG,     \
								(text),    \
								(__FILE__),\
								__LINE__);     \
		g_error_fct.debug(&e, ##__VA_ARGS__);                          \
	}                                                                      \
} while(0)




#endif /* _ERROR_H_ */
