/*
 * Copyright (c) 2013
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *    logging.h
 * \author
 *    Marcus Lunden <marcus.lunden@gmail.com>
 * \desc
 *    logging utility; depends on some underlying layer to store and read data
 *    The calling layer is responsible for checking the return value and retrying
 *    if not successful, or handling sustained errors. This logging driver will
 *    just drop if unsuccessful. Generally, errors are returned as 0. Non-errors
 *    are returned as > 0.
 */

#ifndef __LOGGING_H__
#define __LOGGING_H__

#include "cfs/cfs-coffee.h"

enum loggingtypes {
  /* Legas */
  LOGTYPE_TRACE_LEGA_TXREQ = 1, /* Tx trace request at IS */
  LOGTYPE_TRACE_LEGA_TXSAVE,    /* Tx trace Lega->IS for saving */
  LOGTYPE_TRACE_LEGA_RXPLAY,    /* Lega got trace for playing, saved wo problems */
  LOGTYPE_TRACE_LEGA_OVERWR,    /* Lega got trace for playing, overwrote oldest */
  LOGTYPE_TRACE_LEGA_PLAYED,    /* Played trace, duration... */
  LOGTYPE_POSITION,             /* When checking position, log where we are and were we were last */

  /* IS nodes */
  LOGTYPE_TRACE_IS_REQ,     /* Tx trace request at IS */
  LOGTYPE_TRACE_IS_SAVE,    /* Tx trace Lega->IS for saving */
  LOGTYPE_TRACE_IS_TX,      /* Tx trace to a Lega */
  LOGTYPE_BATTERY,          /* Battery state (include in energest instead?) */

  /* Common for both IS and Lega */
  LOGTYPE_TRACE_BUF_DROP,   /* Trace buffer was overfull when saving, dropped */
  LOGTYPE_TRACE_PURGE,      /* Traces are purged (IS) or forgotten (Lega) */
  LOGTYPE_TRACE_WARN,       /* Generic warning when trace handling (unknown packet recv etc) */

  LOGTYPE_JESUS_REBOOT,     /* At every reboot */
  LOGTYPE_JESUS_WAKEUP,     /* When waking up from suspension (Sandman) */
  LOGTYPE_JESUS_SUSPEND,    /* Going to sleep (Sandman) */

  LOGTYPE_SYS_INFO,         /* Generic info log from system, eg util commands? */
  LOGTYPE_SYS_WARN,         /* Warnings from system, eg a parameter that was out of range */
  LOGTYPE_SYS_ERROR,        /* Errors, but not fatal. Eg stopping corrupt trace playing that was to long */
  LOGTYPE_SYS_FATAL,        /* Fatal error, such as CFS not able to write something etc */

  /* Others */
  LOGTYPE_LOG_EXTENDED_ENTRY,   /* This entry will be longer than a standard entry (eg energest logs, or params) */
  LOGTYPE_LOG_DIVIDER,
};

volatile enum log_states {
  LOGGING_NOT_INIT = 0x00,
  LOGGING_STOPPED,
  LOGGING_BUSY,
  LOGGING_OK_TO_LOG,
  LOGGING_FATAL_ERROR,
} log_state = LOGGING_NOT_INIT;


/* each log entry is preceded by the log header */
struct logheader {
  uint16_t  divider;  /* Always first to ensure that it can be translated to CR+LF when parsing log */
  uint32_t  time;     /* timestamp, as expressed in local time */
  enum loggingtypes  type;    /*  */
  uint16_t  len;      /* length of log payload */
};

//#define LOG_HDR_LEN     sizeof(struct logheader)  
#define LOGGING_FILE_NAME       "log.txt" /* log cfs file name on flash */
#define LOGGING_LOGDIVIDER      0x9999  /* the divider between individual log entries */
#define LOGGING_MAX_LOGLEN      25      /* the maximum length of the logging message, excl header */
/*----------------------------------------------------------------------------*/
int
logging_init(void);

int
logging_log(enum loggingtypes type, void *log, uint8_t len);

void
logging_serialprint(void);

int
logging_stop(void);

enum log_states
logging_state(void);

int
logging_purge_logs(void);

int
logging_format(void);
/*--------------------------------------------------------------------------*/
#endif /* #ifndef __LOGGING_H__ */
