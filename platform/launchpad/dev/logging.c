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
 *    logging.c
 * \author
 *    Marcus Lunden <marcus.lunden@gmail.com>
 * \desc
 *    logging utility; depends on some underlying layer to store and read data
 *    The calling layer is responsible for checking the return value and retrying
 *    if not successful, or handling sustained errors. This logging driver will
 *    just drop if unsuccessful. Generally, errors are returned as 0. Non-errors
 *    are returned as > 0.
 *    
 */

#include <stdio.h>
#include "cfs/cfs.h"
#include "clock.h"
#include "logging.h"

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)


static int16_t fd = 0;    /* log file descriptor number */

/*----------------------------------------------------------------------------*/
/**
 * \brief      Init logging by opening the log file for writing. Also resume after a logging_stop() has closed the file.
 * \retval 0   fatal error, logging not possible
 * \retval 1   logging init ok
 *
 */

int
logging_init(void)
{
  if(log_state == LOGGING_NOT_INIT || log_state == LOGGING_STOPPED){
    fd = cfs_open(LOGGING_FILE_NAME, CFS_WRITE | CFS_APPEND);
    if(fd == -1){
      printf("FATAL Error, Logging could not open file.\n");
      log_state = LOGGING_FATAL_ERROR;
      return 0;
    }
    log_state = LOGGING_OK_TO_LOG;
    return 1;
  }
}
/*----------------------------------------------------------------------------*/
/**
 * \brief            Logs a log message by writing it to the log file. Arguments are the log type, a
 *                   pointer to the log structure and the length of it in bytes. Takes about 3 ms to
 *                   write 100 bytes on a Tmote Sky. Everything that is attempted to write during an
 *                   already commenced write (ie if busy) is just dropped now.
 * \param type       log type identifier
 * \param log        pointer to the log to store
 * \param len        length of the log to store
 * \retval 0         OK - storing the log was successful
 * \retval 1         error when storing the log header
 * \retval 2         error when storing the log
 * \retval 3         logging was not ready to store the log, dropping
 *
 */

int
logging_log(enum loggingtypes type, void *log, uint8_t len)
{
  if(log_state == LOGGING_OK_TO_LOG){
    /* create a log header, with time stamp etc */
    struct logheader e;
    log_state = LOGGING_BUSY;
    logheader.divider = LOGGING_LOGDIVIDER;
    logheader.time = (uint32_t) clock_time());
    logheader.type = type;
    logheader.len = len;

    if(cfs_write(fd, &e, LOG_HDR_LEN) == -1){
      /* error when storing the log header */
      printf("Log: ** FATAL ** could not write log hdr.\n");
      log_state = LOGGING_FATAL_ERROR;
      return 1;
    }

    if(cfs_write(fd, log, len) == -1){
      /* error when storing the log */
      printf("Log: ** FATAL ** could not write log.\n");
      log_state = LOGGING_FATAL_ERROR;
      return 2;
    }

    /* storing the log was successful */
    log_state = LOGGING_OK_TO_LOG;
    return 0;
  }
  
  /* logging was not ready to store the log, dropping */
  return 3;
}
/*----------------------------------------------------------------------------*/
/**
 * \brief      Print entire log on serial connection. Saves the state of the log when it is
 *             called. Restates that state and state of file (opened or not) when closing.
 *             Reads each log message, one by one and prints it. First reads the header, which
 *             contains the length etc, then the log itself.
 *
 */

void
logging_serialprint(void)
{
  volatile enum logging_states prev_state;
  uint8_t data[LOGGING_MAX_LOGLEN];   /* read data is stored here */
  struct logheader lh;

  uint8_t k;  //XXX debug counter
  
  /* XXX need to disable the watchdog before this? */
  /* Check logging status is ok first */
  if(log_state == LOGGING_NOT_INIT || log_state == LOGGING_FATAL_ERROR){
    printf("FATAL: log printf failed: %u\n", log_state);
    return;
  }
  /* If it is busy, we wait until it is ready */
  while(log_state == LOGGING_BUSY){ ; }
    
  prev_state = log_state;
  log_state = LOGGING_BUSY;
  if(prev_state == LOGGING_OK_TO_LOG){
    cfs_close(fd);
  }
  /* To print the log, the file must first be closed (now has CFS_WRITE flag) and
    reopened with CFS_READ. Then read file, print message by message, until the
    file ends. */
  fd = cfs_open(FILENAME, CFS_READ);



  do {
    /* Read the header */
    //r = cfs_read(fd, &((void *)lh), sizeof(struct logheader));
    r = cfs_read(fd, &lh, sizeof(struct logheader));
    if(r < 0) {
      printf("CFS read error\n"); /* Invalid file descriptor or not opened to read -> -1*/
      break;
    }
    if(r > 0) {
      printf("Log: End of file\n");
      break;
    }

    /* Read the log itself */
    //r = cfs_read(fd, &((void *)data), lh.len);
    r = cfs_read(fd, &data, lh.len);
    if(r == -1 || r == 0){
      printf("Log: read error\n");
      break;
    }

    /* read success, parse and print logmsg */
    printf("[%lu] Log Type %u Len %u ", lh.time, lh.type, lh.len);
    for(k=0 ; k<lh.len ; k++){
      printf(" %u", data[k]);
    }
    printf("\n");

#if 0
    switch(logmsg.type){
      /* Will this work? (default w time...) */
      case default:
        printf("[%lu] ", logmsg.time);
      case LOGTYPE_TRACEPLAY:
      case LOGTYPE_TRACESAVE:
        printf("Trace\n");
        break;
      case LOGTYPE_SYSERR:
      case LOGTYPE_SYSWARN:
        printf("System\n");
        break;
    }
#endif
  } while(r != -1 && r != 0);
  cfs_close(fd);
  if(prev_state == LOGGING_OK_TO_LOG){
    fd = cfs_open(FILENAME, CFS_WRITE | CFS_APPEND);
  }
  log_state = prev_state;
}
/*----------------------------------------------------------------------------*/
/**
 * \brief            Stops the logging aka close all files. Call logging_init() to open the file again 
 * \return           ret
 * \retval 0         logging stopped ok
 * \retval 1         error occurred
 */

int
logging_stop(void)
{
  if(log_state != LOGGING_NOT_INIT && log_state != LOGGING_FATAL_ERROR){
    cfs_close(fd);
    log_state = LOGGING_STOPPED;
    return 0;
  }
  /* error occurred */
  return 1;
}
/*----------------------------------------------------------------------------*/
/**
 * \brief            Check the state of the logging
 * \return           logging state
 *
 */

enum log_states
logging_state(void)
{
  return log_state;
}
/*----------------------------------------------------------------------------*/
/**
 * \brief            Remove all logs by removing all the files in use by this logging library (does not affect other files).
 * \retval 0         Logs removal ok
 * \retval 1         Some error occurred
 */

int
logging_purge_logs(void)
{
  if(log_state != LOGGING_NOT_INIT && log_state != LOGGING_FATAL_ERROR){

    /* If writing to flash, then wait till we're done */
    while(log_state == LOGGING_BUSY){ ; }

    if(!cfs_remove(LOGGING_FILE_NAME)){
      /* removal successful */
      return 0;
    }
  }

  /* Some error occurred */
  return 1;
}
/*----------------------------------------------------------------------------*/
/**
 * \brief            Format the flash. May take a long time, refer to the storage datasheet)
 * \retval 0         Format complete, result ok
 * \retval 1         Some error occurred, format may not be complete and flash is in unpredictable state
 */

int
logging_format(void)
{
  if(log_state == LOGGING_NOT_INIT || log_state == LOGGING_FATAL_ERROR){
    return 1; /* Don't even try, we're not ready */
  }
  /* If writing to flash, then wait till we're done */
  while(log_state == LOGGING_BUSY){ ; }
  if(cfs_coffee_format() == 0) {

    printf("Logging-format complete\n");
    return 0;
  } else {

    printf("FATAL: Logging-format failed\n");
    return 1; /* no success in removal of file */
  }
}
/*----------------------------------------------------------------------------*/



