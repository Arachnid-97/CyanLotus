/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */


/* ------------------------ System architecture includes ----------------------------- */
#include "stm32f4xx.h"

/* ------------------------ lwIP includes --------------------------------- */
#include "lwip/debug.h"
#include "lwip/sys.h"
#include "lwip/opt.h"
#include "lwip/stats.h"


#ifndef errno
int errno = 0;
#endif

#define SYS_ARCH_BLOCKING_TICKTIMEOUT    ((portTickType)1000)

/* This is the number of threads that can be started with sys_thread_new() */
#define SYS_THREAD_MAX 6

static u16_t s_nextthread = 0;


/************************************************************************
* Generates a pseudo-random number.
* NOTE: Contrubuted by the FNET project.
*************************************************************************/
static  u32_t _rand_value;
u32_t lwip_rand(void)
{
    _rand_value = _rand_value * 1103515245u + 12345u;
	return((u32_t)(_rand_value>>16u) % (32767u + 1u));
}

/*
 * Prints an assertion messages and aborts execution.
 */
void sys_assert( char *pcMessage , char *File, int Line)
{
//FSL:only needed for debugging
#ifdef LWIP_DEBUG
    printf("[Lwip Info] %s. \n\rWrong parameters value: file %s on line %d\r\n", pcMessage, File, Line);
    printf("\r\n");
#endif
#if !NO_SYS
    portENTER_CRITICAL();
#endif
    for (;;)
    {}
}

/* Structure associating a thread to a struct sys_timeouts */
struct TimeoutlistPerThread {
	sys_thread_t pid;        /* The thread id */
};


/**
 * \brief Initialize the sys_arch layer.
 */
void sys_init(void)
{
	// keep track of how many threads have been created
	s_nextthread = 0;
}

/**
 * \brief Creates and returns a new semaphore.
 *
 * \param sem Pointer to the semaphore.
 * \param count Initial state of the semaphore.
 *
 * \return ERR_OK for OK, other value indicates error.
 */
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
	err_t err_sem = ERR_MEM;

//    LWIP_UNUSED_ARG(count);
    LWIP_ASSERT("sem != NULL", sem != NULL);

    if( count > 1U )
    {
        *sem = xSemaphoreCreateCounting( count, count );
    }
    else
    {
        *sem = xSemaphoreCreateBinary();
    }

    if( *sem != SYS_SEM_NULL )
    {
		/* Means we want the sem to be
		   unavailable at init state. */
        if( count == 0U )
        {
            xSemaphoreTake( *sem, 1UL );
        }

        err_sem = ERR_OK;
        SYS_STATS_INC_USED( sem );
    }
    else
    {
        SYS_STATS_INC( sem.err );
    }

	return err_sem;
}

/**
 * \brief Frees a semaphore created by sys_sem_new.
 *
 * \param sem Pointer to the semaphore.
 */
void sys_sem_free(sys_sem_t *sem)
{
    LWIP_ASSERT("sem != NULL", sem != NULL);

	/* Sanity check */
	if ( sem != SYS_SEM_NULL ) {
		SYS_STATS_DEC(sem.used);
	}

	vQueueDelete( *sem );
}

/**
 * \brief Signals (or releases) a semaphore.
 *
 * \param sem Pointer to the semaphore.
 */
void sys_sem_signal(sys_sem_t *sem)
{
    LWIP_ASSERT("sem != NULL", sem != NULL);

	xSemaphoreGive( *sem );
}

/**
 * \brief Blocks the thread while waiting for the semaphore to be signaled.
 * Note that there is another function sys_sem_wait in sys.c, but it is a wrapper
 * for the sys_arch_sem_wait function. Please note that it is important for the
 * semaphores to return an accurate count of elapsed milliseconds, since they are
 * used to schedule timers in lwIP.
 *
 * \param sem Pointer to the semaphore.
 * \param timeout The timeout parameter specifies how many milliseconds the
 * function should block before returning; if the function times out, it should
 * return SYS_ARCH_TIMEOUT. If timeout=0, then the function should block
 * indefinitely. If the function acquires the semaphore, it should return how
 * many milliseconds expired while waiting for the semaphore. 
 *
 * \return SYS_ARCH_TIMEOUT if times out, ERR_MEM for semaphore erro otherwise
 * return the milliseconds expired while waiting for the semaphore.
 */
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
	TickType_t StartTime;
	TickType_t EndTime;
	TickType_t Elapsed;
	unsigned long Return;

    LWIP_ASSERT("sem != NULL", sem != NULL);
    
    StartTime = xTaskGetTickCount();

    if( timeout != 0UL )
    {
        if( xSemaphoreTake( *sem, timeout / portTICK_PERIOD_MS ) == pdTRUE )
        {
            EndTime = xTaskGetTickCount();
            Elapsed = (EndTime - StartTime) * portTICK_PERIOD_MS;
            Return = Elapsed;
        }
        else
        {
            Return = SYS_ARCH_TIMEOUT;
        }
    }
    else
    {
        while( xSemaphoreTake( *sem, portMAX_DELAY ) != pdTRUE );
        EndTime = xTaskGetTickCount();
        Elapsed = ( EndTime - StartTime ) * portTICK_PERIOD_MS;

        if( Elapsed == 0UL )
        {
            Elapsed = 1UL;
        }

        Return = Elapsed;
    }

    return Return;
}

#ifndef sys_sem_valid
/**
 * \brief Check if a sempahore is valid/allocated.
 *
 * \param sem Pointer to the semaphore.
 *
 * \return Semaphore number on valid, 0 for invalid.
 */
int sys_sem_valid(sys_sem_t *sem)
{
	return ( (int)(*sem) );
}

#endif

#ifndef sys_sem_set_invalid
/**
 * \brief Set a semaphore invalid.
 *
 * \param sem Pointer to the semaphore.
 */
void sys_sem_set_invalid(sys_sem_t *sem)
{
	*sem = SYS_SEM_NULL;
}
#endif


/* Mutex functions: */
/** Define LWIP_COMPAT_MUTEX if the port has no mutexes and binary semaphores
 *  should be used instead */
#if !LWIP_COMPAT_MUTEX
/**
 * \brief Create a new mutex.
 *
 * \param mutex Pointer to the mutex to create.
 *
 * \return A new mutex.
 */
err_t sys_mutex_new(sys_mutex_t *mutex)
{
	err_t err_mutex = ERR_MEM;

    LWIP_ASSERT("mutex != NULL", mutex != NULL);

    *mutex = xSemaphoreCreateMutex();

    if( *mutex != SYS_SEM_NULL )
    {
        err_mutex = ERR_OK;
        SYS_STATS_INC_USED( mutex );
    }
    else
    {
        SYS_STATS_INC( mutex.err );
    }

    return err_mutex;
}

/**
 * \brief Delete a semaphore.
 *
 * \param mutex the mutex to delete.
 */
void sys_mutex_free(sys_mutex_t *mutex)
{
    LWIP_ASSERT("mutex != NULL", mutex != NULL);
    
	/* Sanity check */
	if ( mutex != SYS_SEM_NULL ) {
		SYS_STATS_DEC( mutex.used );
	}

    vQueueDelete( *mutex );
}

/**
 * \brief Lock a mutex.
 *
 * \param mutex the mutex to lock.
 */
void sys_mutex_lock(sys_mutex_t *mutex)
{
    LWIP_ASSERT("mutex != NULL", mutex != NULL);

    while( xSemaphoreTake( *mutex, portMAX_DELAY ) != pdPASS );
}

/**
 * \brief Unlock a mutex.
 *
 * \param mutex the mutex to unlock.
 */
void sys_mutex_unlock(sys_mutex_t *mutex)
{
    LWIP_ASSERT("mutex != NULL", mutex != NULL);

	xSemaphoreGive(*mutex);
}

#ifndef sys_mutex_valid
/**
 * \brief Check if a mutex is valid/allocated.
 *
 * \param mutex Pointer to the mutex.
 *
 * \return Valid mutex number or 0 for invalid.
 */
int sys_mutex_valid(sys_mutex_t *mutex)
{
	return ((int)(*mutex));
}

#endif

#ifndef sys_mutex_set_invalid
/**
 * \brief Set a mutex invalid so that sys_mutex_valid returns 0.
 *
 * \param mutex Pointer to the mutex.
 */
void sys_mutex_set_invalid(sys_mutex_t *mutex)
{
	*mutex = SYS_SEM_NULL;
}

#endif

#endif /* LWIP_COMPAT_MUTEX */

/**
 * \brief Creates an empty mailbox for maximum "size" elements. Elements stored
 * in mailboxes are pointers. 
 *
 * \param mBoxNew Pointer to the new mailbox.
 * \param size Maximum "size" elements.
 *
 * \return ERR_OK if successfull or ERR_MEM on error.
 */
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	err_t err_mbox = ERR_MEM;

    LWIP_ASSERT("mbox != NULL", mbox != NULL);

    *mbox = xQueueCreate( size, sizeof( void * ) );

    if( *mbox != SYS_MBOX_NULL )
    {
        err_mbox = ERR_OK;
        SYS_STATS_INC_USED( mbox );
    }

    return err_mbox;
}

/**
 * \brief Deallocates a mailbox.
 * If there are messages still present in the mailbox when the mailbox is
 * deallocated, it is an indication of a programming error in lwIP and the
 * developer should be notified.
 *
 * \param mbox Pointer to the new mailbox.
 */
void sys_mbox_free(sys_mbox_t *mbox)
{
	unsigned long MessagesWaiting;

    LWIP_ASSERT("mbox != NULL", mbox != NULL);
    LWIP_ASSERT("mbox not empty", !uxQueueMessagesWaiting(*mbox));

    MessagesWaiting = uxQueueMessagesWaiting( *mbox );

	if( MessagesWaiting != 0UL )
	{
		/* Line for breakpoint.  Should never break here! */
		portNOP();
		SYS_STATS_INC( mbox.err );

		// TODO notify the user of failure.
	}

	/* Sanity check */
	if ( mbox != SYS_MBOX_NULL ) {
		SYS_STATS_DEC( mbox.used );
	}

    vQueueDelete( *mbox );
}

/**
 * \brief Posts the "msg" to the mailbox. This function have to block until the
 * "msg" is really posted.
 *
 * \param mbox Pointer to the mailbox.
 * \param msg Pointer to the message to be post.
 */
void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
    LWIP_ASSERT("mbox != NULL", mbox != NULL);

    while( xQueueSendToBack( *mbox, &msg, portMAX_DELAY ) != pdTRUE );
}

/**
 * \brief Try to posts the "msg" to the mailbox.
 *
 * \param mbox Pointer to the mailbox.
 * \param msg Pointer to the message to be post.
 *
 * \return ERR_MEM if the mailbox is full otherwise ERR_OK if the "msg" is posted.
 */
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	err_t err_mbox = ERR_MEM;
    portBASE_TYPE taskToWake = pdFALSE;

    LWIP_ASSERT("mbox != NULL", mbox != NULL);

#ifdef __CA7_REV
    if (SystemGetIRQNestingLevel())
#else
    if (__get_IPSR())
#endif
    {
        if (pdTRUE == xQueueSendToBackFromISR(*mbox, &msg, &taskToWake))
        {
            if(taskToWake == pdTRUE)
            {
                portYIELD_FROM_ISR(taskToWake);
            }
            err_mbox = ERR_OK;
        }
        else
        {
            /* The queue was already full. */
            SYS_STATS_INC( mbox.err );
            err_mbox = ERR_MEM;
        }
    }
    else
    {
        if(pdTRUE == xQueueSendToBack(*mbox, &msg, 0) )
        {
            err_mbox = ERR_OK;
        }
        else
        {
            /* The queue was already full. */
            SYS_STATS_INC( mbox.err );
            err_mbox = ERR_MEM;
        }
    }

	return err_mbox;
}

/**
 * \brief Blocks the thread until a message arrives in the mailbox, but does not
 * block the thread longer than "timeout" milliseconds (similar to the
 * sys_arch_sem_wait() function).
 *
 * \param mbox Pointer to the mailbox.
 * \param msg A result parameter that is set by the function (i.e., by doing
 * "*msg = ptr"). The "msg" parameter maybe NULL to indicate that the message
 * should be dropped.
 * \timeout 0 indicates the thread should be blocked until a message arrives.
 *
 * \return Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was
 * a timeout. Or ERR_MEM if invalid pointer to message box.
 */
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	TickType_t StartTime;
	TickType_t EndTime;
	TickType_t Elapsed;
	void *dummyptr;
	unsigned long Return;

    LWIP_ASSERT("mbox != NULL", mbox != NULL);

	StartTime = xTaskGetTickCount();

	if ( msg == NULL )
	{
		msg = &dummyptr;
	}

    if( timeout != 0UL )
    {
        if( pdTRUE == xQueueReceive( *mbox, &( *msg ), timeout/ portTICK_PERIOD_MS ) )
        {
            EndTime = xTaskGetTickCount();
            Elapsed = ( EndTime - StartTime ) * portTICK_PERIOD_MS;

            Return = Elapsed;
        }
        else
        {
            /* Timed out. */
            *msg = NULL;
            Return = SYS_ARCH_TIMEOUT;
        }
    }
    else // block forever for a message.
    {
        while( pdTRUE != xQueueReceive( *mbox, &( *msg ), portMAX_DELAY ) );
        EndTime = xTaskGetTickCount();
        Elapsed = ( EndTime - StartTime ) * portTICK_PERIOD_MS;

        if( Elapsed == 0UL )
        {
            Elapsed = 1UL;
        }

        Return = Elapsed;
    }

    return Return; // return time blocked TODO test
}

#ifndef sys_arch_mbox_tryfetch
/**
 * \brief This is similar to sys_arch_mbox_fetch, however if a message is not
 * present in the mailbox, it immediately returns with the code SYS_MBOX_EMPTY.
 * On success 0 is returned.
 *
 * \param mbox Pointer to the mailbox.
 * \param msg A result parameter that is set by the function (i.e., by doing
 * "*msg = ptr"). The "msg" parameter maybe NULL to indicate that the message
 * should be dropped.
 *
 * \return Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was
 * a timeout. Or ERR_MEM if invalid pointer to message box.
 */
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	void *dummyptr;
	unsigned long Return;

    LWIP_ASSERT("mbox != NULL", mbox != NULL);

    if( msg== NULL )
    {
        msg = &dummyptr;
    }

    if( pdTRUE == xQueueReceive( *mbox, &( *msg ), 0UL ) )
    {
        Return = ERR_OK;
    }
    else
    {
        Return = SYS_MBOX_EMPTY;
    }

    return Return;
}
#endif

#ifndef sys_mbox_valid
/**
 * \brief Check if an mbox is valid/allocated.
 *
 * \param mbox Pointer to the mailbox.
 *
 * \return Mailbox for valid, 0 for invalid.
 */
int sys_mbox_valid(sys_mbox_t *mbox)
{
	return ( (int)(*mbox) );
}
#endif

#ifndef sys_mbox_set_invalid
/**
 * \brief Set an mbox invalid.
 *
 * \param mbox Pointer to the mailbox.
 */
void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
	*mbox = SYS_MBOX_NULL;
}

#endif



//If threads are supported by the underlying operating system and if
//such functionality is needed in lwIP, the following function will have
//to be implemented as well:

/**
 * \brief Instantiate a thread for lwIP. Both the id and the priority are
 * system dependent.
 *
 * \param name Pointer to the thread name.
 * \param thread Thread function.
 * \param arg Argument will be passed into the thread().
 * \param stacksize Stack size of the thread.
 * \param prio Thread priority.
 *
 * \return The id of the new thread.
 */
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg,
		int stacksize, int prio)
{
	xTaskHandle CreatedTask;
	portBASE_TYPE Result;
    sys_thread_t Return;

    LWIP_ASSERT("invalid stacksize", stacksize > 0);
	LWIP_ASSERT("thread overflow", s_nextthread < SYS_THREAD_MAX);

	if ( s_nextthread < SYS_THREAD_MAX )
	{
		Result = xTaskCreate( thread, name, (configSTACK_DEPTH_TYPE)stacksize, arg, prio, &CreatedTask );
		LWIP_ASSERT("task creation failed", Result == pdPASS);

		if(Result == pdPASS)
		{
			Return = CreatedTask;
            s_nextthread++;
		}
		else
		{
			Return = NULL;
		}
	}
	else
	{
		Return = NULL;
	}

	return Return;
}

/* This optional function does a "fast" critical region protection and returns
 * the previous protection level. This function is only called during very short
 * critical regions. An embedded system which supports ISR-based drivers might
 * want to implement this function by disabling interrupts. Task-based systems
 * might want to implement this by using a mutex or disabling tasking. This
 * function should support recursive calls from the same task or interrupt. In
 * other words, sys_arch_protect() could be called while already protected. In
 * that case the return value indicates that it is already protected.*/
#if SYS_LIGHTWEIGHT_PROT
/**
 * \brief Protect the system.
 *
 * \return 0 on success.
 */
sys_prot_t sys_arch_protect(void)
{
    sys_prot_t result = 0;

#ifdef __CA7_REV
    if (SystemGetIRQNestingLevel())
#else
    if (__get_IPSR())
#endif    
    {
        result = portSET_INTERRUPT_MASK_FROM_ISR();
    }
    else
    {
        portENTER_CRITICAL();
    }
    return result;
}

/**
 * \brief Unprotect the system.
 *
 * \param pval Protect value.
 */
void sys_arch_unprotect(sys_prot_t pval)
{
#ifdef __CA7_REV
    if (SystemGetIRQNestingLevel())
#else
    if (__get_IPSR())
#endif
    {
        portCLEAR_INTERRUPT_MASK_FROM_ISR(pval);
    }
    else
    {
        portEXIT_CRITICAL();
    }
}

#endif /* SYS_LIGHTWEIGHT_PROT */

u32_t sys_now(void)
{
#ifdef __CA7_REV
    if (SystemGetIRQNestingLevel())
#else
    if (__get_IPSR())
#endif
    {
        return xTaskGetTickCountFromISR();
    }
    else
    {
        return xTaskGetTickCount();
    }
}

