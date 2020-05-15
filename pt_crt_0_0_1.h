/*
 * Cornell Rocketry Team
 * Electrical and Software Subteam
 *  File: pt_crt_0_0_1.h
 *  Created: 1/7/2020 3:25:33 PM
 *  Author: sdd58
 */

#include "crt_config_0_0_1.h"

/*
 * Copyright (c) 2004-2005, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: pt.h,v 1.7 2006/10/02 07:52:56 adam Exp $
 */



/**
 * \file
 * Protothreads implementation.
 * \author
 * Adam Dunkels <adam@sics.se>
 *
 */

#ifndef __PT_H__
#define __PT_H__

////////////////////////
//#include "lc.h"
////////////////////////
/**
 * \file lc.h
 * Local continuations
 * \author
 * Adam Dunkels <adam@sics.se>
 *
 */

#ifdef DOXYGEN
/**
 * Initialize a local continuation.
 *
 * This operation initializes the local continuation, thereby
 * unsetting any previously set continuation state.
 *
 * \hideinitializer
 */
#define LC_INIT(lc)

/**
 * Set a local continuation.
 *
 * The set operation saves the state of the function at the point
 * where the operation is executed. As far as the set operation is
 * concerned, the state of the function does <b>not</b> include the
 * call-stack or local (automatic) variables, but only the program
 * counter and such CPU registers that needs to be saved.
 *
 * \hideinitializer
 */
#define LC_SET(lc)

/**
 * Resume a local continuation.
 *
 * The resume operation resumes a previously set local continuation, thus
 * restoring the state in which the function was when the local
 * continuation was set. If the local continuation has not been
 * previously set, the resume operation does nothing.
 *
 * \hideinitializer
 */
#define LC_RESUME(lc)

/**
 * Mark the end of local continuation usage.
 *
 * The end operation signifies that local continuations should not be
 * used any more in the function. This operation is not needed for
 * most implementations of local continuation, but is required by a
 * few implementations.
 *
 * \hideinitializer 
 */
#define LC_END(lc)

/**
 * \var typedef lc_t;
 *
 * The local continuation type.
 *
 * \hideinitializer
 */
#endif /* DOXYGEN */

//#ifndef __LC_H__
//#define __LC_H__


//#ifdef LC_INCLUDE
//#include LC_INCLUDE
//#else

/////////////////////////////
//#include "lc-switch.h"
/////////////////////////////

//#ifndef __LC_SWITCH_H__
//#define __LC_SWITCH_H__

/* WARNING! lc implementation using switch() does not work if an
   LC_SET() is done within another switch() statement! */

/** \hideinitializer */
/*
typedef unsigned short lc_t;

#define LC_INIT(s) s = 0;

#define LC_RESUME(s) switch(s) { case 0:

#define LC_SET(s) s = __LINE__; case __LINE__:

#define LC_END(s) }

#endif /* __LC_SWITCH_H__ */

/** @} */

//#endif /* LC_INCLUDE */

//#endif /* __LC_H__ */

/** @} */
/** @} */

/////////////////////////////
//#include "lc-addrlabels.h"
/////////////////////////////

#ifndef __LC_ADDRLABELS_H__
#define __LC_ADDRLABELS_H__

/** \hideinitializer */
typedef void * lc_t;

#define LC_INIT(s) s = NULL

#define LC_RESUME(s)				\
  do {						\
    if(s != NULL) {				\
      goto *s;					\
    }						\
  } while(0)

#define LC_CONCAT2(s1, s2) s1##s2
#define LC_CONCAT(s1, s2) LC_CONCAT2(s1, s2)

#define LC_SET(s)				\
  do {						\
    LC_CONCAT(LC_LABEL, __LINE__):   	        \
    (s) = &&LC_CONCAT(LC_LABEL, __LINE__);	\
  } while(0)

#define LC_END(s)

#endif /* __LC_ADDRLABELS_H__ */

//////////////////////////////////////////
struct pt {
  lc_t lc;
};

#define PT_WAITING 0
#define PT_YIELDED 1
#define PT_EXITED  2
#define PT_ENDED   3

/**
 * \name Initialization
 * @{
 */

/**
 * Initialize a protothread.
 *
 * Initializes a protothread. Initialization must be done prior to
 * starting to execute the protothread.
 *
 * \param pt A pointer to the protothread control structure.
 *
 * \sa PT_SPAWN()
 *
 * \hideinitializer
 */
#define PT_INIT(pt)   LC_INIT((pt)->lc)

/** @} */

/**
 * \name Declaration and definition
 * @{
 */

/**
 * Declaration of a protothread.
 *
 * This macro is used to declare a protothread. All protothreads must
 * be declared with this macro.
 *
 * \param name_args The name and arguments of the C function
 * implementing the protothread.
 *
 * \hideinitializer
 */
#define PT_THREAD(name_args) char name_args

/**
 * Declare the start of a protothread inside the C function
 * implementing the protothread.
 *
 * This macro is used to declare the starting point of a
 * protothread. It should be placed at the start of the function in
 * which the protothread runs. All C statements above the PT_BEGIN()
 * invokation will be executed each time the protothread is scheduled.
 *
 * \param pt A pointer to the protothread control structure.
 *
 * \hideinitializer
 */
#define PT_BEGIN(pt) { char PT_YIELD_FLAG = 1; LC_RESUME((pt)->lc)

/**
 * Declare the end of a protothread.
 *
 * This macro is used for declaring that a protothread ends. It must
 * always be used together with a matching PT_BEGIN() macro.
 *
 * \param pt A pointer to the protothread control structure.
 *
 * \hideinitializer
 */
#define PT_END(pt) LC_END((pt)->lc); PT_YIELD_FLAG = 0; \
                   PT_INIT(pt); return PT_ENDED; }

/** @} */

/**
 * \name Blocked wait
 * @{
 */

/**
 * Block and wait until condition is true.
 *
 * This macro blocks the protothread until the specified condition is
 * true.
 *
 * \param pt A pointer to the protothread control structure.
 * \param condition The condition.
 *
 * \hideinitializer
 */
#define PT_WAIT_UNTIL(pt, condition)	        \
  do {						\
    LC_SET((pt)->lc);				\
    if(!(condition)) {				\
      return PT_WAITING;			\
    }						\
  } while(0)

/**
 * Block and wait while condition is true.
 *
 * This function blocks and waits while condition is true. See
 * PT_WAIT_UNTIL().
 *
 * \param pt A pointer to the protothread control structure.
 * \param cond The condition.
 *
 * \hideinitializer
 */
#define PT_WAIT_WHILE(pt, cond)  PT_WAIT_UNTIL((pt), !(cond))

/** @} */

/**
 * \name Hierarchical protothreads
 * @{
 */

/**
 * Block and wait until a child protothread completes.
 *
 * This macro schedules a child protothread. The current protothread
 * will block until the child protothread completes.
 *
 * \note The child protothread must be manually initialized with the
 * PT_INIT() function before this function is used.
 *
 * \param pt A pointer to the protothread control structure.
 * \param thread The child protothread with arguments
 *
 * \sa PT_SPAWN()
 *
 * \hideinitializer
 */
#define PT_WAIT_THREAD(pt, thread) PT_WAIT_WHILE((pt), PT_SCHEDULE(thread))

/**
 * Spawn a child protothread and wait until it exits.
 *
 * This macro spawns a child protothread and waits until it exits. The
 * macro can only be used within a protothread.
 *
 * \param pt A pointer to the protothread control structure.
 * \param child A pointer to the child protothread's control structure.
 * \param thread The child protothread with arguments
 *
 * \hideinitializer
 */
#define PT_SPAWN(pt, child, thread)		\
  do {						\
    PT_INIT((child));				\
    PT_WAIT_THREAD((pt), (thread));		\
  } while(0)

/** @} */

/**
 * \name Exiting and restarting
 * @{
 */

/**
 * Restart the protothread.
 *
 * This macro will block and cause the running protothread to restart
 * its execution at the place of the PT_BEGIN() call.
 *
 * \param pt A pointer to the protothread control structure.
 *
 * \hideinitializer
 */
#define PT_RESTART(pt)				\
  do {						\
    PT_INIT(pt);				\
    return PT_WAITING;			\
  } while(0)

/**
 * Exit the protothread.
 *
 * This macro causes the protothread to exit. If the protothread was
 * spawned by another protothread, the parent protothread will become
 * unblocked and can continue to run.
 *
 * \param pt A pointer to the protothread control structure.
 *
 * \hideinitializer
 */
#define PT_EXIT(pt)				\
  do {						\
    PT_INIT(pt);				\
    return PT_EXITED;			\
  } while(0)

/** @} */

/**
 * \name Calling a protothread
 * @{
 */

/**
 * Schedule a protothread.
 *
 * This function shedules a protothread. The return value of the
 * function is non-zero if the protothread is running or zero if the
 * protothread has exited.
 *
 * \param f The call to the C function implementing the protothread to
 * be scheduled
 *
 * \hideinitializer
 */
#define PT_SCHEDULE(f) ((f) < PT_EXITED)
//#define PT_SCHEDULE(f) ((f))

/** @} */

/**
 * \name Yielding from a protothread
 * @{
 */

/**
 * Yield from the current protothread.
 *
 * This function will yield the protothread, thereby allowing other
 * processing to take place in the system.
 *
 * \param pt A pointer to the protothread control structure.
 *
 * \hideinitializer
 */
#define PT_YIELD(pt)				\
  do {						\
    PT_YIELD_FLAG = 0;				\
    LC_SET((pt)->lc);				\
    if(PT_YIELD_FLAG == 0) {			\
      return PT_YIELDED;			\
    }						\
  } while(0)

/**
 * \brief      Yield from the protothread until a condition occurs.
 * \param pt   A pointer to the protothread control structure.
 * \param cond The condition.
 *
 *             This function will yield the protothread, until the
 *             specified condition evaluates to true.
 *
 *
 * \hideinitializer
 */
#define PT_YIELD_UNTIL(pt, cond)		\
  do {						\
    PT_YIELD_FLAG = 0;				\
    LC_SET((pt)->lc);				\
    if((PT_YIELD_FLAG == 0) || !(cond)) {	\
      return PT_YIELDED;                        \
    }						\
  } while(0)

/** @} */

#endif /* __PT_H__ */

#ifndef __PT_SEM_H__
#define __PT_SEM_H__

//#include "pt.h"

struct pt_sem {
  unsigned int count;
};

/**
 * Initialize a semaphore
 *
 * This macro initializes a semaphore with a value for the
 * counter. Internally, the semaphores use an "unsigned int" to
 * represent the counter, and therefore the "count" argument should be
 * within range of an unsigned int.
 *
 * \param s (struct pt_sem *) A pointer to the pt_sem struct
 * representing the semaphore
 *
 * \param c (unsigned int) The initial count of the semaphore.
 * \hide initializer
 */
#define PT_SEM_INIT(s, c) (s)->count = c

/**
 * Wait for a semaphore
 *
 * This macro carries out the "wait" operation on the semaphore. The
 * wait operation causes the protothread to block while the counter is
 * zero. When the counter reaches a value larger than zero, the
 * protothread will continue.
 *
 * \param pt (struct pt *) A pointer to the protothread (struct pt) in
 * which the operation is executed.
 *
 * \param s (struct pt_sem *) A pointer to the pt_sem struct
 * representing the semaphore
 *
 * \hideinitializer
 */
#define PT_SEM_WAIT(pt, s)	\
  do {						\
    PT_WAIT_UNTIL(pt, (s)->count > 0);		\
    --(s)->count;				\
  } while(0)

/**
 * Signal a semaphore
 *
 * This macro carries out the "signal" operation on the semaphore. The
 * signal operation increments the counter inside the semaphore, which
 * eventually will cause waiting protothreads to continue executing.
 *
 * \param pt (struct pt *) A pointer to the protothread (struct pt) in
 * which the operation is executed.
 *
 * \param s (struct pt_sem *) A pointer to the pt_sem struct
 * representing the semaphore
 *
 * \hideinitializer
 */
#define PT_SEM_SIGNAL(pt, s) ++(s)->count

#endif /* __PT_SEM_H__ */

//=====================================================================
//=== BRL4 additions to Protothreads ==================================
//=====================================================================

// macro to time thread execution interval in millisec
#define PT_YIELD_TIME_msec(delay_time) \
	do { static unsigned int time_thread ;\
		time_thread = time_tick_millsec + (unsigned int)delay_time ;\
		PT_YIELD_UNTIL(pt, (time_tick_millsec >= time_thread)) ;\
	} while(0);
                                 
//macro to return system time								                          
#define PT_GET_TIME() (time_tick_millsec)
                                                                                                          
// init rate sehcduler
//#define PT_INIT(pt, priority)   LC_INIT((pt)->lc ; (pt)->pri = priority)
//PT_PRIORITY_INIT
#define PT_RATE_INIT() int pt_pri_count = 0;
// maitain proority frame count
//PT_PRIORITY_LOOP maitains a counter used to control execution
#define PT_RATE_LOOP() pt_pri_count = (pt_pri_count+1) & 0xf ;
// schecedule priority thread
//PT_PRIORITY_SCHEDULE
// 5 levels
// rate 0 is highest -- every time thru loop
// priority 1 -- every 2 times thru loop
// priority 2 -- every 4 times thru loop
//  3 is  -- every 8 times thru loop
#define PT_RATE_SCHEDULE(f,rate) \
	if((rate==0) | \
	(rate==1 && ((pt_pri_count & 0b1)==0) ) | \
	(rate==2 && ((pt_pri_count & 0b11)==0) ) | \
	(rate==3 && ((pt_pri_count & 0b111)==0)) | \
	(rate==4 && ((pt_pri_count & 0b1111)==0))) \
		PT_SCHEDULE(f);                                                                                                                    
                                                           
// macros to manipulate a semaphore without blocking
#define PT_SEM_SET(s) (s)->count=1
#define PT_SEM_CLEAR(s) (s)->count=0
#define PT_SEM_READ(s) (s)->count
#define PT_SEM_ACCEPT(s) \
	s->count; \
	if (s->count) s->count-- ; \
	
//====================================================================
// IMPROVED SCHEDULER

// === thread structures ===
// thread control structs

// A modified scheduler
static struct pt pt_sched ;

// count of defined tasks
int pt_task_count = 0 ;

// The task structure
struct ptx {
	struct pt pt;              // thread context
	int num;                    // thread number
	char (*pf)(struct pt *pt); // pointer to thread function
	int rate;
};

// === extended structure for scheduler ===============
// an array of task structures
#define MAX_THREADS 10
static struct ptx pt_thread_list[MAX_THREADS];
// see https://github.com/edartuz/c-ptx/tree/master/src
// and the license above
// add an entry to the thread list
//struct ptx *pt_add( char (*pf)(struct pt *pt), int rate) {
int pt_add( char (*pf)(struct pt *pt), int rate) {
	if (pt_task_count < (MAX_THREADS)) {
		// get the current thread table entry
		struct ptx *ptx = &pt_thread_list[pt_task_count];
		// enter the tak data into the thread table
		ptx->num   = pt_task_count;
		// function pointer
		ptx->pf    = pf;
		// rate scheduler rate
		ptx->rate  = rate ;
		PT_INIT( &ptx->pt );
		// count of number of defined threads
		pt_task_count++;
		// return current entry
		return pt_task_count-1;
	}
	return NULL;
}

/* Scheduler
Copyright (c) 2014 edartuz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
// === Scheduler Thread =================================================
// update a 1 second tick counter
// schedulser code was almost copied from
// https://github.com/edartuz/c-ptx
// see license above

// choose schedule method
#define SCHED_ROUND_ROBIN 0
#define SCHED_RATE 1
int pt_sched_method = SCHED_ROUND_ROBIN ;

#define PT_SET_RATE(thread_num, new_rate) pt_thread_list[thread_num].rate = new_rate
#define PT_GET_RATE(thread_num) pt_thread_list[thread_num].rate

static PT_THREAD (protothread_sched(struct pt *pt))
{
	PT_BEGIN(pt);
	// set up rate counter
	PT_RATE_INIT()
	
	static int i, rate;
	
	if (pt_sched_method==SCHED_ROUND_ROBIN){
		while(1) {
			// test stupid round-robin
			// on all defined threads
			struct ptx *ptx = &pt_thread_list[0];
			// step thru all defined threads
			// -- loop can have more than one initialization or increment/decrement,
			// -- separated using comma operator. But it can have only one condition.
			for (i=0; i<pt_task_count; i++, ptx++ ){
				// call thread function
				(pt_thread_list[i].pf)(&ptx->pt);
			}
			// Never yields!
			// NEVER exit while!
		} // END WHILE(1)
	} // end if(pt_sched_method==SCHED_ROUND_ROBIN)
	
	else if (pt_sched_method==SCHED_RATE){
		while(1) {
			PT_RATE_LOOP() ;
			// test stupid round-robin
			// on all defined threads
			struct ptx *ptx = &pt_thread_list[0];
			// step thru all defined threads
			// -- loop can have more than one initialization or increment/decrement,
			// -- separated using comma operator. But it can have only one condition.
			for (i=0; i<pt_task_count; i++, ptx++ ){
				// rate
				rate = pt_thread_list[i].rate ;
				if((rate==0) |
				(rate==1 && ((pt_pri_count & 0b1)==0) ) |
				(rate==2 && ((pt_pri_count & 0b11)==0) ) |
				(rate==3 && ((pt_pri_count & 0b111)==0)) |
				(rate==4 && ((pt_pri_count & 0b1111)==0))){
					// call thread function
					(pt_thread_list[i].pf)(&ptx->pt);
				}
			}
			// Never yields!
			// NEVER exit while!
		} // END WHILE(1)
	} //end if (pt_sched_method==SCHED_RATE)
	
	PT_END(pt);
} // scheduler thread


//=====================================================================
//=== CRT additions to Protothreads  ==================================
//===================================================================== 
 
// system time updated in Systick ISR below
volatile unsigned int time_tick_millsec ;
volatile unsigned int ul_tickcount ;

/* Interupt handler for Systick */
void SysTick_Handler() {
	
	ul_tickcount++ ;
	time_tick_millsec++ ;
	
	if (ul_tickcount % 1000 == 0){ // Update every 1000
		//PORT->Group[0].OUTTGL.reg = PORT_PA17;
	}

}

/* Protothreads setup function */
void PT_setup (void){
	
	//Configure the device to run on the SAMD

	PORT->Group[0].DIRSET.reg = PORT_PA17; // Make PA17 an output
	PORT->Group[0].OUTSET.reg = PORT_PA17; // Set output level
	
	SysTick->CTRL = 0;     // Disable SysTick
	SysTick->LOAD = 47999UL; // Set reload to 1ms (999)
	
	NVIC_SetPriority(SysTick_IRQn, 3); // SysTick is low priority
	
	SysTick->VAL = 0;				   // Clear Value 
	SysTick->CTRL = 0x00000007;		   // Enable SysTick, Enable Exceptions, Use CPU CLK
	
	NVIC_EnableIRQ(SysTick_IRQn);      // Enable Interrupts
}

/*******************************************************************************
 * Function:        void ClocksInit(void)
 *
 * PreCondition:    OSC8M Enabled, CPU and Synch. Bus Clocks at reset condition (1 MHz)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    CPU and Synchronous Bus Clocks at 48 MHz.
 *					8 MHz Asynchronous Peripheral Clock available on GCLK 3
 *					Flash Read Wait States Adjusted Accordingly.
 *					GCLK_MAIN output on PA28
 *
 * Overview:
 *
 *	Configure a 48 MHz Synchronous Clock for (CPU, AHB, APBx) & Set Flash Wait States for 48MHz
 *		- Use DFLL48M Source (Closed Loop, using external 32.768 kHz crystal)
 *  Configure a 8 MHz Asynchronous Clock on Clock Generator 3 for use with peripherals. 
 *		- Use OSC8M Source
 *
 *	At reset:
 *	- OSC8M clock source is enabled with a divider by 8 (1MHz).
 *	- Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
 *	- OSCULP32K is enabled and fed to Generic Clock Generator 2
 *	- 0 Flash wait states (NVCTRL->CTRLB.RWS = 0)
 *	- Instruction cache is enabled (NVMCTRL->CTRLB.CACHEDIS = 0)
 *	- CPU, APBx clock division is by 1 (CPU, APBx Buses running at 1MHz)
 *	- APBA clocks are connected to EIC, RTC, WDT, GCLK, SYSCTRL,PM, PAC0 peripherals.
 *		
 *	The following steps will be followed to switch over to the 48MHz clock
 *	1) Set Flash wait states for 48 MHz (per Table 37-40 in data sheet)
 *	2) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 *	3) Put XOSC32K as source of Generic Clock Generator 1, 
 *	4) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 *	5) Enable DFLL48M clock
 *	6) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 *	7) Modify prescaler value of OSC8M to produce 8MHz output
 *	8) Put OSC8M as source for Generic Clock Generator 3
 *	9) Set CPU and APBx BUS Clocks for 48MHz operation
 *	 
 * Notes:
 *
 ******************************************************************************/

void CLK_setup (void) {
	
	uint32_t tempDFLL48CalibrationCoarse;	/* used to retrieve DFLL48 coarse calibration value from NVM */

	/* ----------------------------------------------------------------------------------------------
	* 1) Set Flash wait states for 48 MHz (per Table 37-40 in data sheet)
	*/
	
	NVMCTRL->CTRLB.bit.RWS = 1;		/* 1 wait state required @ 3.3V & 48MHz */
	
	/* ----------------------------------------------------------------------------------------------
	* 2) Enable XOSC32K clock (External on-board 32.768kHz oscillator), will be used as DFLL48M reference.
	*/
	
	// Configure SYSCTRL->XOSC32K settings
	SYSCTRL_XOSC32K_Type sysctrl_xosc32k = {
		.bit.WRTLOCK = 0,		/* XOSC32K configuration is not locked */
		.bit.STARTUP = 0x2,		/* 3 cycle start-up time */
		.bit.ONDEMAND = 0,		/* Osc. is always running when enabled */
		.bit.RUNSTDBY = 0,		/* Osc. is disabled in standby sleep mode */
		.bit.AAMPEN = 0,		/* Disable automatic amplitude control */
		.bit.EN32K = 1,			/* 32kHz output is disabled */
		.bit.XTALEN = 1			/* Crystal connected to XIN32/XOUT32 */
	};
	// Write these settings
	SYSCTRL->XOSC32K.reg = sysctrl_xosc32k.reg;
	// Enable the Oscillator - Separate step per data sheet recommendation (sec 17.6.3)
	SYSCTRL->XOSC32K.bit.ENABLE = 1;
	
	// Wait for XOSC32K to stabilize
	while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY);
	
	/* ----------------------------------------------------------------------------------------------
	* 3) Put XOSC32K as source of Generic Clock Generator 1
	*/
	
	// Set the Generic Clock Generator 1 output divider to 1
	// Configure GCLK->GENDIV settings
	GCLK_GENDIV_Type gclk1_gendiv = {
		.bit.DIV = 1,								/* Set output division factor = 1 */
		.bit.ID = GENERIC_CLOCK_GENERATOR_XOSC32K	/* Apply division factor to Generator 1 */
	};
	// Write these settings
	GCLK->GENDIV.reg = gclk1_gendiv.reg;
	
	// Configure Generic Clock Generator 1 with XOSC32K as source
	GCLK_GENCTRL_Type gclk1_genctrl = {
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 0,			/* Disable generator output to GCLK_IO[1] */
		.bit.OOV = 0,			/* GCLK_IO[1] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x05,		/* Generator source: XOSC32K output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_XOSC32K			/* Generator ID: 1 */
	};
	// Write these settings
	GCLK->GENCTRL.reg = gclk1_genctrl.reg;
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	/* ----------------------------------------------------------------------------------------------
	* 4) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
	*/
	
	GCLK_CLKCTRL_Type gclk_clkctrl = {
		.bit.WRTLOCK = 0,		/* Generic Clock is not locked from subsequent writes */
		.bit.CLKEN = 1,			/* Enable the Generic Clock */
		.bit.GEN = GENERIC_CLOCK_GENERATOR_XOSC32K, 	/* Generic Clock Generator 1 is the source */
		.bit.ID = 0x00			/* Generic Clock Multiplexer 0 (DFLL48M Reference) */
	};
	// Write these settings
	GCLK->CLKCTRL.reg = gclk_clkctrl.reg;
	
	/* ----------------------------------------------------------------------------------------------
	* 5) Enable DFLL48M clock
	*/
	
	// DFLL Configuration in Closed Loop mode, cf product data sheet chapter
	// 17.6.7.1 - Closed-Loop Operation
	
	// Enable the DFLL48M in open loop mode. Without this step, attempts to go into closed loop mode at 48 MHz will
	// result in Processor Reset (you'll be at the in the Reset_Handler in startup_samd21.c).
	// PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
	// Note that the DFLLRDY bit represents status of register synchronization - NOT clock stability
	// (see Data Sheet 17.6.14 Synchronization for detail)
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	SYSCTRL->DFLLCTRL.reg = (uint16_t)(SYSCTRL_DFLLCTRL_ENABLE);
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	
	// Set up the Multiplier, Coarse and Fine steps
	SYSCTRL_DFLLMUL_Type sysctrl_dfllmul = {
		.bit.CSTEP = 31,		/* Coarse step - use half of the max value (63) */
		.bit.FSTEP = 511,		/* Fine step - use half of the max value (1023) */
		.bit.MUL = 1465			/* Multiplier = MAIN_CLK_FREQ (48MHz) / EXT_32K_CLK_FREQ (32768 Hz) */
	};
	// Write these settings
	SYSCTRL->DFLLMUL.reg = sysctrl_dfllmul.reg;
	// Wait for synchronization
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	
	// To reduce lock time, load factory calibrated values into DFLLVAL (cf. Data Sheet 17.6.7.1)
	// Location of value is defined in Data Sheet Table 10-5. NVM Software Calibration Area Mapping
	
	// Get factory calibrated value for "DFLL48M COARSE CAL" from NVM Software Calibration Area
	tempDFLL48CalibrationCoarse = *(uint32_t*)FUSES_DFLL48M_COARSE_CAL_ADDR;
	tempDFLL48CalibrationCoarse &= FUSES_DFLL48M_COARSE_CAL_Msk;
	tempDFLL48CalibrationCoarse = tempDFLL48CalibrationCoarse>>FUSES_DFLL48M_COARSE_CAL_Pos;
	// Write the coarse calibration value
	SYSCTRL->DFLLVAL.bit.COARSE = tempDFLL48CalibrationCoarse;
	// Switch DFLL48M to Closed Loop mode and enable WAITLOCK
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	SYSCTRL->DFLLCTRL.reg |= (uint16_t) (SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK);
	
	/* ----------------------------------------------------------------------------------------------
	* 6) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
	*/
	
	// Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
	// Enable output of Generic Clock Generator 0 (GCLK_MAIN) to the GCLK_IO[0] GPIO Pin
	GCLK_GENCTRL_Type gclk_genctrl0 = {
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 1,			/* Enable generator output to GCLK_IO[0] */
		.bit.OOV = 0,			/* GCLK_IO[0] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x07,		/* Generator source: DFLL48M output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_MAIN			/* Generator ID: 0 */
	};
	GCLK->GENCTRL.reg = gclk_genctrl0.reg;
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	// Direct the GCLK_IO[0] output to PA28
	PORT_WRCONFIG_Type port0_wrconfig = {
		.bit.HWSEL = 1,			/* Pin# (28) - falls in the upper half of the 32-pin PORT group */
		.bit.WRPINCFG = 1,		/* Update PINCFGy registers for all pins selected */
		.bit.WRPMUX = 1,		/* Update PMUXn registers for all pins selected */
		.bit.PMUX = 7,			/* Peripheral Function H selected (GCLK_IO[0]) */
		.bit.PMUXEN = 1,		/* Enable peripheral Multiplexer */
		.bit.PINMASK = (uint16_t)(1 << (28-16)) /* Select the pin(s) to be configured */
	};
	// Write these settings
	PORT->Group[0].WRCONFIG.reg = port0_wrconfig.reg;
	
	/* ----------------------------------------------------------------------------------------------
	* 7) Modify prescaler value of OSC8M to produce 8MHz output
	*/

	SYSCTRL->OSC8M.bit.PRESC = 0;		/* Prescale by 1 */
	SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;	/* Oscillator is always on if enabled */
	
	/* ----------------------------------------------------------------------------------------------
	* 8) Put OSC8M as source for Generic Clock Generator 3
	*/
	
	// Set the Generic Clock Generator 3 output divider to 1
	// Configure GCLK->GENDIV settings
	GCLK_GENDIV_Type gclk3_gendiv = {
		.bit.DIV = 1,								/* Set output division factor = 1 */
		.bit.ID = GENERIC_CLOCK_GENERATOR_OSC8M		/* Apply division factor to Generator 3 */
	};
	// Write these settings
	GCLK->GENDIV.reg = gclk3_gendiv.reg;
	
	// Configure Generic Clock Generator 3 with OSC8M as source
	GCLK_GENCTRL_Type gclk3_genctrl = {
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 0,			/* Disable generator output to GCLK_IO[1] */
		.bit.OOV = 0,			/* GCLK_IO[2] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x06,		/* Generator source: OSC8M output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_OSC8M			/* Generator ID: 3 */
	};
	// Write these settings
	GCLK->GENCTRL.reg = gclk3_genctrl.reg;
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	/* ----------------------------------------------------------------------------------------------
	* 9) Set CPU and APBx BUS Clocks to 48MHz
	*/
	PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
	PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
	PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;
	
} // ClockSysInit48M()


/**********************
* build a string from the UART of length max_chars
* assuming that a HUMAN is typing at a terminal!
***********************/
#define MAX_CHARS 64
#define BACKSPACE 0x7f
#define USART_SERIAL_CHANNEL SERCOM0
char PT_term_buffer[MAX_CHARS];
int PT_get_serial_buffer(struct pt *pt) {
	static uint8_t character;
	static int num_char;
	
	// start of input thread
	PT_BEGIN(pt);
	
	num_char = 0;
	// clear buffer
	memset(PT_term_buffer, 0, MAX_CHARS);
	
	while(num_char < MAX_CHARS) {
		// get the character
		// yield until there is a valid character so that other
		// threads can execute
		PT_YIELD_UNTIL(pt, usart_data_available(USART_SERIAL_CHANNEL));
		character = usart_receive_byte(USART_SERIAL_CHANNEL);
		PT_YIELD_UNTIL(pt, !usart_is_busy(USART_SERIAL_CHANNEL));
		usart_transmit_byte(USART_SERIAL_CHANNEL, character);

		// end line
		if(character == '\r') {
			PT_term_buffer[num_char] = 0; // zero terminate the string
			PT_YIELD_UNTIL(pt, !usart_is_busy(USART_SERIAL_CHANNEL));
			usart_transmit_byte(USART_SERIAL_CHANNEL,'\n');
			break;
		}
		// backspace
		else if(character == BACKSPACE) {
			// Some terminal emulators do not remove character
			// on back space (they just move cursor back). So, after
			// moving cursor back on backspace, overwrite with space
			// then write another backspace. Last comment on:
			// https://www.microchip.com/forums/m979711.aspx
			PT_YIELD_UNTIL(pt, !usart_is_busy(USART_SERIAL_CHANNEL));
			usart_transmit_byte(USART_SERIAL_CHANNEL, ' ');
			PT_YIELD_UNTIL(pt, !usart_is_busy(USART_SERIAL_CHANNEL));
			usart_transmit_byte(USART_SERIAL_CHANNEL, BACKSPACE);
			num_char--;
			// check for buffer underflow
			if(num_char < 0) { num_char = 0; }
		}
		else { PT_term_buffer[num_char++] = character;}
	}
	
	PT_EXIT(pt);
	PT_END(pt);
}

/**********************
* send a string to the UART
***********************/
volatile char PT_send_buffer[MAX_CHARS] = "ack\r\n";
int num_send_chars;
int PT_put_serial_buffer(struct pt *pt) {
	PT_BEGIN(pt);
	num_send_chars = 0;
	while(PT_send_buffer[num_send_chars] != 0) {
		PT_YIELD_UNTIL(pt, !usart_is_busy(USART_SERIAL_CHANNEL));
		usart_transmit_byte(USART_SERIAL_CHANNEL, PT_send_buffer[num_send_chars]);
		num_send_chars++;
	}
	
	PT_EXIT(pt);
	PT_END(pt);
}