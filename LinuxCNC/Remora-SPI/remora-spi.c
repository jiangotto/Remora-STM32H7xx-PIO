/********************************************************************
* Description:  remora-spi.c
*               This file, 'remora-rpispi.c', is a HAL component that
*               provides and SPI connection to a external STM32 running Remora PRU firmware.
*  				
*		Initially developed for RaspberryPi -> Arduino Due.
*		Further developed for RaspberryPi -> Smoothieboard and clones (LPC1768).
                Even further developed for RaspberryPi -> STM32 boards
*
* Author: Scott Alford
* License: GPL Version 3
*
*		Credit to GP Orcullo and PICnc V2 which originally inspired this
*		and portions of this code is based on stepgen.c by John Kasunich
*		and hm2_rpspi.c by Matsche
*
* Copyright (c) 2024	All rights reserved.
*
* Last change: updated for RPi5 with RP1 southbridge
********************************************************************/


#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"	/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

#define SPI_DEV_PATH "/dev/spidev1.0"


// Include these in the source directory when using "halcompile --install remora-spi.c"

// Using BCM2835 driver library by Mike McCauley, why reinvent the wheel!
// http://www.airspayce.com/mikem/bcm2835/index.html
// #include "bcm2835.h"
// #include "bcm2835.c"

// // Raspberry Pi 5 uses the RP1
// #include "rp1lib.h"
// #include "rp1lib.c"
// #include "gpiochip_rp1.h"
// #include "gpiochip_rp1.c"
#include "spi-dw.h"
#include "spi-dw.c"

// #include "dtcboards.h"

#include "remora.h"
#include "remoraStatus.h"

#define MODNAME "remora-spi"
#define PREFIX "remora"

MODULE_AUTHOR("Scott Alford AKA scotta");
MODULE_DESCRIPTION("Driver for Remora STM32 control boards")
MODULE_LICENSE("GPL v3");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

typedef struct {
	uint8_t			remoraStatus;
	hal_bit_t		*SPIenable;
	hal_bit_t		*SPIreset;
	hal_bit_t		*PRUreset;
	bool			SPIresetOld;
	hal_bit_t		*SPIstatus;
	hal_bit_t 		*stepperEnable[JOINTS];
	int				pos_mode[JOINTS];
	hal_float_t 	*pos_cmd[JOINTS];			// pin: position command (position units)
	hal_float_t 	*vel_cmd[JOINTS];			// pin: velocity command (position units/sec)
	hal_float_t 	*pos_fb[JOINTS];			// pin: position feedback (position units)
	hal_s32_t		*count[JOINTS];				// pin: psition feedback (raw counts)
	hal_float_t 	pos_scale[JOINTS];			// param: steps per position unit
	float 			freq[JOINTS];				// param: frequency command sent to PRU
	hal_float_t 	*freq_cmd[JOINTS];			// pin: frequency command monitoring, available in LinuxCNC
	hal_float_t 	maxvel[JOINTS];				// param: max velocity, (pos units/sec)
	hal_float_t 	maxaccel[JOINTS];			// param: max accel (pos units/sec^2)
	hal_float_t		*pgain[JOINTS];
	hal_float_t		*ff1gain[JOINTS];
	hal_float_t		*deadband[JOINTS];
	//float 			old_pos_cmd[JOINTS];		// previous position command (counts)
	//float 			old_pos_cmd_raw[JOINTS];		// previous position command (counts)
	float 			old_scale[JOINTS];			// stored scale value
	float 			scale_recip[JOINTS];		// reciprocal value used for scaling
	float			prev_cmd[JOINTS];
	float			cmd_d[JOINTS];					// command derivative
	hal_float_t 	*setPoint[VARIABLES];
	hal_float_t 	*processVariable[VARIABLES];
	hal_bit_t   	*outputs[DIGITAL_OUTPUTS];
	hal_bit_t   	*inputs[DIGITAL_INPUTS*2];
} data_t;

static data_t *data;

#pragma pack(push, 1)

typedef union
{
  // this allow structured access to the outgoing SPI data without having to move it
  // this is the same structure as the PRU rxData structure
  struct
  {
    uint8_t txBuffer[SPIBUFSIZE];
  };
  struct
  {
	int32_t header;
    int32_t jointFreqCmd[JOINTS];
    float 	setPoint[VARIABLES];
	uint8_t jointEnable;
	uint16_t outputs;
    uint8_t spare0;
  };
} txData_t;

static txData_t txData;


typedef union
{
  // this allow structured access to the incoming SPI data without having to move it
  // this is the same structure as the PRU txData structure
  struct
  {
    uint8_t rxBuffer[SPIBUFSIZE];
  };
  struct
  {
    int32_t header;
    int32_t jointFeedback[JOINTS];
    float 	processVariable[VARIABLES];
    uint16_t inputs;
  };
} rxData_t;

#pragma pack(pop)
static rxData_t rxData;



/* other globals */
static int 			comp_id;				// component ID
static const char 	*modname = MODNAME;
static const char 	*prefix = PREFIX;

static int 			num_chan = 0;			// number of step generators configured
static long 		old_dtns;				// update_freq function period in nsec - (THIS IS RUNNING IN THE PI)
static double		dt;						// update_freq period in seconds  - (THIS IS RUNNING IN THE PI)
static double 		recip_dt;				// recprocal of period, avoids divides

static int64_t 		accum[JOINTS] = { 0 };
static int32_t 		count[JOINTS] = { 0 };
static int32_t 		old_count[JOINTS] = { 0 };
static int8_t		filter_count[JOINTS] = { 0 };
static int32_t		accum_diff = 0;

typedef enum CONTROL { POSITION, VELOCITY, INVALID } CONTROL;

char *ctrl_type[JOINTS] = { "p" };
RTAPI_MP_ARRAY_STRING(ctrl_type,JOINTS,"control type (pos or vel)");

int PRU_base_freq = 200000;
RTAPI_MP_INT(PRU_base_freq, "PRU base thread frequency");

int CS_num = 0;
RTAPI_MP_INT(CS_num, "CS number");

int32_t SPI_freq = 10000000;
RTAPI_MP_INT(SPI_freq, "SPI frequency");

static int reset_gpio_pin = 233;				// RPI GPIO pin number used to force watchdog reset of the PRU 

struct gpiod_chip * reset_chip;      //GPIO控制器句柄
struct gpiod_line * reset_line;      //GPIO引脚句柄
int spi_fd;
static unsigned  mode = SPI_MODE_0;
static uint8_t bits = 8;
static uint8_t lsb_first = 0; // 将这个值设置为 0 表示 MSB 首先，设置为 1 则表示 LSB 首先


/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
static int rt_peripheral_init(void);

static void update_freq(void *arg, long period);
static void spi_write();
static void spi_read();
static void spi_transfer();
static void checkRemoraStatus(uint8_t status);
static CONTROL parse_ctrl_type(const char *ctrl);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];
	int n, retval;

	for (n = 0; n < JOINTS; n++) {
		if(parse_ctrl_type(ctrl_type[n]) == INVALID) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"STEPGEN: ERROR: bad control type '%s' for axis %i (must be 'p' or 'v')\n",
					ctrl_type[n], n);
			return -1;
		}
    }
	
	// check to see if the PRU base frequency has been set at the command line
	if (PRU_base_freq != -1)
	{
		if ((PRU_base_freq < 40000) || (PRU_base_freq > 240000))
		{
			rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: PRU base frequency incorrect\n");
			return -1;
		}
	}
	else
	{
		PRU_base_freq = PRU_BASEFREQ;
	}
	

    // connect to the HAL, initialise the driver
    comp_id = hal_init(modname);
    if (comp_id < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
		return -1;
    }

	// allocate shared memory
	data = hal_malloc(sizeof(data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: hal_malloc() failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	
	// initialise the gpio and spi peripherals
	if(!rt_peripheral_init())
	{
	  rtapi_print_msg(RTAPI_MSG_ERR,"rt_peripheral_init failed.\n");
      return -1;
		
	}

	// export remoraPRU SPI enable and status bits
	retval = hal_pin_bit_newf(HAL_IN, &(data->SPIenable),
			comp_id, "%s.SPI-enable", prefix);
	if (retval != 0) goto error;
	
	retval = hal_pin_bit_newf(HAL_IN, &(data->SPIreset),
			comp_id, "%s.SPI-reset", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_OUT, &(data->SPIstatus),
			comp_id, "%s.SPI-status", prefix);
	if (retval != 0) goto error;
	
	retval = hal_pin_bit_newf(HAL_IN, &(data->PRUreset),
			comp_id, "%s.PRU-reset", prefix);
	if (retval != 0) goto error;

    // export all the variables for each joint
    for (n = 0; n < JOINTS; n++) {
		// export pins

		data->pos_mode[n] = (parse_ctrl_type(ctrl_type[n]) == POSITION);
/*
This is throwing errors from axis.py for some reason...
		
		if (data->pos_mode[n]){
			rtapi_print_msg(RTAPI_MSG_ERR, "Creating pos_mode[%d] = %d\n", n, data->pos_mode[n]);
			retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
					comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->pos_cmd[n]) = 0.0;
		} else {
			rtapi_print_msg(RTAPI_MSG_ERR, "Creating vel_mode[%d] = %d\n", n, data->pos_mode[n]);
			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
					comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->vel_cmd[n]) = 0.0;			
		}
*/

		retval = hal_pin_bit_newf(HAL_IN, &(data->stepperEnable[n]),
				comp_id, "%s.joint.%01d.enable", prefix, n);
		if (retval != 0) goto error;

		retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
				comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->pos_cmd[n]) = 0.0;
		
		if (data->pos_mode[n] == 0){
			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
					comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->vel_cmd[n]) = 0.0;			
		}

		retval = hal_pin_float_newf(HAL_OUT, &(data->freq_cmd[n]),
		        comp_id, "%s.joint.%01d.freq-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->freq_cmd[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->pos_fb[n]),
		        comp_id, "%s.joint.%01d.pos-fb", prefix, n);
		if (retval < 0) goto error;
		*(data->pos_fb[n]) = 0.0;
		
		retval = hal_param_float_newf(HAL_RW, &(data->pos_scale[n]),
		        comp_id, "%s.joint.%01d.scale", prefix, n);
		if (retval < 0) goto error;
		data->pos_scale[n] = 1.0;

		retval = hal_pin_s32_newf(HAL_OUT, &(data->count[n]),
		        comp_id, "%s.joint.%01d.counts", prefix, n);
		if (retval < 0) goto error;
		*(data->count[n]) = 0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->pgain[n]),
				comp_id, "%s.joint.%01d.pgain", prefix, n);
		if (retval < 0) goto error;
		*(data->pgain[n]) = 0.0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->ff1gain[n]),
				comp_id, "%s.joint.%01d.ff1gain", prefix, n);
		if (retval < 0) goto error;
		*(data->ff1gain[n]) = 0.0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->deadband[n]),
				comp_id, "%s.joint.%01d.deadband", prefix, n);
		if (retval < 0) goto error;
		*(data->deadband[n]) = 0.0;
		
		retval = hal_param_float_newf(HAL_RW, &(data->maxaccel[n]),
		        comp_id, "%s.joint.%01d.maxaccel", prefix, n);
		if (retval < 0) goto error;
		data->maxaccel[n] = 1.0;
	}

	for (n = 0; n < VARIABLES; n++) {
	// export pins

		retval = hal_pin_float_newf(HAL_IN, &(data->setPoint[n]),
		        comp_id, "%s.SP.%01d", prefix, n);
		if (retval < 0) goto error;
		*(data->setPoint[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->processVariable[n]),
		        comp_id, "%s.PV.%01d", prefix, n);
		if (retval < 0) goto error;
		*(data->processVariable[n]) = 0.0;
	}

	for (n = 0; n < DIGITAL_OUTPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_IN, &(data->outputs[n]),
				comp_id, "%s.output.%02d", prefix, n);
		if (retval != 0) goto error;
		*(data->outputs[n])=0;
	}

	for (n = 0; n < DIGITAL_INPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_OUT, &(data->inputs[n]),
				comp_id, "%s.input.%02d", prefix, n);
		if (retval != 0) goto error;
		*(data->inputs[n])=0;
			
		retval = hal_pin_bit_newf(HAL_OUT, &(data->inputs[n+DIGITAL_INPUTS]),
				comp_id, "%s.input.%02d.not", prefix, n);
		if (retval != 0) goto error;
		*(data->inputs[n+DIGITAL_INPUTS])=1;   
	}

	error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: pin export failed with err=%i\n",
		        modname, retval);
		hal_exit(comp_id);
		return -1;
	}


	// Export functions
	rtapi_snprintf(name, sizeof(name), "%s.update-freq", prefix);
	retval = hal_export_funct(name, update_freq, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: update function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
	/* no FP operations */
	retval = hal_export_funct(name, spi_write, 0, 0, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: write function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, spi_read, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}


/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

int rt_peripheral_init(void)
{
	int ret = 0;
	// GPIO
	/*获取GPIO控制器*/
    reset_chip = gpiod_chip_open("/dev/gpiochip1");
    if(reset_chip == NULL){
        rtapi_print_msg(RTAPI_MSG_ERR,"gpiod_chip_open error\n");
        return -1;
    }

	/*获取GPIO引脚*/
    reset_line = gpiod_chip_get_line(reset_chip, reset_gpio_pin);
    if(reset_line == NULL){
        rtapi_print_msg(RTAPI_MSG_ERR,"gpiod_chip_get_line error\n");
        goto release_line;
    }

	/*设置GPIO为输出模式*/
    ret = gpiod_line_request_output(reset_line, "remora-reset", 0);
    if(ret < 0){
		rtapi_print_msg(RTAPI_MSG_ERR,"gpiod_line_request_output error\n");
        goto release_chip;
    }

	// SPI	
    //打开 SPI 设备
    spi_fd = open(SPI_DEV_PATH, O_RDWR);
    if (spi_fd < 0)
		rtapi_print_msg(RTAPI_MSG_ERR,"can't open spi\n");

    //spi mode 设置SPI 工作模式
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1)
		rtapi_print_msg(RTAPI_MSG_ERR,"can't set spi mode\n");

	//bits per word  设置一个字节的位数
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
		rtapi_print_msg(RTAPI_MSG_ERR,"can't set bits per word\n");

    //max speed hz  设置SPI 最高工作频率
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPI_freq);
    if (ret == -1)
		rtapi_print_msg(RTAPI_MSG_ERR,"can't set max speed hz\n");

	// 设置 MSB 优先（默认值为 true）
	ret = ioctl(spi_fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
	if (ret == -1)
		rtapi_print_msg(RTAPI_MSG_ERR,"can't set MSB first\n");

 
release_line:
    /*释放GPIO引脚*/
    gpiod_line_release(reset_line);
release_chip:
    /*释放GPIO控制器*/
    gpiod_chip_close(reset_chip);
    return -1;		
}

void update_freq(void *arg, long period)
{
	int i;
	data_t *data = (data_t *)arg;
	double max_ac, vel_cmd, dv, new_vel, max_freq, desired_freq;
		   
	double error, command, feedback;
	double periodfp, periodrecip;
	float pgain, ff1gain, deadband;

	// precalculate timing constants
    periodfp = period * 0.000000001;
    periodrecip = 1.0 / periodfp;

    // calc constants related to the period of this function (LinuxCNC SERVO_THREAD)
    // only recalc constants if period changes
    if (period != old_dtns) 			// Note!! period = LinuxCNC SERVO_PERIOD
	{
		old_dtns = period;				// get ready to detect future period changes
		dt = period * 0.000000001; 		// dt is the period of this thread, used for the position loop
		recip_dt = 1.0 / dt;			// calc the reciprocal once here, to avoid multiple divides later
    }

    // loop through generators
	for (i = 0; i < JOINTS; i++)
	{
		// check for scale change
		if (data->pos_scale[i] != data->old_scale[i])
		{
			data->old_scale[i] = data->pos_scale[i];		// get ready to detect future scale changes
			// scale must not be 0
			if ((data->pos_scale[i] < 1e-20) && (data->pos_scale[i] > -1e-20))	// validate the new scale value
				data->pos_scale[i] = 1.0;										// value too small, divide by zero is a bad thing
				// we will need the reciprocal, and the accum is fixed point with
				//fractional bits, so we precalc some stuff
			data->scale_recip[i] = (1.0 / STEP_MASK) / data->pos_scale[i];
		}

		// calculate frequency limit
		//max_freq = PRU_BASEFREQ/(2.0); 	
		max_freq = PRU_base_freq; // step pulses now happen in a single base thread interval


		// check for user specified frequency limit parameter
		if (data->maxvel[i] <= 0.0)
		{
			// set to zero if negative
			data->maxvel[i] = 0.0;
		}
		else
		{
			// parameter is non-zero, compare to max_freq
			desired_freq = data->maxvel[i] * fabs(data->pos_scale[i]);

			if (desired_freq > max_freq)
			{
				// parameter is too high, limit it
				data->maxvel[i] = max_freq / fabs(data->pos_scale[i]);
			}
			else
			{
				// lower max_freq to match parameter
				max_freq = data->maxvel[i] * fabs(data->pos_scale[i]);
			}
		}
		
		/* set internal accel limit to its absolute max, which is
		zero to full speed in one thread period */
		max_ac = max_freq * recip_dt;
		
		// check for user specified accel limit parameter
		if (data->maxaccel[i] <= 0.0)
		{
			// set to zero if negative
			data->maxaccel[i] = 0.0;
		}
		else 
		{
			// parameter is non-zero, compare to max_ac
			if ((data->maxaccel[i] * fabs(data->pos_scale[i])) > max_ac)
			{
				// parameter is too high, lower it
				data->maxaccel[i] = max_ac / fabs(data->pos_scale[i]);
			}
			else
			{
				// lower limit to match parameter
				max_ac = data->maxaccel[i] * fabs(data->pos_scale[i]);
			}
		}

		/* at this point, all scaling, limits, and other parameter
		changes have been handled - time for the main control */

		

		if (data->pos_mode[i]) {

			/* POSITION CONTROL MODE */

			// use Proportional control with feed forward (pgain, ff1gain and deadband)
			
			if (*(data->pgain[i]) != 0)
			{
				pgain = *(data->pgain[i]);
			}
			else
			{
				pgain = 1.0;
			}
			
			if (*(data->ff1gain[i]) != 0)
			{
				ff1gain = *(data->ff1gain[i]);
			}
			else
			{
				ff1gain = 1.0;
			}
			
			if (*(data->deadband[i]) != 0)
			{
				deadband = *(data->deadband[i]);
			}
			else
			{
				deadband = 1 / data->pos_scale[i];
			}	

			// read the command and feedback
			command = *(data->pos_cmd[i]);
			feedback = *(data->pos_fb[i]);
			
			// calcuate the error
			error = command - feedback;
			
			// apply the deadband
			if (error > deadband)
			{
				error -= deadband;
			}
			else if (error < -deadband)
			{
				error += deadband;
			}
			else
			{
				error = 0;
			}
			
			// calcuate command and derivatives
			data->cmd_d[i] = (command - data->prev_cmd[i]) * periodrecip;
			
			// save old values
			data->prev_cmd[i] = command;
				
			// calculate the output value
			vel_cmd = pgain * error + data->cmd_d[i] * ff1gain;
		
		} else {

			/* VELOCITY CONTROL MODE */
			
			// calculate velocity command in counts/sec
			vel_cmd = *(data->vel_cmd[i]);
		}	
			
		vel_cmd = vel_cmd * data->pos_scale[i];
			
		// apply frequency limit
		if (vel_cmd > max_freq) 
		{
			vel_cmd = max_freq;
		} 
		else if (vel_cmd < -max_freq) 
		{
			vel_cmd = -max_freq;
		}
		
		// calc max change in frequency in one period
		dv = max_ac * dt;
		
		// apply accel limit
		if ( vel_cmd > (data->freq[i] + dv) )
		{
			new_vel = data->freq[i] + dv;
		} 
		else if ( vel_cmd < (data->freq[i] - dv) ) 
		{
			new_vel = data->freq[i] - dv;
		}
		else
		{
			new_vel = vel_cmd;
		}
		
		// test for disabled stepgen
		if (*data->stepperEnable == 0) {
			// set velocity to zero
			new_vel = 0; 
		}
		
		data->freq[i] = new_vel;		// to be sent to the PRU
		*(data->freq_cmd[i]) = data->freq[i];	// feedback to LinuxCNC
	}

}


void spi_read()
{
	int i;
	double curr_pos;
	
	// following error spike filter pramaters
	int n = 2;
	int M = 250;

	// Data header
	txData.header = PRU_READ;
	
	if (*(data->PRUreset))
	{ 
		gpiod_line_set_value(reset_line, 1);
    }
	else
	{
		gpiod_line_set_value(reset_line, 0);
    }
	
	
	if (*(data->SPIenable))
	{
		if (*(data->SPIreset) && !(data->SPIresetOld)) data->remoraStatus = 0x00;

		if( (*(data->SPIreset) && !(data->SPIresetOld)) || *(data->SPIstatus) )
		{
			// reset rising edge detected, try SPI transfer and reset OR PRU running

			// Transfer to and from the PRU
			spi_transfer();

			switch (rxData.header & HEADER_MASK)		// only process valid SPI payloads. This rejects bad payloads
			{
				case PRU_DATA:
					// we have received a GOOD payload from the PRU			
					*(data->SPIstatus) = 1;
					
					// Extract,store and check the controller board status
					uint8_t remoraStatus = rxData.header & STATUS_MASK;
					
					checkRemoraStatus(remoraStatus);

					for (i = 0; i < JOINTS; i++)
					{
						// the PRU DDS accumulator uses 32 bit counter, this code converts that counter into 64 bits */
						old_count[i] = count[i];
						count[i] = rxData.jointFeedback[i];
						accum_diff = count[i] - old_count[i];
						
						// spike filter
						if (abs(count[i] - old_count[i]) > M && filter_count[i] < n)
						{
							// recent big change: hold previous value
							++filter_count[i];
							count[i] = old_count[i];
							rtapi_print("Spike filter active[%d][%d]: %d\n", i, filter_count[i], accum_diff);
						}
						else
						{
							// normal operation, or else the big change must be real after all
							filter_count[i] = 0;
						}

						
						*(data->count[i]) = count[i];
						*(data->pos_fb[i]) = (float)(count[i]) / data->pos_scale[i];

					}

					// Feedback
					for (i = 0; i < VARIABLES; i++)
					{
						*(data->processVariable[i]) = rxData.processVariable[i]; 
					}

					// Inputs
					for (i = 0; i < DIGITAL_INPUTS; i++)
					{
						if ((rxData.inputs & (1 << i)) != 0)
						{
							*(data->inputs[i]) = 1; 		// input is high
							*(data->inputs[i+DIGITAL_INPUTS]) = 0;  // inverted
						}
						else
						{
							*(data->inputs[i]) = 0;			// input is low
							*(data->inputs[i+DIGITAL_INPUTS]) = 1;  // inverted
						}
					}
					break;
					
				case PRU_ESTOP:
					// we have an eStop notification from the PRU
					*(data->SPIstatus) = 0;
					 rtapi_print_msg(RTAPI_MSG_ERR, "An E-stop is active");

				default:
					// we have received a BAD payload from the PRU
					*(data->SPIstatus) = 0;

					rtapi_print("Bad SPI payload = %x\n", rxData.header);
					//for (i = 0; i < SPIBUFSIZE; i++) {
					//	rtapi_print("%d\n",rxData.rxBuffer[i]);
					//}
					break;
			}
		}
	}
	else
	{
		*(data->SPIstatus) = 0;
	}
	
	data->SPIresetOld = *(data->SPIreset);
}


void spi_write()
{
	int i;

	// Data header
	txData.header = PRU_WRITE;

	// Joint frequency commands
	for (i = 0; i < JOINTS; i++)
	{
		txData.jointFreqCmd[i] = data->freq[i];
	}

	for (i = 0; i < JOINTS; i++)
	{
		if (*(data->stepperEnable[i]) == 1)
		{
			txData.jointEnable |= (1 << i);		
		}
		else
		{
			txData.jointEnable &= ~(1 << i);	
		}
	}

	// Set points
	for (i = 0; i < VARIABLES; i++)
	{
		txData.setPoint[i] = *(data->setPoint[i]);
	}

	// Outputs
	for (i = 0; i < DIGITAL_OUTPUTS; i++)
	{
		if (*(data->outputs[i]) == 1)
		{
			txData.outputs |= (1 << i);		// output is high
		}
		else
		{
			txData.outputs &= ~(1 << i);	// output is low
		}
	}

	if( *(data->SPIstatus) )
	{
		// Transfer to and from the PRU
		spi_transfer();
	}

}


void spi_transfer()
{
	// send and receive data to and from the Remora PRU concurrently

	int ret;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)txData.txBuffer,
        .rx_buf = (unsigned long)rxData.rxBuffer,
        .len = SPIBUFSIZE,
        .delay_usecs = 0,
        .speed_hz = SPI_freq,
        .bits_per_word = bits,
        .tx_nbits = 1,
        .rx_nbits = 1
    };

    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    
    if (ret < 1)
		rtapi_print_msg(RTAPI_MSG_ERR,"can't send spi message\n");
}

void checkRemoraStatus(uint8_t status)
{
	if (status == data->remoraStatus) return;	// status has not changed
	
	data->remoraStatus = status; // store the updated status
	
    if (status == 0) return;  // No error

    const char* sourceStr = "Unknown Source";
    const char* codeStr = "Unknown Error";

    int fatal = status & 0x80;
    uint8_t source = status & 0x70;
    uint8_t code   = status & 0x0F;

    switch (source)
    {
        case REMORA_SOURCE_CORE:
            sourceStr = "CORE";
            if (code == REMORA_CORE_ERROR) codeStr = "Core error";
            break;

        case REMORA_SOURCE_JSON_CONFIG:
            sourceStr = "JSON Config";
            switch (code)
            {
                case REMORA_SD_MOUNT_FAILED: codeStr = "SD mount failed"; break;
                case REMORA_CONFIG_FILE_OPEN_FAILED: codeStr = "Config file open failed"; break;
                case REMORA_CONFIG_FILE_READ_FAILED: codeStr = "Config file read failed"; break;
                case REMORA_CONFIG_INVALID_INPUT: codeStr = "Invalid config input"; break;
                case REMORA_CONFIG_NO_MEMORY: codeStr = "Config out of memory"; break;
                case REMORA_CONFIG_PARSE_FAILED: codeStr = "Config parse failed"; break;
            }
            break;

        case REMORA_SOURCE_MODULE:
            sourceStr = "Module Loader";
            if (code == REMORA_MODULE_CREATE_FAILED) codeStr = "Module creation failed";
            break;

        case REMORA_SOURCE_TMC_DRIVER:
            sourceStr = "TMC Driver";
            if (code == REMORA_TMC_DRIVER_ERROR) codeStr = "TMC driver error";
            break;
    }

    if (fatal)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Remora FATAL ERROR [%s]: %s (status=0x%02X)\n", sourceStr, codeStr, status);
    }
    else
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Remora ERROR [%s]: %s (status=0x%02X)\n", sourceStr, codeStr, status);
    }
}

static CONTROL parse_ctrl_type(const char *ctrl)
{
    if(!ctrl || !*ctrl || *ctrl == 'p' || *ctrl == 'P') return POSITION;
    if(*ctrl == 'v' || *ctrl == 'V') return VELOCITY;
    return INVALID;
}
