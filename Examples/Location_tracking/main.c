
#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "lcd.h"
#include "port.h"
#include "Mother_anchor.h"
#include "Anchor1.h"
#include "Anchor2.h"
#include "Tag.h"

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150

#define RESP_RX_TIMEOUT_UUS 2700
/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
		2,               /* Channel number. */
		DWT_PRF_64M,     /* Pulse repetition frequency. */
		DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
		DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
		9,               /* TX preamble code. Used in TX only. */
		9,               /* RX preamble code. Used in RX only. */
		1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
		DWT_BR_110K,     /* Data rate. */
		DWT_PHRMODE_STD, /* PHY header mode. */
		(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


typedef signed long long int64;
typedef unsigned long long uint64;
int new_distances;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
/* Selection of what mode the board should act as.*/
int mode_selection(void)
{
	int mode = 0; /* Mother Anchor*/

	if(is_switch_on(TA_SW1_3) == 0)
	{
		mode = 1; /* Tag*/
	}

	if(is_switch_on(TA_SW1_4) == 0)
	{
		mode = 2; /* Anchor 1*/
	}

	if(is_switch_on(TA_SW1_5) == 0)
	{
		mode = 3; /* Anchor 2*/
	}


	return mode;
}

int main(void)
{
	/* Start with board specific hardware init. */
	peripherals_init();
	uint8 dataseq[32];
	uint8 dataseq2[15];
	usb_init();
	sleep_ms(1000);
	char dist_str_2[32] = "10.3";
	/* Display application name on LCD. */
	int mode = mode_selection();
	if (mode == 0)
	{
#define APP_NAME "MOTHER v1.0"
	}
	if (mode == 1)
	{
#define APP_NAME "Tag v1.0"
	}
	if (mode == 2)
	{
#define APP_NAME "Anchor1 v1.0"
	}
	if (mode == 3)
	{
#define APP_NAME "Anchor2 v1.0"
	}


	/* Reset and initialise DW1000.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	 * performance. */
	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	spi_set_rate_low();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		lcd_display_str("INIT FAILED");
		while (1)
		{ };
	}
	spi_set_rate_high();

	/* Configure DW1000. See NOTE 7 below. */
	dwt_configure(&config);

	/* Apply default antenna delay value. See NOTE 1 below. */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
	if (mode == 1)
	{
		/* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
		 * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
		dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
		dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	}

	/* Set preamble timeout for expected frames. See NOTE 6 below. */
	dwt_setpreambledetecttimeout(PRE_TIMEOUT);


	/* Loop forever responding to ranging requests. */
	while (1)
	{
		/* Depending on which module the bord should act as there is different methods, the module is determined
		 * by which swith on SW1 is turned on.
		 */
		if (mode == 0)
		{
			/* Mother anchor ranging process*/
			new_distances = run_mother_anchor();
			if (new_distances == 1)
			{
				int n = sprintf(dist_str_2, "D1: %i D2: %i D3: %i", (int)(distance*1000),(int)(distance_2nd*1000),(int)(distance_3*1000));
				memcpy(dataseq, (const uint8 *) dist_str_2, n);
				send_usbmessage(&dataseq, n);
				usb_run(); /* Send the distances if there are new values received.*/
			}
		}
		if (mode == 1)
		{
			/* Tag ranging process*/
			run_tag();
		}
		if (mode == 2)
		{
			/* Anchor 1 ranging process*/
			run_anchor1();
		}
		if (mode == 3)
		{
			/* Anchor 1 ranging process*/
			run_anchor2();
		}

	}
}
