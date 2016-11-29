#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "lcd.h"
#include "port.h"
#include "Mother_anchor.h"


/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */


/* Messages for the ranging process between tag and mother anchor.
	The anchor's address is {"W","A"} and the tag's address is {"V","E"}*/
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Message from second anchor to mother anchor containing the timestamps from that ranging process */
static uint8 rx_ranging_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'R', 'A', 0x21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0, 0};

/* Message from third anchor to mother anchor containing the timestamps from that ranging process */
static uint8 rx_ranging_msg_2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'D', 'A', 0x21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0, 0};


/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TEST_DATA 22
#define FINAL_MSG_TS_LEN 4
#define RANGING_MSG_IDX 10
#define RANGING_MSG_POLL_RX_IDX 22
#define RANGING_MSG_RESP_TX_IDX 26
#define RANGING_MSG_FINAL_RX_IDX 30
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;


/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 25
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3300
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double tof_2;
static double tof_3;

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str_mother[16] = {0};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);



/*-------------------------------------------------------------------------------------*/
	/* Start of Mother anchor process*/
int run_mother_anchor(void)
{
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(0);

	/* Activate reception immediately. */
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{ };

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32 frame_len;

		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUFFER_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is a poll sent by "DS TWR initiator" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 resp_tx_time;
			int ret;

			/* Retrieve poll reception timestamp. */
			poll_rx_ts = get_rx_timestamp_u64();

			/* Set send time for response. See NOTE 9 below. */
			resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(resp_tx_time);

			/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
			dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
			dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

			/* Write and send the response message. See NOTE 10 below.*/
			tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
			dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

			/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
			if (ret == DWT_ERROR)
			{
				return 0;
			}

			/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
			{ };

			/* Increment frame sequence number after transmission of the response message (modulo 256). */
			frame_seq_nb++;

			if (status_reg & SYS_STATUS_RXFCG)
			{
				/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUF_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len, 0);
				}

				/* Check that the frame is a final message sent by "DS TWR initiator" example.
				 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
				rx_buffer[ALL_MSG_SN_IDX] = 0;

				if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
				{
					uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
					uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
					double Ra, Rb, Da, Db;
					int64 tof_dtu;

					/* Retrieve response transmission and final reception timestamps. */
					resp_tx_ts = get_tx_timestamp_u64();
					final_rx_ts = get_rx_timestamp_u64();

					/* Get timestamps embedded in the final message. */
					final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

					uint8 testData = rx_buffer[FINAL_MSG_TEST_DATA];

					/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
					poll_rx_ts_32 = (uint32)poll_rx_ts;
					resp_tx_ts_32 = (uint32)resp_tx_ts;
					final_rx_ts_32 = (uint32)final_rx_ts;
					Ra = (double)(resp_rx_ts - poll_tx_ts);
					Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
					Da = (double)(final_tx_ts - resp_rx_ts);
					Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
					tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));



					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;

					/* Display computed distance on LCD. */
					//sprintf(dist_str_mother, "DIST B: %3.2f m", distance);
					//lcd_display_str(dist_str_mother);

				}
			}
			else
			{
				/* Clear RX error/timeout events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

				/* Reset RX to properly reinitialise LDE operation. */
				dwt_rxreset();
			}
		}
		/*---------------------------------------------------Distance 2--------------------------------------------------------------------------------------*/
		/* Calculate the distance the tag and the second anchor. */

		if (memcmp(rx_buffer, rx_ranging_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 poll_tx_ts_2, resp_rx_ts_2, final_tx_ts_2;
			uint32 poll_rx_ts_2, resp_tx_ts_2, final_rx_ts_2;
			double Ra_2, Rb_2, Da_2, Db_2;
			int64 tof_dtu_2;

			/* Get timestamps embedded in the final message. */
			/* Here we have to also retrive the timesamps from the second anchor and not only the tag. */
			final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts_2);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts_2);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_2);
			final_msg_get_ts(&rx_buffer[RANGING_MSG_POLL_RX_IDX], &poll_rx_ts_2);
			final_msg_get_ts(&rx_buffer[RANGING_MSG_RESP_TX_IDX], &resp_tx_ts_2);
			final_msg_get_ts(&rx_buffer[RANGING_MSG_FINAL_RX_IDX], &final_rx_ts_2);


			/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
			Ra_2 = (double)(resp_rx_ts_2 - poll_tx_ts_2);
			Rb_2 = (double)(final_rx_ts_2 - resp_tx_ts_2);
			Da_2 = (double)(final_tx_ts_2 - resp_rx_ts_2);
			Db_2 = (double)(resp_tx_ts_2 - poll_rx_ts_2);



			tof_dtu_2 = (int64)((Ra_2 * Rb_2 - Da_2 * Db_2) / (Ra_2 + Rb_2 + Da_2 + Db_2));

			tof_2 = tof_dtu_2 * DWT_TIME_UNITS;
			distance_2nd = tof_2 * SPEED_OF_LIGHT;

		}

		/*---------------------------------------------------Distance 3--------------------------------------------------------------------------------------*/
		/* Calculate the distance the tag and the third anchor. */


		if (memcmp(rx_buffer, rx_ranging_msg_2, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 poll_tx_ts_3, resp_rx_ts_3, final_tx_ts_3;
			uint32 poll_rx_ts_3, resp_tx_ts_3, final_rx_ts_3;
			double Ra_3, Rb_3, Da_3, Db_3;
			int64 tof_dtu_3;

			/* Get timestamps embedded in the final message. */
			/* Here we have to also retrive the timesamps from the third anchor and not only the tag. */
			final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts_3);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts_3);
			final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_3);
			final_msg_get_ts(&rx_buffer[RANGING_MSG_POLL_RX_IDX], &poll_rx_ts_3);
			final_msg_get_ts(&rx_buffer[RANGING_MSG_RESP_TX_IDX], &resp_tx_ts_3);
			final_msg_get_ts(&rx_buffer[RANGING_MSG_FINAL_RX_IDX], &final_rx_ts_3);


			// uint8 testData = rx_buffer[FINAL_MSG_TEST_DATA];

			/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
			Ra_3 = (double)(resp_rx_ts_3 - poll_tx_ts_3);
			Rb_3 = (double)(final_rx_ts_3 - resp_tx_ts_3);
			Da_3 = (double)(final_tx_ts_3 - resp_rx_ts_3);
			Db_3 = (double)(resp_tx_ts_3 - poll_rx_ts_3);



			tof_dtu_3 = (int64)((Ra_3 * Rb_3 - Da_3 * Db_3) / (Ra_3 + Rb_3 + Da_3 + Db_3));

			tof_3 = tof_dtu_3 * DWT_TIME_UNITS;
			distance_3 = tof_3 * SPEED_OF_LIGHT;
		}
		sprintf(dist_str_mother, "DIST B: %3.2f m", distance);
		lcd_display_str(dist_str_mother);
		char test_str[16] = {0};
		sprintf(test_str, "D%3.2f; C%3.2f", distance_2nd, distance_3);
		writetoLCD(strlen(test_str), 1, (const uint8 *)test_str);

		/* Send data to computer using USB*/
/*		int n = sprintf(dist_str_mother_2, "D1: %i D2: %i D3: %i", (int)(distance*1000),(int)(distance_2nd*1000),(int)(distance_3*1000));
		memcpy(dataseq, (const uint8 *) dist_str_2, n);
		send_usbmessage(&dataseq, n);

		usb_run();*/

		//sleep_ms(100);
		return 1; /*If a message process has been performed. */
	}

	else
	{
		/* Clear RX error/timeout events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();

		return 0;
	}


}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
	int i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		*ts += ts_field[i] << (i * 8);
	}
}

