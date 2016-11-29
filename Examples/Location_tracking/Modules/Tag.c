
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "lcd.h"
#include "port.h"
#include "Tag.h"
/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 200

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */

/* Messages for the first ranging process between tag and mother anchor.
	The anchor's address is {"W","A"} and the tag's address is {"V","E"}*/
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Messages for the second ranging process between tag and second anchor.
	The anchor's address is {"R","A"} and the tag's address is {"V","E"}*/
static uint8 tx_poll_msg_2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg_2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'R', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg_2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Messages for the third ranging process between tag and third anchor.
	The anchor's address is {"D","A"} and the tag's address is {"V","E"}*/
static uint8 tx_poll_msg_3[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg_3[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'D', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg_3[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;
static uint64 poll_tx_ts_2;
static uint64 resp_rx_ts_2;
static uint64 final_tx_ts_2;
static uint64 poll_tx_ts_3;
static uint64 resp_rx_ts_3;
static uint64 final_tx_ts_3;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);

/*! ------------------------------------------------------------------------------------------------------------------
*Tag ranging process
*/
void run_tag(void)
{
	/*---------------------------------------------------------------------------------------------------------------------------------------------*/
	/* First ranging sequence */
	/* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
	tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */


	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);




	/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{ };

	/* Increment frame sequence number after transmission of the poll message (modulo 256). */
	frame_seq_nb++;

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32 frame_len;

		/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is the expected response from the companion "DS TWR responder" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 final_tx_time;
			int ret;

			/* Retrieve poll transmission and response reception timestamp. */
			poll_tx_ts = get_tx_timestamp_u64();
			resp_rx_ts = get_rx_timestamp_u64();

			/* Compute final message transmission time. See NOTE 10 below. */
			final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(final_tx_time);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

			/* Write all timestamps in the final message. See NOTE 11 below. */
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

			/* Write and send final message. See NOTE 8 below. */
			tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
			dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
			ret = dwt_starttx(DWT_START_TX_DELAYED);

			/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
			if (ret == DWT_SUCCESS)
			{
				/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
				{ };

				/* Clear TXFRS event. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

				/* Increment frame sequence number after transmission of the final message (modulo 256). */
				frame_seq_nb++;
			}
		}
	}
	else
	{
		/* Clear RX error/timeout events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();
	}

	sleep_ms(RNG_DELAY_MS);
	/* -----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	/*  2nd ranging sequence"*/
	/* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
	tx_poll_msg_2[ALL_MSG_SN_IDX] = frame_seq_nb;
	dwt_writetxdata(sizeof(tx_poll_msg_2), tx_poll_msg_2, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_poll_msg_2), 0, 1); /* Zero offset in TX buffer, ranging. */


	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);




	/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{ };

	/* Increment frame sequence number after transmission of the poll message (modulo 256). */
	frame_seq_nb++;

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32 frame_len;

		/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is the expected response from the companion "DS TWR responder" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_resp_msg_2, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 final_tx_time_2;
			int ret;

			/* Retrieve poll transmission and response reception timestamp. */
			poll_tx_ts_2 = get_tx_timestamp_u64();
			resp_rx_ts_2 = get_rx_timestamp_u64();

			/* Compute final message transmission time. See NOTE 10 below. */
			final_tx_time_2 = (resp_rx_ts_2 + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(final_tx_time_2);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			final_tx_ts_2= (((uint64)(final_tx_time_2 & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

			/* Write all timestamps in the final message. See NOTE 11 below. */
			final_msg_set_ts(&tx_final_msg_2[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts_2);
			final_msg_set_ts(&tx_final_msg_2[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts_2);
			final_msg_set_ts(&tx_final_msg_2[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts_2);

			/* Write and send final message. See NOTE 8 below. */
			tx_final_msg_2[ALL_MSG_SN_IDX] = frame_seq_nb;
			dwt_writetxdata(sizeof(tx_final_msg_2), tx_final_msg_2, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(sizeof(tx_final_msg_2), 0, 1); /* Zero offset in TX buffer, ranging. */
			ret = dwt_starttx(DWT_START_TX_DELAYED);

			/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
			if (ret == DWT_SUCCESS)
			{
				/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
				{ };

				/* Clear TXFRS event. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

				/* Increment frame sequence number after transmission of the final message (modulo 256). */
				frame_seq_nb++;
			}
		}
	}
	else
	{
		/* Clear RX error/timeout events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();
	}

	sleep_ms(RNG_DELAY_MS);
	/* -----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	/*  3d ranging sequence"*/
	/* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
	tx_poll_msg_3[ALL_MSG_SN_IDX] = frame_seq_nb;
	dwt_writetxdata(sizeof(tx_poll_msg_3), tx_poll_msg_3, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_poll_msg_3), 0, 1); /* Zero offset in TX buffer, ranging. */


	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);




	/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{ };

	/* Increment frame sequence number after transmission of the poll message (modulo 256). */
	frame_seq_nb++;

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32 frame_len;

		/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is the expected response from the companion "DS TWR responder" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_resp_msg_3, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 final_tx_time_3;
			int ret;

			/* Retrieve poll transmission and response reception timestamp. */
			poll_tx_ts_3 = get_tx_timestamp_u64();
			resp_rx_ts_3 = get_rx_timestamp_u64();

			/* Compute final message transmission time. See NOTE 10 below. */
			final_tx_time_3 = (resp_rx_ts_3 + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(final_tx_time_3);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			final_tx_ts_3= (((uint64)(final_tx_time_3 & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

			/* Write all timestamps in the final message. See NOTE 11 below. */
			final_msg_set_ts(&tx_final_msg_3[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts_3);
			final_msg_set_ts(&tx_final_msg_3[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts_3);
			final_msg_set_ts(&tx_final_msg_3[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts_3);

			/* Write and send final message. See NOTE 8 below. */
			tx_final_msg_3[ALL_MSG_SN_IDX] = frame_seq_nb;
			dwt_writetxdata(sizeof(tx_final_msg_3), tx_final_msg_3, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(sizeof(tx_final_msg_3), 0, 1); /* Zero offset in TX buffer, ranging. */
			ret = dwt_starttx(DWT_START_TX_DELAYED);

			/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
			if (ret == DWT_SUCCESS)
			{
				/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
				{ };

				/* Clear TXFRS event. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

				/* Increment frame sequence number after transmission of the final message (modulo 256). */
				frame_seq_nb++;
			}
		}
	}
	else
	{
		/* Clear RX error/timeout events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();
	}

	/*-----------------------------------------------------------------------------------------------------------------*/

	/* Execute a delay between ranging exchanges. */
	sleep_ms(RNG_DELAY_MS);

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
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
	int i;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		ts_field[i] = (uint8) ts;
		ts >>= 8;
	}
}
