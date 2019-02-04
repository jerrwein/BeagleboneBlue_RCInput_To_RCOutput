#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"

#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "AM335X_GPIO.h"

extern volatile unsigned int   *gpio0_setdataout_addr;
extern volatile unsigned int   *gpio1_setdataout_addr;
extern volatile unsigned int   *gpio2_setdataout_addr;
extern volatile unsigned int   *gpio3_setdataout_addr;
extern volatile unsigned int   *gpio0_cleardataout_addr;
extern volatile unsigned int   *gpio1_cleardataout_addr;
extern volatile unsigned int   *gpio2_cleardataout_addr;
extern volatile unsigned int   *gpio3_cleardataout_addr;

#define NUM_RING_ENTRIES 300

#define MAX_CMD_PWMS	12

// state of DSM decoder
struct {
     uint16_t bytes[16]; // including start bit and stop bit
     uint16_t bit_ofs;
} dsm_state;

// shared ring buffer with the PRU which records pin transitions
struct ring_buffer {
        volatile uint16_t ring_head; // owned by ARM CPU
        volatile uint16_t ring_tail; // owned by the PRU
        struct {
               uint16_t pin_value;
               uint16_t delta_t;
        } buffer[NUM_RING_ENTRIES];
};

struct pwm_cmd
{
        uint32_t magic;
        uint32_t enmask;     /* enable mask */
        uint32_t offmsk;     /* state when pwm is off */
        uint32_t periodhi[MAX_CMD_PWMS][2];
        uint32_t hilo_read[MAX_CMD_PWMS][2];
        uint32_t enmask_read;
};

volatile struct ring_buffer *ring_buffer;
volatile struct pwm_cmd *sharedMem_pwm_cmd;

static const uint8_t chan_pru_map[]= {0,1,5,6,2,3,4,7,8,9,10,11};

uint16_t _s0_time;

#define LINUX_RC_INPUT_NUM_CHANNELS 16

#define SYSTEM_ID_QSMX_22 0xa2
#define SYSTEM_ID_QSMX_11 0xb2

#define DSM_FRAME_CHANNELS	7		/* Max supported DSM channels */
#define MIN_NUM_CHANNELS 5

#define MIN_START_DELTA 3000

// #define RCIN_PRUSS_SHAREDRAM_BASE   0x4a312000
#define RCIN_PRU0_SHAREDRAM_BASE      0x4a312000
#define RCOUT_PRU1_SHAREDRAM_BASE     0x4a310000

static const int TICK_PER_US = 200;

	// Alta specific
unsigned long _lAltaFramesProcessed = 0;
unsigned long _lAltaConseqSyncedFrames = 0;
bool _bAltaFrameSynced = false;
bool _bAltaGapByteFound = false;
bool _bAltaSysIdByteFound = false;
int  _nAltaFrameBytesIn = 0;
bool _bAltaSkipNextByte;
unsigned long _lAltaSyncLosses = 0;

uint16_t _pwm_values[LINUX_RC_INPUT_NUM_CHANNELS];
uint8_t  _num_channels;

static uint64_t dsm_last_frame_time;		/* Timestamp for start of last dsm frame */
static unsigned dsm_channel_shift;		/* Channel resolution, 0=unknown, 10=10 bit, 11=11 bit */

volatile unsigned int rc_input_count;
//volatile unsigned int last_rc_input_count;

void process_rcin_byte(uint16_t width_s0, uint16_t width_s1);
void process_qsmx_byte(uint16_t data_delta, uint16_t data_val);

static void dsm_guess_format(bool reset, const uint8_t dsm_frame[16]);
static bool dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value);
bool dsm_decode(uint64_t frame_time, const uint8_t dsm_frame[16], uint16_t *values, uint16_t *num_values, uint16_t max_values);
static uint64_t micros64(void);
void timer_tick(void);
void write_ch (uint8_t ch, uint16_t period_us);
int setup_bbbl_gpio (void);


int main(int argc, char **argv, char **envp)
{
	int mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);

	if (mem_fd == -1)
	{
        	printf("Unable to open /dev/mem");
		return -1;
	}

	ring_buffer = (volatile struct ring_buffer*) mmap(0, 0x1000, PROT_READ|PROT_WRITE,
                                                          MAP_SHARED, mem_fd, RCIN_PRU0_SHAREDRAM_BASE);
        if (ring_buffer == NULL)
        {
                printf("Unable to map PRU-0 shared memory");
        }

      	sharedMem_pwm_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE,
                                                    MAP_SHARED, mem_fd, RCOUT_PRU1_SHAREDRAM_BASE);
        if (sharedMem_pwm_cmd == NULL)
        {
                printf("Unable to map PRU-1 shared memory");
        }

	/* Setup GPIO for 6V servo power enable */
//	setup_bbbl_gpio();

	/* Bring up servo power rail */
//	*gpio2_setdataout_addr |= GPIO2_16_PIN;		/* Servo Power Enable */

//	close(mem_fd);
	ring_buffer->ring_head = 0;
	_s0_time = 0;

	printf("Alta-RCInput_PRU::init() PRU memory map complete.\n");

	uint32_t loops = 250000;
	while(loops--)
	{
		usleep (5000);
		timer_tick();
	};

	/* Shutdown servo power rail */
//	*gpio2_cleardataout_addr |= GPIO2_16_PIN;	/* Servo Power Enable */

	close(mem_fd);
}

void timer_tick(void)
{
	while (ring_buffer->ring_head != ring_buffer->ring_tail)
	{
        	if (ring_buffer->ring_tail >= NUM_RING_ENTRIES)
		{
            		// invalid ring_tail from PRU - ignore RC input
			printf("Alta-RCInput_PRU::timer_tick() Invalid tail.\n");
			return;
        	}

		process_rcin_byte (ring_buffer->buffer[ring_buffer->ring_head].pin_value,
                                  ring_buffer->buffer[ring_buffer->ring_head].delta_t);

        	// move to the next ring buffer entry
        	ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;
    	}
}

/*
  process a RC input byte
 */
void process_rcin_byte(uint16_t width_s0, uint16_t width_s1)
{
 	/* Alta Spektrum QSMX operation */
	process_qsmx_byte (width_s0, width_s1);
}


void process_qsmx_byte(uint16_t data_delta, uint16_t data_val)
{
	// Increment incoming byte counter
	_nAltaFrameBytesIn++;

	if (_bAltaFrameSynced)
	{
		// In the middle of frame acquisition
		if ((1 <= _nAltaFrameBytesIn) && (_nAltaFrameBytesIn <= 16))
			dsm_state.bytes[_nAltaFrameBytesIn-1] = data_val;
		else
			printf("Alta: RCInput::process_qsmx_byte() -- FATAL -- frame index (%d) out of range.\n", data_val);

		/* Are we still in SYNC ? */
		if ((_nAltaFrameBytesIn == 2) &&
			(data_val != SYSTEM_ID_QSMX_11) &&
			(data_val != SYSTEM_ID_QSMX_22))
		{
			printf("Alta: RCInput::process_qsmx_byte() -- ERROR -- system ID sync loss, frame: %ld\n", _lAltaFramesProcessed);
			_bAltaFrameSynced = false;
			_bAltaGapByteFound = false;
			_bAltaSysIdByteFound = false;
			_lAltaSyncLosses++;
		}
		else if (_nAltaFrameBytesIn == 16)
		{
			/* We have a 'synced' frame */
			_lAltaConseqSyncedFrames++;
			uint16_t values[8];
			uint16_t num_values = 0;
			uint8_t bytes[16];
			uint8_t i;
			for (i=0; i<16; i++)
				bytes[i] = dsm_state.bytes[i];

//			hal.console->printf("Alta: RCInput::decoding frame()\n");
			if (dsm_decode(micros64(), bytes, values, &num_values, 8) && num_values >= MIN_NUM_CHANNELS)
			{
	//			printf("Decoding frame successful, num_values = %d V=", num_values);
				for (i=0; i<num_values; i++)
				{
					_pwm_values[i] = values[i];
					 write_ch (i, values[i]);
				}
				_num_channels = num_values;
				rc_input_count++;
//				printf("num_values = %d - %d,%d,%d,%d,%d,%d,%d\n", num_values, _pwm_values[0],_pwm_values[1], _pwm_values[2], _pwm_values[3], _pwm_values[4], _pwm_values[5], _pwm_values[6]);
			}
			else
				printf("Alta: RCInput::process_qsmx_byte() -- WARNING -- bad frame decode, frame: %ld\n", _lAltaFramesProcessed);

			/* Reset for start of next frame */
			_lAltaFramesProcessed++;
			_nAltaFrameBytesIn = 0;
		}
	}
	else    /* Not in SYNC */
	{
		_lAltaConseqSyncedFrames = 0;
		/* 'SkipNext' prevents locking into a frame while attempting to sync */
		if (!_bAltaSkipNextByte)
		{
			if (!_bAltaGapByteFound && (MIN_START_DELTA < data_delta))
			{
				_bAltaGapByteFound = true;
				_bAltaSysIdByteFound = false;
				printf("Alta: RCInput::process_sbus_byte(), acquired gap-byte\n");
				_nAltaFrameBytesIn = 1;
			}
			else if (_bAltaGapByteFound && !_bAltaSysIdByteFound && (_nAltaFrameBytesIn==2))
			{
				if ((data_val == SYSTEM_ID_QSMX_11) || (data_val == SYSTEM_ID_QSMX_22))
				{
					_bAltaSysIdByteFound = true;
					printf("Alta: RCInput::process_sbus_byte(), acquired ID-byte\n");
				}
				else
				{
					_bAltaGapByteFound = false;
					_bAltaSysIdByteFound = false;
					_bAltaFrameSynced = false;
				}
			}
			else if (_bAltaGapByteFound && _bAltaSysIdByteFound && (_nAltaFrameBytesIn == 16))
			{
				printf("Alta: RCInput::process_qsmx_byte(), acquired SYNC END\n");
				_bAltaGapByteFound = false;
				_bAltaSysIdByteFound = false;
				_bAltaFrameSynced = true;
				_nAltaFrameBytesIn = 0;
			}
		}
		else
		{
			printf("Alta: RCInput::process_qsmx_byte(), skipping byte\n");
			_bAltaSkipNextByte = false;
		}
	}
}

static uint64_t micros64(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)));
}

static bool dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value)
{

	if (raw == 0xffff)
		return false;

	*channel = (raw >> shift) & 0xf;

	uint16_t data_mask = (1 << shift) - 1;
	*value = raw & data_mask;

	// printf ("DSM: %d 0x%04x -> %d %d", shift, raw, *channel, *value);

	return true;
}

/**
 * Decode the entire dsm frame (all contained channels)
 *
 */
bool dsm_decode(uint64_t frame_time, const uint8_t dsm_frame[16], uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
	typedef union pack_data_t
	{
		uint8_t	byte[2];
		uint16_t word;
	} pack_data_t;
	pack_data_t pd;
	uint16_t frame_word[8];

#if 0
	printf ("%ld: %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x\n",  _lAltaConseqSyncedFrames,
	dsm_frame[0], dsm_frame[1], dsm_frame[2], dsm_frame[3], dsm_frame[4], dsm_frame[5], dsm_frame[6], dsm_frame[7],
	dsm_frame[8], dsm_frame[9], dsm_frame[10], dsm_frame[11], dsm_frame[12], dsm_frame[13], dsm_frame[14], dsm_frame[15]);
#endif

#if 1
	pd.byte[0] = dsm_frame[1];	pd.byte[1] = dsm_frame[0];
	frame_word[0] = pd.word;
	pd.byte[0] = dsm_frame[3];	pd.byte[1] = dsm_frame[2];
	frame_word[1] = pd.word & 0x7ff;
	pd.byte[0] = dsm_frame[5];	pd.byte[1] = dsm_frame[4];
	frame_word[2] = pd.word & 0x7ff;
	pd.byte[0] = dsm_frame[7];	pd.byte[1] = dsm_frame[6];
	frame_word[3] = pd.word & 0x7ff;
	pd.byte[0] = dsm_frame[9];	pd.byte[1] = dsm_frame[8];
	frame_word[4] = pd.word & 0x7ff;
	pd.byte[0] = dsm_frame[11];	pd.byte[1] = dsm_frame[10];
	frame_word[5] = pd.word & 0x7ff;
	pd.byte[0] = dsm_frame[13];	pd.byte[1] = dsm_frame[12];
	frame_word[6] = pd.word & 0x7ff;
	pd.byte[0] = dsm_frame[15];	pd.byte[1] = dsm_frame[14];
	frame_word[7] = pd.word & 0x7ff;

	printf ("%ld: %04x %03x %03x %03x %03x %03x %03x %03x\n",  _lAltaConseqSyncedFrames, frame_word[0], frame_word[1], frame_word[2], frame_word[3], frame_word[4],frame_word[5], frame_word[6], frame_word[7]);

#endif

	/*
	 * If we have lost signal for at least a second, reset the
	 * format guessing heuristic.
	 */
	if (((frame_time - dsm_last_frame_time) > 1000000) && (dsm_channel_shift != 0))
            dsm_guess_format(true, dsm_frame);

	/* we have received something we think is a dsm_frame */
	dsm_last_frame_time = frame_time;

	/* if we don't know the dsm_frame format, update the guessing state machine */
	if (dsm_channel_shift == 0) {
            dsm_guess_format(false, dsm_frame);
            return false;
	}

	/*
	 * The encoding of the first two bytes is uncertain, so we're
	 * going to ignore them for now.
	 *
	 * Each channel is a 16-bit unsigned value containing either a 10-
	 * or 11-bit channel value and a 4-bit channel number, shifted
	 * either 10 or 11 bits. The MSB may also be set to indicate the
	 * second dsm_frame in variants of the protocol where more than
	 * seven channels are being transmitted.
	 */

	for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

		const uint8_t *dp = &dsm_frame[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		if (!dsm_decode_channel(raw, dsm_channel_shift, &channel, &value))
			continue;

		/* ignore channels out of range */
		if (channel >= max_values)
			continue;

		/* update the decoded channel count */
		if (channel >= *num_values)
			*num_values = channel + 1;

		/* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding. */
		if (dsm_channel_shift == 10)
			value *= 2;

		/*
		 * Spektrum scaling is special. There are these basic considerations
		 *
		 *   * Midpoint is 1520 us
		 *   * 100% travel channels are +- 400 us
		 *
		 * We obey the original Spektrum scaling (so a default setup will scale from
		 * 1100 - 1900 us), but we do not obey the weird 1520 us center point
		 * and instead (correctly) center the center around 1500 us. This is in order
		 * to get something useful without requiring the user to calibrate on a digital
		 * link for no reason.
		 */

		/* scaled integer for decent accuracy while staying efficient */
//		value = ((((int)value - 1024) * 1000) / 1700) + 1500;
		/* The following gives [1000ms,2000ms] for counts of [100,1900] */
//		value =  (((int)value * 1000) / 1800) + 945;
		/* The following gives [1100ms,1900ms] for counts of [100,1900] */
		value =  (((int)value * 800) / 1800) + 1056;

		/*
		 * Store the decoded channel into the R/C input buffer, taking into
		 * account the different ideas about channel assignement that we have.
		 *
		 * Specifically, the first four channels in rc_channel_data are roll, pitch, thrust, yaw,
		 * but the first four channels from the DSM receiver are thrust, roll, pitch, yaw.
		 */
/* Alta Specific - may be re-instated later */
		switch (channel) {
		case 0:
			channel = 2;
			break;

		case 1:
			channel = 0;
			break;

		case 2:
			channel = 1;

		default:
			break;
		}
/* Alta Specific */
		values[channel] = value;
	}

	/*
	 * Spektrum likes to send junk in higher channel numbers to fill
	 * their packets. We don't know about a 13 channel model in their TX
	 * lines, so if we get a channel count of 13, we'll return 12 (the last
	 * data index that is stable).
	 */
	if (*num_values == 13)
		*num_values = 12;

#if 0
	if (dsm_channel_shift == 11) {
		/* Set the 11-bit data indicator */
		*num_values |= 0x8000;
	}
#endif

	/*
	 * XXX Note that we may be in failsafe here; we need to work out how to detect that.
	 */
	return true;
}

/*
 * Attempt to guess if receiving 10 or 11 bit channel values
 *
 * @param[in] reset true=reset the 10/11 bit state to unknown
 */
static void dsm_guess_format(bool reset, const uint8_t dsm_frame[16])
{
	static uint32_t	cs10;
	static uint32_t	cs11;
	static unsigned samples;

	 printf("Alta: dsm_guess_format(), reset=%d\n", reset);
	/* reset the 10/11 bit sniffed channel masks */
	if (reset) {
		cs10 = 0;
		cs11 = 0;
		samples = 0;
		dsm_channel_shift = 0;
		return;
	}

	/* scan the channels in the current dsm_frame in both 10- and 11-bit mode */
	for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

		const uint8_t *dp = &dsm_frame[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		/* if the channel decodes, remember the assigned number */
		if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 31))
			cs10 |= (1 << channel);

		if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 31))
			cs11 |= (1 << channel);

		/* XXX if we cared, we could look for the phase bit here to decide 1 vs. 2-dsm_frame format */
	}

	/* wait until we have seen plenty of frames - 5 should normally be enough */
	if (samples++ < 5)
		return;

	/*
	 * Iterate the set of sensible sniffed channel sets and see whether
	 * decoding in 10 or 11-bit mode has yielded anything we recognize.
	 *
	 * XXX Note that due to what seem to be bugs in the DSM2 high-resolution
	 *     stream, we may want to sniff for longer in some cases when we think we
	 *     are talking to a DSM2 receiver in high-resolution mode (so that we can
	 *     reject it, ideally).
	 *     See e.g. http://git.openpilot.org/cru/OPReview-116 for a discussion
	 *     of this issue.
	 */
	static uint32_t masks[] = {
		0x3f,	/* 6 channels (DX6) */
		0x7f,	/* 7 channels (DX7) */
		0xff,	/* 8 channels (DX8) */
		0x1ff,	/* 9 channels (DX9, etc.) */
		0x3ff,	/* 10 channels (DX10) */
		0x1fff,	/* 13 channels (DX10t) */
		0x3fff	/* 18 channels (DX10) */
	};
	unsigned votes10 = 0;
	unsigned votes11 = 0;

	for (unsigned i = 0; i < sizeof(masks)/sizeof(masks[0]); i++) {

		if (cs10 == masks[i])
			votes10++;

		if (cs11 == masks[i])
			votes11++;
	}

	if ((votes11 == 1) && (votes10 == 0)) {
		dsm_channel_shift = 11;
		printf ("-- DSM: 11-bit format --\n");
		return;
	}

	if ((votes10 == 1) && (votes11 == 0)) {
		dsm_channel_shift = 10;
		printf ("-- DSM: 10-bit format --\n");
		return;
	}

	/* call ourselves to reset our state ... we have to try again */
	printf ("DSM: format detect fail, 10: 0x%08x %d 11: 0x%08x %d", cs10, votes10, cs11, votes11);
	dsm_guess_format(true, dsm_frame);
}

void write_ch (uint8_t ch, uint16_t period_us)
{
//      if (corked)
//      {
//              pending[ch] = period_us;
//              pending_mask |= (1U << ch);
//      }
//      else
//      {
                sharedMem_pwm_cmd->periodhi[chan_pru_map[ch]][1] = TICK_PER_US*period_us;
//      }
}

int setup_bbbl_gpio (void)
{
	gpio_fast_init (0);

	// Export GPIO2.16 - pin #80 --- SERVO POWER Enable for output
	int gpio_num = 80; 	/* gpio2[16] */
	gpio_export (gpio_num);
	/* Set direction as an output */
	gpio_set_dir (gpio_num, GPIO_OUTPUT);

	return 0;
}
