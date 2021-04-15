/* Repeat8 component
 */

#include "global.h"

#ifdef ENABLE_REPEAT8
#include "comm.h"
#include "repeat8.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "xpio.h"

#define TAG "repeat8"

static const byte pcf0_addr = 0x20;										// top PCF8574
static const byte pcf1_addr = 0x21;										// bottom

#define MAX_CHANNEL 15													// analog channels
#define HIGHEST_CHANNEL 11												// only sample channels 0..11
// channels 0..7 are the port inputs
#define CHANNEL_LOW0	 8
#define CHANNEL_LOW1	 9
#define CHANNEL_HIGH0	10
#define CHANNEL_HIGH1	11

static const char* channel_names[MAX_CHANNEL] = {
	"port0", "port1", "port2", "port3", "port4", "port5", "port6", "port7",
	"refL0", "refL1", "refH0", "refH1",
};

typedef enum {
	channel_mode_idle,
	channel_mode_degc,
	channel_mode_degf,
	channel_mode_volt,
	channel_mode_ref_low,
	channel_mode_ref_high,
	channel_mode_button,
	channel_mode_resistor,
} channel_mode_t;

static const char *channel_mode_names[] = {
	"idle", "degc", "degf", "volt", "refl", "refh", "button", "resistor"
};

typedef struct {
	const char *name;							// "channel 1"
	channel_mode_t mode;
	const char* mode_name;						// "degc"
	int avg;									// moving average
	int lastavg;								// change detection
	int val;									// temp or voltage, *100
} channel_data_t;

static channel_data_t channel_data[MAX_CHANNEL];

typedef union {															// top PCF8574 pins 0x20
	struct {
		unsigned s012:3;
		unsigned s345:3;
		unsigned buzzer:1;
		unsigned led:1;
	};
	byte reg;
} pcf0_reg_t;

typedef union {															// // bottom PCF8574 pins 0x21
	struct {
		unsigned ntc5:1;
		unsigned ntc6:1;
		unsigned ntc7:1;
		unsigned ntc8:1;
		unsigned ntc1:1;
		unsigned ntc2:1;
		unsigned ntc3:1;
		unsigned ntc4:1;
	};
	byte reg;
} pcf1_reg_t;

static pcf0_reg_t pcf0_reg;
static pcf1_reg_t pcf1_reg;

typedef enum {

} channel_t;

static DRAM_ATTR TaskHandle_t repeat8_task_h;							// task handle
static void repeat8_task(void *pvParameters);
static void init_adc();
static int read_adc_256();
static int convert_adc256_to_mv(int val);
void calibrate(int countl, int counth);
int scale(int avg);
int ntcLookup(int mvi, int *pt, int *pr);				// returns temp and resistance
void setChannelMode(int ch, channel_mode_t mode);
void setChannelMux(int ch);
//void autodetect();
void measure_channel(int ch);
void compute_channel(int ch);
static void set_Led(int led, int on);									// turn a led on/off
static void setBuzzer(int on);											// turn buzzer on/off
static void set4051addr(int addr);										// set address pins of 4051
static void write_pcf0_value();
static void write_pcf1_value();

//const int ana_count = 8;												// analog inputs, connected to 4051
//int ana_mv[ana_count];													// value in mV

Repeat8::Repeat8() : Component("Repeat8", FT_repeat8_module) {
}
Repeat8::~Repeat8() {}

int Repeat8::init() {
	LOG("Repeat8::init()");
	int err;

	if (!enabled.get())
		return 1;

	enabled.init(this, FT_enabled_module, "enable" );
	for (int n=0; n<MAX_PORT; n++) {
		ports[n].init(this, FT_port_module, "port");
		ports[n].val.init(&ports[n], FT_repeat8val_module, "val");
	}
	activate();													// this starts automatic sending of the fields to console/website

	i2c = i2cMasters[i2cbus];
	if ((err = i2c->scan_locked(pcf0_addr)) != 0) {
		ERR("PCF8574-0 not found, err=%d  %s", err, esp_err_to_name(err));
		return err;
	}
	if ((err = i2c->scan_locked(pcf1_addr)) != 0) {
		ERR("PCF8574-1 not found, err=%d  %s", err, esp_err_to_name(err));
		return err;
	}

	for (int ch=0; ch<MAX_CHANNEL; ch++) {
		auto d = channel_data+ch;
		d->name = channel_names[ch];
		d->avg = -1;		// force initial load
		d->val = 0;
		d->lastavg = -1;	// force change
		setChannelMode(ch, channel_mode_idle);
	}

	// set channels that are not ports
	setChannelMode(CHANNEL_LOW0, channel_mode_ref_low);
	setChannelMode(CHANNEL_LOW1, channel_mode_idle);
	setChannelMode(CHANNEL_HIGH0, channel_mode_ref_high);
	setChannelMode(CHANNEL_HIGH1, channel_mode_idle);

	// set ports 1-8. this also sets the channel mode
	setPortMode(0, port_mode_degc);
	setPortMode(1, port_mode_degc);
	setPortMode(2, port_mode_degc);
	setPortMode(3, port_mode_degc);
	setPortMode(4, port_mode_degc);
	setPortMode(5, port_mode_degc);
	setPortMode(6, port_mode_volt);
	setPortMode(7, port_mode_volt);

	setModeBits();												// set pcf1 outputs to volt/temp

	init_adc();													// adc
	return 0;
}

void Repeat8::setPortMode(int port, int mode) {
	if (port>= 0 && port < MAX_PORT) {
		ports[port].val.setmode(mode);

		switch(mode) {
		case port_mode_volt:
			setChannelMode(port, channel_mode_volt);
			break;
		case port_mode_degc:
			setChannelMode(port, channel_mode_degc);
			break;
		case port_mode_degf:
			setChannelMode(port, channel_mode_degf);
			break;
		case port_mode_resistor:
			setChannelMode(port, channel_mode_resistor);
			break;
		case port_mode_button:
			setChannelMode(port, channel_mode_button);
			break;
		default:
			setChannelMode(port, channel_mode_idle);
		}
	}
}
void setChannelMode(int ch, channel_mode_t mode) {
	auto d = &channel_data[ch];
	d->mode = mode;
	d->mode_name = channel_mode_names[mode];
}

int Repeat8::start() {
	LOG("Repeat8::start()");
//	setBuzzer(1);

	xTaskCreatePinnedToCore(repeat8_task, "repeat8_task", 4*1024/*bytes*/, NULL, 5, &repeat8_task_h, 0);
	return 0; // ok
}

// optional. app.poll() in main() calls all started components
// for slow untimed tasks

void Repeat8::poll() {
}

void Repeat8::valuechanged(Value *val) {							// called on parent when changed
	for (int p=0; p<MAX_PORT; p++) {
		if (val == &(ports[p].val)) {
			LOG("value %d changed", p);
		}
	}
}

void Repeat8::setModeBits() {
	// set the mosfet in the analog inputs to either volt or temp
	// strange code because of the pcf bit order
	pcf1_reg.reg = 0;												// 0 = NTC, 1=volts
	pcf1_reg.ntc1 = ports[0].val.getmode() == port_mode_volt;
	pcf1_reg.ntc2 = ports[1].val.getmode() == port_mode_volt;
	pcf1_reg.ntc3 = ports[2].val.getmode() == port_mode_volt;
	pcf1_reg.ntc4 = ports[3].val.getmode() == port_mode_volt;
	pcf1_reg.ntc5 = ports[4].val.getmode() == port_mode_volt;
	pcf1_reg.ntc6 = ports[5].val.getmode() == port_mode_volt;
	pcf1_reg.ntc7 = ports[6].val.getmode() == port_mode_volt;
	pcf1_reg.ntc8 = ports[7].val.getmode() == port_mode_volt;
	write_pcf1_value();
}

//#define CONV_ADC 1										// use esp adc conversion
const int SETTLE_TIME = 100;								// ms settle time after a channel switch
const int OVERRIDE_DELTA = (10*256);						// overrides avg (counts*256)
//const int PRINT_DELTA = 128; 	// (0.5*256);				// calc when avg count changes (counts*256)
//const int VREF_DELTA = (128);	// 0.5*256					// calc when avg count changes (counts*256)
const int vref= 2.495;

static void repeat8_task(void *pvParameters) {
	vTaskDelay(500);										// wait for everybody

	int led = 0;
	int slowch = 0;

	LOG("Calibration");
	for (int i=0; i<16; i++) {								// Sample the reference voltages and compute initial calibration
		for (int ch=0; ch<MAX_CHANNEL; ch++) {
			if (ch == CHANNEL_LOW0 || ch == CHANNEL_HIGH0) {
				auto d = &channel_data[ch];
				setChannelMux(ch);							// set PCF8574 pins
				vTaskDelay(SETTLE_TIME);					// let analog filter settle
				int val = read_adc_256();					// read with oversampling. output is counts*256 for more resolution
				if (i == 0)
					d->avg = val;							// initial value
				else
					d->avg = (d->avg*15 + val) / 16;		// heavy moving average
				vTaskDelay(10);
			}
		}
	}

	// compute offset/slope according to the low,high ref voltages
#ifdef CONV_ADC
	mvl = convert_adc256_to_mv(channel_data[CHANNEL_LOW0].avg) * 2;
	mvh = convert_adc256_to_mv(channel_data[CHANNEL_HIGH0].avg) * 2;
	calibrate(mvl, mvh);
#else
	// range mv*10
	calibrate(channel_data[CHANNEL_LOW0].avg, channel_data[CHANNEL_HIGH0].avg);
#endif

//	LOG("Autodetect");
//	autodetect();

	// Measure time is 100ms per channel.
	// Oversampling 8x per measurement
	// Voltages are samples more often, and averaged less
	// Average is 4x for voltage, 16x for temp and vref

	LOG("Running");
	while(true) {
		set_Led(0, led);
		led = !led;

		for (int fastch=0; fastch<MAX_CHANNEL; fastch++) {			// measure all voltage/temp inputs
			auto d = &channel_data[fastch];
			if (d->mode == channel_mode_volt || d->mode == channel_mode_button || d->mode == channel_mode_degc || d->mode == channel_mode_degf ) {
				measure_channel(fastch);
			}
		}

		for (int i=0; i<MAX_CHANNEL; i++) {							// measure one temp/vref channel
			slowch++;
			if (slowch >= MAX_CHANNEL)
				slowch = 0;

			auto d = &channel_data[slowch];
			if (d->mode == channel_mode_resistor || d->mode == channel_mode_ref_low || d->mode == channel_mode_ref_high) {
				measure_channel(slowch);
				break;
			}
		}
	}
}

void measure_channel(int ch) {
//	LOG("measure %d", ch);

	if (ch < 0 || ch >= MAX_CHANNEL)
		return;
	auto d = &channel_data[ch];

	setChannelMux(ch);										// set PCF8574 pins to select the multiplexer
	vTaskDelay(SETTLE_TIME);								// let analog filter settle (100ms)
	int val = read_adc_256();								// read with oversampling. output is counts*256 for more resolution

	if (d->avg == -1) {										// preload avg
		d->avg = val;
	} else {
		if (ch >= 0 && ch < MAX_PORT) {						// analog inputs (volt and temp)
															// moving average with override,
			if (val < d->avg-OVERRIDE_DELTA || val > d->avg+OVERRIDE_DELTA) {	// 10 counts
				LOG("%s override avg=%d val=%d", d->name, d->avg, val);
//				d->avg = (d->avg + val) / 2;				// 50% override
				d->avg = val;								// 100% override
			} else {
				d->avg = (d->avg*7 + val) / 8;				// 12% average
			}
			// only compute after a big enough count change
//			if (d->avg < d->lastavg-PRINT_DELTA || d->avg > d->lastavg+PRINT_DELTA) {
//				d->lastavg = d->avg;
				compute_channel(ch);
//			}
		} else {											// vref's: heavy avg, no override
			d->avg = (d->avg*31 + val) / 32;
//			if (d->avg < d->lastavg-VREF_DELTA || d->avg > d->lastavg+VREF_DELTA) {
//				d->lastavg = d->avg;
				compute_channel(ch);
//			}
		}
	}

//	LOG("measure %02d val=%6d avg=%6d", ch, val, d->avg);

}

void compute_channel(int ch) {
	Repeat8 &fx = Repeat8::get();
	auto d = &channel_data[ch];

// convert to mV on the mux input.
#ifdef CONV_ADC
	// after the filter we divide by 2
	float mv = convert_adc256_to_mv(d->avg) * 2;
#else
	// dont use the esp linearisation function, we have our own vref's
	int mvi = scale(d->avg);								// acd*256 -> mv*10
#endif

#define INV_TOO_LOW 1
#define INV_TOO_HIGH 2
#define INV_DISCONNECTED 3
#define INV_OUTRANGE_LOW 4
#define INV_OUTRANGE_HIGH 5

	if (ch >= 0 && ch < MAX_PORT) {									// channels 0..7 are also ports
		Port *p = &fx.ports[ch];
		auto f = &p->val;

		switch(d->mode) {
		case channel_mode_degc:
		case channel_mode_degf: {
			// v = 2.495v * (10k+R) / (10k+10k+R)
			// see lookup table spreadsheet
			// -20deg = 86.43k=22675, 100deg=
			// open = 24950

//			if (ch == 0)
//				LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi", ch, d->name, d->mode_name, d->avg, mvi);

			if (mvi > 24750) {										// 2.495V
				if (f->seterror(INV_DISCONNECTED))
					LOG("ch %d disconnected %d", ch, mvi);
			} else {
				int t, r;
				int res = ntcLookup(mvi, &t, &r);					// mv*10 to degc*1000
				if (res < 0) {
					if (f->seterror(INV_OUTRANGE_LOW))
						LOG("ch %d out of range low %d", ch, mvi);
				} else if (res > 0) {
					if (f->seterror(INV_OUTRANGE_HIGH, 0))
						LOG("ch %d out of range high %d", ch, mvi);
				} else {
					int val = t/10.0;								// degc*100
					if (val < -10) {								// -0.1 ... 50
						if (f->setinvalid(val, INV_TOO_LOW, 0))
							LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi %6.2fkR %6.2fC too low", ch, d->name, d->mode_name, d->avg, mvi, r/1000.0, val/100.0);
					} else if (val > 5000) {
						if (f->setinvalid(val, INV_TOO_HIGH, 0))
							LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi %6.2fkR %6.2fC too high", ch, d->name, d->mode_name, d->avg, mvi, r/1000.0, val/100.0);
					} else {
						if (f->change(val))
							LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi %6.2fkR %6.2fC", ch, d->name, d->mode_name, d->avg, mvi, r/1000.0, val/100.0);
					}
				}
			}
		}	break;
		case channel_mode_volt: {
			// input is mv*10
			// 0v -> 382.4mV
			// 10v -> 1912.2mV
			// a = (y1-y2)/(x1-x2) = 10/(19122-3824) = 6.5368e-4
			// b = y - a*x
			const float A = 6.5368e-4;						// mvi to mV
			const float B = -2.4997;
			float res = A*mvi + B;							// measured V
			int val = res*100;								// integer, v, 2 dec
//			LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi %f", ch, d->name, d->mode_name, d->avg, mvi, res);
			if (val < -50) {								// -50mV is ok
				if (f->setinvalid(val, INV_TOO_LOW, 0))
					LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi %6.2fV TOO LOW", ch, d->name, d->mode_name, d->avg, mvi, val/100.0);
			} else if (val > 1050) {						// 10.5V
				if (f->setinvalid(val, INV_TOO_HIGH, 0))
					LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi %6.2fV TOO HIGH", ch, d->name, d->mode_name, d->avg, mvi, val/100.0);
			} else {
				if (f->change(val))
					LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi %6.2fV", ch, d->name, d->mode_name, d->avg, mvi, val/100.0);
			}
		}	break;
		case channel_mode_resistor: {
			if (mvi > 24750) {										// 2.495V
				if (f->seterror(INV_DISCONNECTED))
					LOG("ch %d disconnected %d", ch, mvi);
			} else {
				int t, r;
				int res = ntcLookup(mvi, &t, &r);					// mv*10 to degc*1000
				if (res < 0) {
					if (f->seterror(INV_OUTRANGE_LOW))
						LOG("ch %d out of range low %d", ch, mvi);
				} else if (res > 0) {
					if (f->seterror(INV_OUTRANGE_HIGH, 0))
						LOG("ch %d out of range high %d", ch, mvi);
				} else {
					int val = t/10.0;								// kR*100
					if (f->change(val))
						LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi %6.2fkR", ch, d->name, d->mode_name, d->avg, mvi, r/1000.0);
				}
			}
		}	break;
		case channel_mode_button: {
			if (mvi > 24000) {										// 2.495V open
				if (f->change(0))
					LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi open", ch, d->name, d->mode_name, d->avg, mvi);
			} else {
				if (f->change(1))
					LOG("ch %2d %-10.10s %-10.10s %7d %7dmvi closed", ch, d->name, d->mode_name, d->avg, mvi);
			}
		}	break;
		case channel_mode_ref_low:							// vref changes
		case channel_mode_ref_high: {
			calibrate(channel_data[CHANNEL_LOW0].avg, channel_data[CHANNEL_HIGH0].avg);
		}	break;
		default:
			LOG("channel %d??", ch);
			break;
		}
	}
}

void set_Led(int led, int on) {						// turn a led on/off
	pcf0_reg.led = on ? 1:0;
	write_pcf0_value();
}
void setBuzzer(int on) {						// turn buzzer on/off
	pcf0_reg.buzzer = on?1:0;
	write_pcf0_value();
}
void set4051addr(int addr) {					// set address pins of 4051 analog multiplexer
//	pcf_value &= ~(0x7 << pcf_addr_bits);
//	pcf_value |= (addr & 0x07) << pcf_addr_bits;
//	write_pcf_value();
}

/*
 * Two multiplexers: top=0, bottom=1
 * s0..2 set the top, s3..5 set the bottom
 *
 * top:			channel
 * 0	mux 1	-
 * 1	ANL4		3
 * 2	ANL5		4
 * 3	0.382v		8
 * 4	ANL6		5
 * 5	ANL8		7
 * 6	1.25v		10
 * 7	ANL7		6
 *
 * bottom:
 * 0	temp		12
 * 1	fan			13
 * 2	ANL1		0
 * 3	rh			14
 * 4	ANL2		1
 * 5	0.382v		9
 * 6	ANL3		2
 * 7	1.25v		11
 */

// channels 0..7 are real analog values
// 8,9 are 0.382v
// 10,11 are 1.25v
// 12,13,14 are temp,fan,rh. not sampled

static byte table[MAX_CHANNEL*2] = {
/* 		s0-2	s3-5 */
/* 0*/	0,		2,
/* 1*/	0,		4,
/* 2*/	0,		6,
/* 3*/	1,		0,
/* 4*/	2,		0,
/* 5*/	4,		0,
/* 6*/	7,		0,
/* 7*/	5,		0,
/* 8*/	3,		0,
/* 9*/	0,		5,
/*10*/	6,		0,
/*11*/	0,		7,
/*12*/	0,		0,
/*13*/	0,		1,
/*14*/	0,		3,
};

void setChannelMux(int ch) {
	if (ch >= 0 && ch < MAX_CHANNEL) {
		pcf0_reg.s012 = table[ch*2];
		pcf0_reg.s345 = table[ch*2+1];
	}
	write_pcf0_value();
}

void write_pcf0_value() {						// write new value to pcf
//	LOG("pcf0 write 0x%02x", pcf0_reg.reg);
	Repeat8::get().i2c->write_locked(pcf0_addr, &pcf0_reg.reg, 1);
}
void write_pcf1_value() {						// write new value to pcf
//	LOG("pcf1 write 0x%02x", pcf1_reg.reg);
	Repeat8::get().i2c->write_locked(pcf1_addr, &pcf1_reg.reg, 1);
}

//const adc_atten_t atten = ADC_ATTEN_DB_0;				// 0..800mV
const adc_atten_t atten = ADC_ATTEN_DB_11;				// 0..1100mV
static esp_adc_cal_characteristics_t adc_chars;

void init_adc() {
    // Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        LOG("eFuse Two Point: Supported");
    } else {
        LOG("eFuse Two Point: NOT supported");
    }

    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        LOG("eFuse Vref: Supported\n");
    } else {
        LOG("eFuse Vref: NOT supported");
    }

    // configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    // GPIO39 (sensor vn)
    // attenuation 0..1100mV
    adc1_config_channel_atten(ADC1_CHANNEL_3, atten);

    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_EFUSE_VREF, &adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        LOG("Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        LOG("Characterized using eFuse Vref");
    } else {
        LOG("Characterized using Default Vref");
    }
}

static int read_adc_256() {
	const int OVERSAMPLE_COUNT = 8;				// 2^n, 1..256
    static int buf[OVERSAMPLE_COUNT];

	word sum = 0;
	for (int i = 0; i < OVERSAMPLE_COUNT; i++) {
		buf[i] = adc1_get_raw(ADC1_CHANNEL_3);	// gpio39, sensor vn
		sum += buf[i];
		vTaskDelay(1);							// 1ms apart, so 8ms sample time
	}

	// return avg value * 256, for more avg resolution
	int avg = sum * (256/OVERSAMPLE_COUNT);		// just a shift

//	word v = esp_adc_cal_raw_to_voltage(avg, &adc_chars);

//	word goodsum = 0;							// get rid of too high/too low values
//	word goodcount = 0;
//	for (int i=0; i<count; i++) {
//		if (buf[i] >= avg-delta && buf[i] <= avg+delta) {
//			goodsum += buf[i];
//			goodcount++;
//		}
//	}

	// 3.06 ->  1777mV  *1,722
	// 1.69 ->  1000mV * 1,722 = 1,772

//	if (goodcount > count/2) {					// only accept if we have enough values
//		word goodavg = goodsum / goodcount;
//		word goodv = esp_adc_cal_raw_to_voltage(goodavg, &adc_chars);
////		LOG("port %d raw = %d, %dmV, good: samples=%d, avg=%d, %dmV", port, avg, v, goodcount, goodavg, goodv);
//
//		ana_mv[port] = goodv * 1.722;			// 'real' volts
//
//		if (modelinfo.mid == REPEAT8_LV_MID && port == 2) {
//			int x = ana_mv[port] / 3.0;			// 0..100,0%
//			if (x < 0) x = 0;
//			if (x > 1000) x = 1000;
//			Repeat8::get().potmeter.change(x);
//			LOG("potmeter=%d %d %d %d %d", x, buf[0], buf[1], buf[2], buf[3]);
//		}
//	} else {
//		LOG("port %d raw = %d, %dmV, invalid samples", port, avg, v);
//	}
	return avg;
}

#ifdef CONV_ADC
static int convert_adc256_to_mv(int val) {
	return esp_adc_cal_raw_to_voltage((val+128)/256, &adc_chars);
}
#endif

// input is adc counts*256
// output is mv*10
// this is the voltage on the mux inputs
// it is divided/2 before it goes to the adc, but that is all calibrated away
static float A;
static float B;

void calibrate(int countl, int counth) {
	const float refl =  3811.8; 		// 0,382
	const float refh = (24950.0)/2.0; 	// 1,25
	A = (refh-refl) / (counth-countl);
	B = refh - A*counth;
	LOG("Calibrate %d-%d A=%f, B=%f", countl, counth, A, B);
}

int scale(int avg) {
	return A*(float)avg + B;
}

// NTC lookup table, see spreadsheet
// Input is mv on the analog circuit *10
// Output is degc*1000

const int lutlen = 19;
const int lut[lutlen*3] = {
	// T, R, V (x10000)
	-20000,	87430,	22628,
	-10000,	51820,	21476,
		 0,	31770,	20131,
	  5000,	24940,	19398,
	 10000,	19680,	18662,
	 15000,	15620,	17946,
	 20000,	12470,	17266,
	 25000,	10000,	16633,
	 30000,	 8064,	16060,
	 35000,	 6538,	15548,
	 40000,	 5327,	15099,
	 50000,	 3592,	14374,
	 60000,	 2472,	13847,
	 70000,	 1735,	13471,
	 80000,	 1243,	13205,
	 90000,	  908,	13017,
	100000,	  674,	12882,
	120000,	  384,	12710,
	150000,	  177,	12584
};


int ntcLookup(int mvi, int *pt, int *pr) {				// returns temp and resistance
	// a disconnected ntc computes as -9degc, this is undetected here!
	if (mvi > lut[0+2]) {								// max mv
		*pt = lut[0];
		*pr = 0;
		return -1;										// too low
	}
	if (mvi < lut[(lutlen-1)*3+2]) {					// min mv
		*pt = lut[(lutlen-1)*3];						// 150C
		*pr = 0;
		return 1;										// too high
	}

	int n;
	for (n=0; n<lutlen*3; n+=3) {
		if (lut[n+2] <= mvi)
			break;
	}

	int t1 = lut[n-3+0], t0 = lut[n+0];			// t1 < t0
	int r1 = lut[n-3+1], r0 = lut[n+1];			// r1 > r0
	int v1 = lut[n-3+2], v0 = lut[n+2];			// v1 > v0
	*pt = t0 + (t1-t0) * (mvi-v0) / (v1-v0);
	*pr = r0 + (r1-r0) * (mvi-v0) / (v1-v0);
//	LOG("v=%d v1=%d t0=%d t1=%d t=%d r=%d", v0, v1, t0, t1, *pt, *pr);
	return 0;
}

#endif

/*
 * Analog port auto detection.
 * NTC, voltage, or open circuit have different values when
 * mosfet is switched. We can base the connected device by
 * looking at the mV value:
 *
 * MOSFET	 open	   NTC		 NTC		0V	11.5V	button
 * 					  -20C		100C	=short	clamp	  220k
 * 					   87k		674R
 * 	NTC		 2500	  2265	    1290	  1250    max     2395
 *	V 		  451	   434	     386	   380	 1900	  448(430)
 *
 * So we can determine the connection by switching the mosfet. We
 * do two measurements: NTC and V.
 *
 * An open circuit gives 250mV NTC / 451 mV
 *
 * An NTC is within (1290-2666), and V must be 386-434 mV. Voltage inputs
 * are higher then 1.2V so are easy to detect.
 *
 * If the range is 164-1250 NTC we can calculkate the expected
 * value it would have if it is a voltage: between and 2140-3840 V
 * it is a voltage.
 */

// Takes two samples of each channel, one in NTC and one in V mode
// Detects the connected device on those values

//void autodetect() {
//	Repeat8 &fx = Repeat8::get();
//
//	for (int ch=0; ch<MAX_PORT; ch++) {										// only inputs
//		auto d = &channel_data[ch];
//		if (d->mode != channel_mode_idle) {									// if not set
//			LOG("port %d is set to %s", ch, d->mode_name);
//		} else {
//			setChannelMux(ch);												// set PCF8574 pins to select the multiplexer
//
//			fx.setPortMode(ch, port_mode_degc);								// measure ntc
//			fx.setModeBits();												// turn off mosfet
//
//			vTaskDelay(500);												// let analog filter settle (100ms)
//			int val = read_adc_256();
//			int mvi_c = scale(val);											// mv*10
//
//			fx.setPortMode(ch, port_mode_volt);								// measure volts
//			fx.setModeBits();												// turn on mosfet
//			vTaskDelay(500);
//			val = read_adc_256();
//			int mvi_v = scale(val);
//
//			LOG("port %d autodetect ntc=%6d V=%6d", ch, mvi_c, mvi_v);
//
//			// An open circuit gives 21400 NTC / 4550 V
//			if (mvi_c >= 21000 && mvi_c < 22000 && mvi_v >= 4000 && mvi_v < 5000) {
//				LOG("port %d no connection", ch);
//				fx.setPortMode(ch, port_mode_none);
//			// If the NTC measurement is within (1290-2666), and V between 386 and 434 mV
//			// it is an NTC.
//			} else if (mvi_c >= 12900-40 && mvi_c < 26660+40 && mvi_v >= 3860-40 && mvi_v < 4340+40) {
//				LOG("port %d NTC connected", ch);
//				fx.setPortMode(ch, port_mode_degc);
//
//			// Otherwise, if the range is 164-1250 NTC and 2140-max V
//			// it is a voltage.
//			} else if (mvi_c >= 1640-40 && mvi_c < 1250+40 && mvi_v >= 21400-40 && mvi_v < 50000+40) {
//				LOG("port %d Voltage input", ch);
//				fx.setPortMode(ch, port_mode_volt);
//
//			} else {
//				LOG("port %d cant detect ntc=%6d V=%6d", ch, mvi_c, mvi_v);
//				fx.setPortMode(ch, port_mode_none);
//			}
//		}
//	}
//
//	fx.setModeBits();
//}
