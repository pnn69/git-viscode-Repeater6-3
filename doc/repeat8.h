/*
 * Repeat8 component
 */

#ifndef _Repeat8_H_
#define _Repeat8_H_

#include "global.h"

#ifdef ENABLE_REPEAT8
#include "fields.h"

#define MAX_PORT 8

#define port_mode_none 	0
#define port_mode_volt 	1
#define port_mode_degc 	2
#define port_mode_degf 	3
#define port_mode_button 	4
#define port_mode_resistor 	5

class Port : public Values {
public:
	Port() : Values(){}
	virtual ~Port() {}
	IntValue	val;			// temperature or voltage + mode
};

class Repeat8 : public Component {

public:
	IntValue 	enabled;
	Port		ports[MAX_PORT];

	static Repeat8& get() {			// singleton
		static Repeat8 instance{};
		return instance;
	}

private:
	Repeat8();
	virtual ~Repeat8();
	Repeat8(Repeat8 const&) = delete;
	void operator=(Repeat8 const&) = delete;
public:
	virtual int init();
	virtual int start();
	virtual void poll();
	virtual void valuechanged(Value *val);							// called on parent when changed
	void setPortMode(int port, int mode);
	void setModeBits();
	int i2cbus{0};
	I2cMaster *i2c{nullptr};

public:
};

#endif
#endif

