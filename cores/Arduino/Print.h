#pragma once

#include "Arduino.h"

#define BIN	(2)
#define OCT (8)
#define DEC (10)
#define HEX (16)

class Print
{
public:
	size_t print(const char* val);
	size_t print(char val);
	size_t print(unsigned char val, int base = DEC);
	size_t print(int val, int base = DEC);
	size_t print(unsigned int val, int base = DEC);
	size_t print(long val, int base = DEC);
	size_t print(unsigned long val, int base = DEC);
	size_t print(double val, int digits = 2);
	size_t print(float val, int digits = 2);
	size_t println();
	size_t println(const char* val);
	size_t println(char val);
	size_t println(unsigned char val, int base = DEC);
	size_t println(int val, int base = DEC);
	size_t println(unsigned int val, int base = DEC);
	size_t println(long val, int base = DEC);
	size_t println(unsigned long val, int base = DEC);
	size_t println(double val, int digits = 2);
	size_t println(float val, int digits = 2);

public:
	virtual size_t write(uint8_t val) = 0;

private:
	size_t printNumber(unsigned long val, int base);
	size_t printFloat(double val, int digits);

};
