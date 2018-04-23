#include "Arduino.h"
#include <string.h>

#define DELIM	"\r\n"

size_t Print::print(const char* val)
{
	auto len = (int)strlen(val);
	for (int i = 0; i < len; i++) write(val[i]);
	return len;
}

size_t Print::print(char val)
{
	write(val);
	return 1;
}

size_t Print::print(unsigned char val, int base)
{
	return print((unsigned long)val, base);
}

size_t Print::print(int val, int base)
{
	return print((long)val, base);
}

size_t Print::print(unsigned int val, int base)
{
	return print((unsigned long)val, base);
}

size_t Print::print(long val, int base)
{
	switch (base) {
	case DEC:
		if (val < 0) {
			int t = print('-');
			val = -val;
			return printNumber(val, base) + t;
		}
		return printNumber(val, base);
	default:
		return printNumber(val, base);
	}
}

size_t Print::print(unsigned long val, int base)
{
	return printNumber(val, base);
}

size_t Print::print(double val, int digits)
{
	return printFloat(val, digits);
}

size_t Print::print(float val, int digits)
{
	return printFloat(val, digits);
}

size_t Print::println()
{
	return print(DELIM);
}

size_t Print::println(const char* val)
{
	auto n = print(val);
	n += println();
	return n;
}

size_t Print::println(char val)
{
	auto n = print(val);
	n += println();
	return n;
}

size_t Print::println(unsigned char val, int base)
{
	auto n = print(val, base);
	n += println();
	return n;
}

size_t Print::println(int val, int base)
{
	auto n = print(val, base);
	n += println();
	return n;
}

size_t Print::println(unsigned int val, int base)
{
	auto n = print(val, base);
	n += println();
	return n;
}

size_t Print::println(long val, int base)
{
	auto n = print(val, base);
	n += println();
	return n;
}

size_t Print::println(unsigned long val, int base)
{
	auto n = print(val, base);
	n += println();
	return n;
}

size_t Print::println(double val, int digits)
{
	auto n = print(val, digits);
	n += println();
	return n;
}

size_t Print::println(float val, int digits)
{
	auto n = print(val, digits);
	n += println();
	return n;
}

size_t Print::printNumber(unsigned long val, int base)
{
	char buf[8 * sizeof(val) + 1];
	char* str = &buf[sizeof(buf) - 1];
	*str = '\0';

	do {
		char c = val % base;
		val /= base;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (val);

	return print(str);
}

size_t Print::printFloat(double val, int digits)
{
	size_t n = 0;

	if (val < 0) {
		n += print('-');
		val = -val;
	}

	double rounding = 0.5;
	for (int i = 0; i < digits; i++) {
		rounding /= 10.0;
	}
	val += rounding;

	unsigned long int_part = (unsigned long)val;
	double remainder = val - (double)int_part;
	n += print(int_part);

	if (digits > 0) {
		n += print('.');
	}

	while (digits-- > 0) {
		remainder *= 10.0;
		unsigned int toPrint = (unsigned int)remainder;
		n += print(toPrint);
		remainder -= toPrint;
	}

	return n;
}
