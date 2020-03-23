/*
  LL.cpp - Implementation of LEAD-LAG logic
*/

#include "LL.h"

LL::LL(float Te, float K, float Tp, float Tz)
{
	_Te = Te;
	_K = K;
	_Tp = Tp;
	_Tz = Tz;
	I=0.0;
}
float LL::getNextCommand(float u, float y) {
	I=I+_Te*(_K*u-y)/_Tp;
	y=I+_K*u*_Tz/_Tp;
	return y;
}