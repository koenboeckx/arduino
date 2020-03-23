/*
  LL.h - Library for encapsulating LEAD-LAG logic
*/
#ifndef LL_h
#define LL_h

class LL
{
	public:
		LL(float Te, float K, float Tp, float Tz);
		float getNextCommand(float u, float y);
	private:
		float I;
		float _Te;
		float _K;
		float _Tp;
		float _Tz;
};

#endif