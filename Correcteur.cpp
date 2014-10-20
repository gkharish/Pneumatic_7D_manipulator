#include "Correcteur.h"

bool Correcteur::isPID()
{
	if(_type == _PID)
		return true;
	else
		return false;
}



bool Correcteur::isTW()
{
	if(_type == _TW)
		return true;
	else
		return false;
}


bool Correcteur::isSTW()
{
	if(_type == _STW)
		return true;
	else
		return false;
}

/*
// amar le 10-04-2012******debut

bool Correcteur::isTSMC()
{
	if(_type == _TSMC)
		return true;
	else
		return false;
}
// amar le 10-04-2012******** fin 

 */