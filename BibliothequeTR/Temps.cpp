// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#include "Temps.h"
#include "tickLib.h"


//##ModelId=40A0A2BF03B2
Duree::Duree()
{
	_secondes = 0.0;
}

//##ModelId=40A07118030E
Duree::Duree(float valeur, UniteDeTemps unite)
{
	switch (unite)
	{
		case US : _secondes = valeur/1000000.0; break;
		case MS : _secondes = valeur/1000.0; break;
		case S  : _secondes = valeur; break;
		case MN : _secondes = valeur*60.0; break;
		case H  : _secondes = valeur*3600.0;
	}
}

//##ModelId=40A071180336
Duree::~Duree()
{
}

//##ModelId=40A089F10011
double Duree::toSecondes() const
{
	return _secondes;
}

//##ModelId=40A071180338
const Duree& Duree::operator=(const Duree& obj)
{
	_secondes = obj._secondes;
	return *this;
}

//##ModelId=40A073DF0049
Duree Duree::operator+(const Duree& obj)
{
	return Duree(_secondes + obj._secondes, S);
}

//##ModelId=40A073D702FA
Duree Duree::operator-(const Duree& obj)
{
	return Duree(_secondes - obj._secondes, S);
}


//##ModelId=40A0798003E4
Date::Date()
{
	_date = tickGet();
}

//##ModelId=40A0798003E5
Date::~Date()
{
}

//##ModelId=409F950E00D0
void Date::setNow()
{
	_date = tickGet();
}

//##ModelId=40A0981F03BB
Date Date::getDateActuelle()
{
	Date dateActuelle;
	return dateActuelle;
}

//##ModelId=40A073F30278
Date Date::operator=(const Date& obj)
{
	_date = obj._date;
	return *this;
}

//##ModelId=409F971B017F
Duree Date::operator-(const Date& obj)
{
	ULONG diffDates;
	float secondes;

	diffDates = _date - obj._date;
	secondes = (float) diffDates / (float) sysClkRateGet();

	return Duree(secondes, S);
}

//##ModelId=40A096CF0372
Date Date::operator+(const Duree& duree)
{
	Date newDate;
	ULONG ticksDuree;
	ticksDuree = (ULONG) (duree.toSecondes() * (float) sysClkRateGet());
	newDate._date = _date + ticksDuree;
	
	return newDate;
 }

//##ModelId=40A09B9303D0
bool Date::operator<(const Date& right)
{
	return (_date < right._date);
}

//##ModelId=40A09B9303DA
bool Date::operator>(const Date& right)
{
	return (_date > right._date);
}

