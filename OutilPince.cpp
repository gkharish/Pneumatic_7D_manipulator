#include "OutilPince.h"

//##ModelId=4090B6C800AC
OutilPince::OutilPince(InterfaceCarteDeSortieAnalogique* pCarteDeSortieAnalogique, char voiePince1, char voiePince2)
{
	_pCarteDeSortieAnalogique = pCarteDeSortieAnalogique;
	_voiePince1 = voiePince1;
	_voiePince2 = voiePince2;
 	this->fermer();
}

//##ModelId=4088DC0400D5
OutilPince::~OutilPince()
{
}

//##ModelId=4088DF4E0252
bool OutilPince::estOuverte() const
{
	return _pinceOuverte;
}

//##ModelId=4088DF4C0105
bool OutilPince::estFermee() const
{
	return (_pinceOuverte == false);
}

//##ModelId=407D4BFC0369
bool OutilPince::fermer()
{
	_pCarteDeSortieAnalogique->daConv(_voiePince1, NIVEAU_HAUT);
 	taskDelay(sysClkRateGet()/4); // duree de l'impulsion
 	_pCarteDeSortieAnalogique->daConv(_voiePince1, NIVEAU_BAS);
 	_pinceOuverte = false;
 	
 	return true;
}

//##ModelId=407D4C030138
bool OutilPince::ouvrir()
{
	_pCarteDeSortieAnalogique->daConv(_voiePince2, NIVEAU_HAUT);
 	taskDelay(sysClkRateGet()/4); // duree de l'impulsion
 	_pCarteDeSortieAnalogique->daConv(_voiePince2, NIVEAU_BAS);
 	_pinceOuverte = true;
 	
 	return true;
}

