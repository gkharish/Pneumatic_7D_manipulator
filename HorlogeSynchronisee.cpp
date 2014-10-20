// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/


#include "HorlogeSynchronisee.h"

//#include "Temps.cpp"

//##ModelId=40AB5E890254
Horloge::ParametresDHorloge::ParametresDHorloge(int id, int periode, Evenement* pEvenement)
{ 
_id = id;
_periode = periode;
_pEvenement = pEvenement;
}


//##ModelId=40AB02040335
Horloge::Horloge()
{
	static int nouvelId = 1;
 	_id = nouvelId++;
}


//##ModelId=40AB02040336
Horloge::~Horloge()
{
	this->arreter();
}

//##ModelId=40AB02040339
bool Horloge::demarrer(const Duree& periode)
{
	int periodeEnMs = (int) ((periode.toSecondes()*1000.0)+0.5);

	_listeDesHorloges.push_front(ParametresDHorloge(_id, periodeEnMs, &_finDePeriode));

	//On initialise l'horloge auxiliaire VxWorks si c'est la premiere horloge crée
	if (_listeDesHorloges.size() == 1)
	{
		if (sysAuxClkRateSet(1024)==ERROR) // Reglage de l'horloge auxiliaire à 1ms
			return false; 
		sysAuxClkConnect((FUNCPTR) Horloge::procedureDHorloge, 0);
		sysAuxClkEnable();
	}
	
	return true;
}

//##ModelId=40AB0204033B
bool Horloge::arreter()
{
	ListeDHorloges::iterator i;

	for(i = _listeDesHorloges.begin(); i != _listeDesHorloges.end(); ++i)
	{
		if (i->_id == _id)
			{
				_listeDesHorloges.erase(i);
				break;
			}
	}

	// S'il n'y a plus d'horloge démarée on arrete les interruptions
	if (_listeDesHorloges.empty())
	{
		sysAuxClkDisable(); //Arret de l'horloge auxiliaire
	}

	return true;
}

//##ModelId=40AB0204033C
bool Horloge::attendre() const
{
  return (_finDePeriode.attendre());
}

//##ModelId=40AB0204033F
void Horloge::procedureDHorloge()
{
	static unsigned long int clock = 0;
	ListeDHorloges::iterator i;
	//bool reajustementPossible = true;
	//static ULONG dateReajustement;

	clock++;

  for(i = _listeDesHorloges.begin(); i != _listeDesHorloges.end(); ++i)
    {
      if(clock%(i->_periode) == 0) i->_pEvenement->signalerATous();
				//reajustementPossible = false;
    }

	/*if (reajustementPossible && clock > dateReajustement == 0)
 	{
 		dateReajustement = clock + 42;
  	clock--;
	}*/

}

//##ModelId=40AB0E360077
Horloge::ListeDHorloges Horloge::_listeDesHorloges;

