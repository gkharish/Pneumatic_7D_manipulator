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
