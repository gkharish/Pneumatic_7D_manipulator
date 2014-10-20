#include <stdio.h>

#include "Actionneur.h"
#include "InterfaceES/InterfaceCarteDeSortieAnalogique.h"

//##ModelId=4087CB3A0070
Actionneur::Actionneur(InterfaceCarteDeSortieAnalogique* pCarteDeSortieAnalogique, char voieMuscle1, char voieMuscle2, float pente, float offset)
{
	_pCarteDeSortieAnalogique = pCarteDeSortieAnalogique;
	_pressionDeBase = 2.5;
	_pressionMuscle1 = 0.0;
	_pressionMuscle2 = 0.0;
  _pression_repos  = 0.0f;

	_voieMuscle1 = voieMuscle1;
	_voieMuscle2 = voieMuscle2;
	_pente = pente;
	_offset = offset;
}

//##ModelId=4087CA4501B9
Actionneur::~Actionneur()
{
}

//##ModelId=407CF4280351
bool Actionneur::envoyerCommande(float commande, float pressionDeBase)
{
  float deltaPp = _pression_repos + commande;

	float m1 = _pressionDeBase + deltaPp;
	float m2 = _pressionDeBase - deltaPp;
	return (envoyerCommandeDecouplee(m1,m2));
}

//##ModelId=407CF4280351
/*
bool Actionneur::envoyerCommande(float deltaP, float pressionActuelle1, float pressionActuelle2)
{
	float m1 = pressionActuelle1 + deltaP; //_pressionDeBase + deltaP;
	float m2 = pressionActuelle2 - deltaP; //_pressionDeBase - deltaP;
	return (envoyerCommandeDecouplee(m1,m2));
}
*/

//##ModelId=407CF85C0204
bool Actionneur::envoyerCommandeDecouplee(float pressionMuscle1, float pressionMuscle2)
{
	float buffer;

	//Vérification de la validité des pressions
	if (!((pressionMuscle1 >= PRESSION_MIN && pressionMuscle1 <= PRESSION_MAX)
		&& (pressionMuscle2 >= PRESSION_MIN && pressionMuscle2 <= PRESSION_MAX)))
	{
 	//	printf("Pression Invalide\n");
   		return false;
//////////////////////////////////////////////////////////////
	//	limitation du taux de gonflement
	if((_pressionMuscle1 - pressionMuscle1)>ECHELON_MAX)
					pressionMuscle1 = _pressionMuscle1 - ECHELON_MAX;
	if((_pressionMuscle1 - pressionMuscle1)<-ECHELON_MAX)
					pressionMuscle1 = _pressionMuscle1 + ECHELON_MAX;
//////////////////////////////////////////////////////////////

	}

	buffer = (pressionMuscle1) * _pente + _offset;
	//printf("Pression Mucle1 : %f Intensite : %f ", pressionMuscle1, buffer);
	if(_pCarteDeSortieAnalogique->daConv(_voieMuscle1, buffer))
		_pressionMuscle1 = pressionMuscle1; //mise à jour pression courante

	buffer = (pressionMuscle2) * _pente + _offset;
	if(_pCarteDeSortieAnalogique->daConv(_voieMuscle2, buffer))
  		_pressionMuscle2 = pressionMuscle2; //mise à jour pression courante

	return true;
}

//##ModelId=407D1E3F0164
float Actionneur::pressionMuscle1() const
{
	return _pressionMuscle1;
}

//##ModelId=408E79A101EA
float Actionneur::pressionMuscle2() const
{
	return _pressionMuscle2;
}

//##ModelId=40A4B194014A
float Actionneur::pressionDeBase() const
{
    return _pressionDeBase;
}

float Actionneur::deltaPression() const
{
	return (_pressionDeBase - _pressionMuscle2);
}

//##ModelId=4087659E008A
bool Actionneur::setPressionDeBase(float pression)
{
	if (pression >= PRESSION_DE_BASE_MIN && pression <= PRESSION_DE_BASE_MAX)
	{
 		_pressionDeBase = pression;
		return true;
	}
	else
 	{
  		printf("Pression de Base invalide\n");
    	return false;
	}
}


void Actionneur::setPressionRepos(float pression_repos) {
  _pression_repos = pression_repos;
}

float Actionneur::getPressionRepos(void) {
  return _pression_repos;
}
