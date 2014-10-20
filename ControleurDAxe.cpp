#include "ControleurDAxe.h"
#include "Actionneur.h"
#include "Correcteur.h"
#include "BibliothequeTR\TypeProcedure.h"


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
ControleurDAxe::ControleurDAxe(Actionneur* pActionneur, Correcteur* pCorrecteur, float commandeAuRepos, float gain)

{
	_pActionneur = pActionneur;
	_pCorrecteur = pCorrecteur;
	_deltaCommande = 0.0;
	_positionMin =	0.0;	// -90.0;
	_positionMax = 	110.0;	//+150.0;
	_asservissementActif=true;
	_boucleOuverte=false;
	_commandeRepos = commandeAuRepos;
	_gainBoucleOuverte = gain;
	//_tacheDAsservissement.creer("CtrlAxe", PRIORITE_CTRL_AXE, 0, 2000, (TypeProcedure) ControleurDAxe::procedureDAsservissement, 0,0,0,0,0,0,0,0,0,0);

	//on initialise juste au cas ou l'on oublie ds le prog.
	_commande	=	0.0f;
	_consigneDePosition	=	0.0f;
	_consigneDeVitesse	=0.0f;
	_consigneDeAcceleration	=	0.0f;

}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
ControleurDAxe::~ControleurDAxe()
{
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::envoyerConsigneDePosition(float position)
{
	if ((position > _positionMax) || (position < _positionMin))
	{
		return false;
 	}
	
  _consigneDePosition = position;
	
	return true;
}

/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool	ControleurDAxe::envoyerConsigneDePosition(float position, float vitesse, float acceleration)
{
	if ((position > _positionMax) || (position < _positionMin))	return	false;

	else {
	_consigneDePosition	=	position;
	_consigneDeVitesse	=	vitesse;
	_consigneDeAcceleration	=	acceleration;

		return	true;}

}

/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::mettreAJourCommande()
{

	_commande	=	_pCorrecteur->calculerCommande(_consigneDePosition, _consigneDeVitesse, _consigneDeAcceleration);

	if(_commande > _saturationMax)	_commande	=	_saturationMax;
		else if(_commande < _saturationMin)	_commande	=	_saturationMin;

	if(!_pActionneur->envoyerCommande(_commande)) {	return	false;	}

	_commandePrecedente	=	_commande;
	return	true;

}


/*
    float deltaP = _pCorrecteur->calculerCommande();
    
    //saturation
	  if (deltaP > 4.5f) deltaP = 4.5f;
    if (deltaP < 0.5f) deltaP = 0.5f;
	  
    printf("deltaP : %2.4f\n", deltaP);
    
    //envoyer la DELTA_PRESSION trouvee
    _pActionneur->envoyerCommande(deltaP -2.5f); //+ _commandeRepos);
  */  
    
    
  /*
	double commande = _pCorrecteur->calculerCommande(_consigneDePosition,
                                                   _deltaCommande,
																						       //_pActionneur->deltaPression(),
																						       _periodeDEchantillonnage);
  
  //envoyer la commande en forme d'une DELTA_PRESSION (conversion degres)
  double delta_P = _commandeRepos + (0.75* _gainBoucleOuverte*_consigneDePosition) + commande;
  
  //correction
	if (delta_P >  2.5) delta_P =  2.5;
	if (delta_P < -2.5) delta_P = -2.5;
	
  //envoyer la DELTA_PRESSION trouvee
	if(!(_pActionneur->envoyerCommande(delta_P)))
    return false;
  
  //mise en jour
	_deltaCommande = _deltaCommande - delta_P;
	
	return commande;
*/



/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::asservirAxe(const Duree& periodeDEchantillonnage)
{
	_tacheDAsservissement.creer("CtrlAxe", PRIORITE_CTRL_AXE, 0, 2000, (TypeProcedure) ControleurDAxe::procedureDAsservissement, (int)this);


	//	printf("Depart de l'Asservissement..........................\n");
	_periodeDEchantillonnage = periodeDEchantillonnage;
  
	_asservissementActif = true; 
	return(_horlogeDAsservissement.demarrer(_periodeDEchantillonnage));
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::arreterAsservissement()
{
	printf("Arreter l'Asservissement..........................\n");
	
	_tacheDAsservissement.tuer();
	_asservissementActif=false;
	return(_horlogeDAsservissement.arreter());
}

/*************************************************************************/
/*************************************************************************/
/*************************************************************************/
bool	ControleurDAxe::tuerAsservissement()
{
	return(_tacheDAsservissement.tuer());
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
//double
float ControleurDAxe::consigne() const
{
	return _consigneDePosition;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
//double
float ControleurDAxe::commande() const
{
	return _deltaCommande;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::procedureDAsservissement()
{

	while(_asservissementActif)
	{
		_horlogeDAsservissement.attendre();
		if(_horlogeDAsservissement.attendre());

		mettreAJourCommande();

	}

	
	return false;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::procedureDInitialisation()
{
	float pressionMuscle1 = _pActionneur->pressionMuscle1();
	float pressionMuscle2 = _pActionneur->pressionMuscle2();
	float pressionDeBase  = _pActionneur->pressionDeBase();
	
	//printf("Pression axe : %f %f \n", pressionMuscle1, pressionMuscle2);
	while (pressionMuscle1 < (pressionDeBase + _commandeRepos)|| pressionMuscle2 < (pressionDeBase - _commandeRepos))
  {
    if (pressionMuscle1<(pressionDeBase + _commandeRepos))
			pressionMuscle1 += 0.05;
    else
    	pressionMuscle1 = pressionDeBase + _commandeRepos;

		if (pressionMuscle2<(pressionDeBase - _commandeRepos))
			pressionMuscle2 += 0.05;
    else pressionMuscle2 = pressionDeBase - _commandeRepos;

    _pActionneur->envoyerCommandeDecouplee(pressionMuscle1,pressionMuscle2);
			
    taskDelay(sysClkRateGet()/10);
  }
  _deltaCommande = _commandeRepos;
  //printf("Commande precedente : %f\n", _deltaCommande);
  
  return true;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::initialiser(float commandeRepos)
{
	_commandeRepos = commandeRepos;

	return (Tache::creerTache("initMuscle", 100, 0, 20000, (TypeProcedure)ControleurDAxe::procedureDInitialisation, (int) this));
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::setPositionMax(float left)
{
    _positionMax = left;
    return true;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::setPositionMin(float left)
{
    _positionMin = left;
    return true;
}
/***************************************************************/
bool	ControleurDAxe::setSaturationMax(float satMax)
{
	_saturationMax	=	satMax;
	return	true;
}
/***************************************************************/
bool	ControleurDAxe::setSaturationMin(float satMin)
{
	_saturationMax	=	satMin;
	return	true;
}

/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
//double
float ControleurDAxe::getPositionMax() const
{
    return _positionMax;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
//double
float ControleurDAxe::getPositionMin() const
{
    return _positionMin;
}

/***********************************************************************/
float	ControleurDAxe::getSaturationMax()
{
	return	_saturationMax;
}
/*********************************************************************/
float	ControleurDAxe::getSaturationMin()
{
	return	_saturationMin;
}
/*********************************************************************/
float	ControleurDAxe::getCommande()
{
	return	_commande;
}
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::setBoucleOuverte(bool changer)
{
	_boucleOuverte=changer;
    return _boucleOuverte;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
bool ControleurDAxe::getBoucleOuverte()
{
    return _boucleOuverte;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/

void ControleurDAxe::setCorrecteur(Correcteur* correcteur)
{
  _pCorrecteur = correcteur;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/

Correcteur* ControleurDAxe::getCorrecteur() {
  return _pCorrecteur;
}


/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
//double
float ControleurDAxe::getPressionRepos() {
  return _commandeRepos;
}
