	/*	CORRECTEUR TWI.cpp */

#include	"CorrecteurTWI.h"

CorrecteurTWI :: CorrecteurTWI(CapteurDePosition* pCapteurDePosition,
																float	b,
																float	a2,
																float	a1,
																float	dt)
{
	_pCapteur	=	pCapteurDePosition;

/* parametres du modele */
//-------------------------
_B	=	b;
_A2	=	a2;
_A1	=	a1;


/* parametres de commutation */
//------------------------------
_C	=	6.50f;
_ULIM	=	1.0f;
_AMIN	=	0.2f;
_AMAX	=	0.35f;
_p = 0.0573f;
_i = 2.865f;


_dt			=	dt;	//periode d'echantill.
_suivi	=	false; //pas de suivi de trajectoire

/* type de correcteur */
//------------------------
_type	=	_TWI;

}

CorrecteurTWI :: ~CorrecteurTWI()
{
}

float	CorrecteurTWI :: calculerCommande(float theta_desiree, float vitesse_desiree, float acceleration_desiree)
{
	float	u	=	0.0;
	u	=	commandeTWI(theta_desiree, vitesse_desiree, acceleration_desiree);
	return u;
}

float	CorrecteurTWI :: commandeTWI(float theta_desiree, float vitesse_desiree, float acceleration_desiree)
{
	float	theta_mesuree	=	(float)(_pCapteur->lirePositionRAD());
	_erreur	=	theta_mesuree	-	theta_desiree;
	_erreur_derivee	=	(_erreur	-	_erreur_precedente)/_dt;
	_erreur_precedente	=	_erreur;
	_erreur_integree = _erreur_integree_precedente + (_erreur*_dt);
	_erreur_integree_precedente = _erreur_integree;


/* commande equivalente */
//-------------------------
if(_suivi)	{
		_uEQ = (1.0f/_B)*(((_A2-_C)*_erreur_derivee)
															+(_A1*_erreur)+(_A1*theta_desiree)
															+(_A2*vitesse_desiree)	+(acceleration_desiree));

}	else {
							_uEQ = (1.0f/_B)*(((_A2-_C)*_erreur_derivee)
															+(_A1*_erreur)+(_A1*theta_desiree));
											}//fin Ueq

/* surface et spoint */
//----------------------
_surface	=	(_C*_erreur) + _erreur_derivee;
_surface_derivee	=	(_surface - _surface_precedente)/_dt;
_surface_precedente	=	_surface;



/* terme discontinu */
//-----------------------


if(absOf(_uDELTA) <= _ULIM) {	// absOf(_uEQ)){
		if((_surface * _surface_derivee) > 0.0f) { _uDELTA_derivee	=	-(_AMAX * signOf(_surface));
}	else	{ _uDELTA_derivee	=	-(_AMIN * signOf(_surface));}
}	else	{ _uDELTA_derivee	=	-(_uDELTA);}

_uDELTA	=	( _uDELTA_derivee * _dt) + _uDELTA_precedente;
_uDELTA_precedente	=	_uDELTA;

/* commande= ueq + u1 */
//-----------------------
float commande;

if (absOf(_surface) > 0.1) {
commande	=	_uEQ + _uDELTA;
//acceleration de convergence
//if(absOf(_surface) > 1) commande = commande*2.0;
}	else	{
				commande = (_p * _erreur) + (_i * _erreur_integree);}

return	commande;

}//fin TW

/* fonction signOf */
//--------------------
float	CorrecteurTWI :: signOf(const	float	value)
{
	if(value < 0.0f)
					return	-1.0f;
	else	if(value > 0.0f)
					return	1.0f;
	else
					return	0.0f;
}

/* fonction absOf */
//-------------------
float CorrecteurTWI :: absOf(const	float	value)
{
	if(value < 0.0f)
				return (value * -1.0f);
	else
				return	value;
}

/* INITialisation des variables */
//---------------------------------
void	CorrecteurTWI :: initialise(float consigne)
{
	_erreur_precedente	=	-consigne;
	_surface_precedente	=	-(_C * consigne);
	_uDELTA_precedente	=	0.0f;
	_uDELTA	=	0.0f;
	_erreur_integree_precedente = 0.0f;

}


/* lecture GET des variables du twist */
//------------------------------------
float	CorrecteurTWI :: getC(void) {return _C;}
float	CorrecteurTWI :: getUlim(void) {return _ULIM;}
float	CorrecteurTWI :: getAmin(void) {return _AMIN;}
float	CorrecteurTWI :: getAmax(void) {return _AMAX;}
float CorrecteurTWI :: getP(void) {return _p;}
float CorrecteurTWI :: getI(void) {return _i;}

float	CorrecteurTWI :: getUeq(void) {return _uEQ;}
float	CorrecteurTWI :: getUdelta(void) {return _uDELTA;}
float	CorrecteurTWI :: getSurface(void) {return _surface;}
float	CorrecteurTWI :: getSurfacePoint(void) {return _surface_derivee;}
float	CorrecteurTWI :: getErreur(void) {return _erreur;}


/* reglage des variables du twist */
//----------------------------------
void	CorrecteurTWI	::	setC(float c){_C = c;}
void	CorrecteurTWI	::	setUlim(float ulim){_ULIM = ulim;}
void	CorrecteurTWI	::	setAmin(float amin){_AMIN = amin;}
void	CorrecteurTWI	::	setAmax(float amax){_AMAX = amax;}
void	CorrecteurTWI ::	setP(float p){_p = p;}
void	CorrecteurTWI ::	setI(float i){_i = i;}

void	CorrecteurTWI	::	setSuivi(bool s){_suivi = s;}

