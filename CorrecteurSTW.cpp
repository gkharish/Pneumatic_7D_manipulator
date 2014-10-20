		/*	CORRECTEURSTW.cpp		*/

#include	"CorrecteurSTW.h"
#include	"math.h"

////////////////////////////////////////////////////////////////////////////
	CorrecteurSTW::CorrecteurSTW(CapteurDePosition* pCapteurDePosition,
																	float	b,
																	float a2,
																	float a1,
																	float dt)
{
	_pCapteur	=	pCapteurDePosition;

/* parametres du modele */
//------------------------
_B	=	b;
_A2	=	a2;
_A1	=	a1;

/* parametres de commutation */
//------------------------------
_C	=	6.50f;
_ULIM	=	4.0f;
_W	=	0.3f;
_LAMBDA	=	0.15f;
_S0	=	0.2f;
_ALPHA = 0.0f; //accelerateur de convergence karim 08/12/2008
_sauv_alpha=_ALPHA; // karim ajout 27/11/2008
_ATTENUATOR = 0.1f; //attenuateur de uequ 25 mai 09
_k=0.0f;//karim 25 mai 2009

_dt	=	dt;
_suivi	=	false; //no tracking

/* type de correcteur */
//-----------------------
_type	=	_STW;

}

/////////////////////////////////////
CorrecteurSTW::~CorrecteurSTW()
{
}


///////////////////////////////////////////////////////////////////////////////////////
float	CorrecteurSTW::calculerCommande(float theta_desiree, float vitesse_desiree, float acceleration_desiree)
{
	float	u	=	0.0f;
	u	=	commandeSTW(theta_desiree, vitesse_desiree, acceleration_desiree);
	return	u;
}// fin calculerCommande

///////////////////////////////////////////////////////////////////////////////////////
float CorrecteurSTW::commandeSTW(float theta_desiree, float vitesse_desiree, float acceleration_desiree)
{
	float	commande = 0.0;
	float	theta_mesuree	= (float)(_pCapteur->lirePositionRAD());
  _erreur	=	theta_mesuree	-	theta_desiree;
	_erreur_derivee	=	(_erreur	-	_erreur_precedente)/_dt;
	_erreur_precedente	=	_erreur;

/* commande equivalente */
if(_suivi) {
							_uEQ	=	(((_A2 * _C) *_erreur_derivee)
																+ (_A1 * _erreur) + (_A1 * theta_desiree)
																+ (_A2 * vitesse_desiree)	+ (acceleration_desiree)) / _B;
}	else {
							_uEQ	=	(((_A2 * _C) *_erreur_derivee)
																+ (_A1 * _erreur)	+ (_A1 * theta_desiree)) / _B;
 							} //fin Ueq

_uEQ = _uEQ * (_ATTENUATOR); //a rajouter ensuite

/* surface */
//-----------
_surface	=	(_C * _erreur) + _erreur_derivee;
_surface_derivee	=	(_surface - _surface_precedente)/_dt;
_surface_precedente	=	_surface;


/* terme issu du terme discontinu U3=integrale de U1point+U2*/
//------------------------------------------------------------
if(absOf(_uDELTA) <= _ULIM) {
													_uUN_derivee = -( _W * signOf(_surface));

}	else	{ _uUN_derivee = -( _uDELTA);}


// karim ajout condition d'acceleration 

/*if (fabs(_erreur)<=0.15)	
					{									
						_ALPHA=0.0f;

 					}

if (fabs(_erreur)>0.15)	
					{									
						_ALPHA=_sauv_alpha;

 					}*/

// fin ajout de karim condition d'acceleration 



_uUN	=	_uUN	- _ALPHA*(_surface); // ajout karim acclerateur de convergence 12/12/2008

_uUN	=	(_uUN_derivee * _dt) + _uUN_precedente;
_uUN_precedente	=	_uUN;

if(absOf(_surface) <= _S0) {
									_uDEUX	=	-(_LAMBDA * sqrt(absOf(_surface)) * signOf(_surface));
}	else	{
					_uDEUX	=	-(_LAMBDA * sqrt(absOf(_S0)) * signOf(_surface));}


_uDELTA	=	_uUN + _uDEUX+_uEQ; // ajout karim du terme ueq le 18 mai 2009

/* commande */
//-------------

_uDELTA=_uDELTA-_k*absOf(_erreur)*signOf(_surface); // karim ajout acclerateur 25/05/09


/**************************ajout karim 4/06/2009*************************/
/*if (fabs(_erreur)<=0.1 )	// ancienne valeur 0.155
					{									
					_uDELTA=_sauv_pression;
          _k=0.0f;//_k/1000;
          _ALPHA=0.0f;//_ALPHA/1000;

				}
else 
{_sauv_pression=_uDELTA; }*/

/************************************************************************/

commande = _uDELTA;

	return commande;
}// fin commandeSTW
////////////////////////////////////////////////////
/* fonction signOf */
//--------------------
float	CorrecteurSTW::signOf(const	float	value)
{
	if(value < 0.0f)
				return	-1.0f;
	else if(value > 0.0f)
				return	1.0f;
	else
			return	0.0f;
}
////////////////////////////////////////////////////
/* fonction absOf */
//--------------------
//double
float CorrecteurSTW::absOf(const	float	value)
{
	if(value < 0.0f)
					return	(value * -1.0f);
	else
				return	value;
}

/* INITialisation des variables */
//----------------------------------
void CorrecteurSTW::initialise(float consigne)
{
	float	theta_mesuree	=	(float)(_pCapteur->lirePositionRAD());
	_erreur_precedente	=	theta_mesuree-consigne;
	_surface_precedente	=	_C *	_erreur_precedente;

	_uEQ = 0.0f;
	_uDELTA	=	0.0f;
	_uUN_precedente	=	0.0f;
//	_uDELTA_precedente		=	0.0f;
}



////////////////////////////////////////////////////
/* lecture des variables via GET */
//----------------------------------
float	CorrecteurSTW::getC(void)	{return _C;}
float	CorrecteurSTW::getUlim(void)	{return _ULIM;}
float	CorrecteurSTW::getW(void)	{return _W;}
float	CorrecteurSTW::getLambda(void)	{return _LAMBDA;}
float	CorrecteurSTW::getSo(void)	{return _S0;}

float	CorrecteurSTW::getUeq(void)	{return _uEQ;} 
float	CorrecteurSTW::getUdelta(void)	{return _uDELTA;}
float	CorrecteurSTW::getSurface(void)	{return _surface;}
float	CorrecteurSTW::getSurfacePoint(void)	{return _surface_derivee;}
float	CorrecteurSTW::getErreur(void)	{return _erreur;}
float CorrecteurSTW::getAlpha(void) {_ALPHA=_sauv_alpha; return _ALPHA;} // ajout karim 12/12/08

float	CorrecteurSTW::getuUN(void)	{return _uUN;}  // karim ajout 
float	CorrecteurSTW::getuDEUX(void)	{return _uDEUX;}  //  karim ajout 
//attenuateur de ueq
float CorrecteurSTW::getAttenuator(void) {return _ATTENUATOR;} // karim ajout 26/05/09

/* reglage des variables via SET */
//-----------------------------------
void	CorrecteurSTW::setC(float c)	{	_C	=	c;}
void	CorrecteurSTW::setUlim(float ulim)	{	_ULIM	=	ulim;}
void	CorrecteurSTW::setW(float w)	{	_W	=	w;}
void	CorrecteurSTW::setLambda(float lambda)	{	_LAMBDA	=	lambda;}
void	CorrecteurSTW::setSo(float so)	{	_S0	=	so;}
void	CorrecteurSTW::setAttenuator(float attenuator){_ATTENUATOR = attenuator;} // ajout karim 26 mai09

void	CorrecteurSTW::setSuivi(bool s)	{_suivi	=	s;}

void	CorrecteurSTW::setAlpha(float alpha){_ALPHA = alpha; _sauv_alpha=alpha;} // karim ajout 12/12/08


float	CorrecteurSTW :: get_k(void) {return _k;} // karim 25/05/09
void	CorrecteurSTW :: set_k(float k){_k = k;}  // karim 25/05/09