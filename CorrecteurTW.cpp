	/*	CORRECTEUR TW.cpp */

#include	"CorrecteurTW.h"
#include	"math.h"

CorrecteurTW :: CorrecteurTW(CapteurDePosition* pCapteurDePosition,
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
_ULIM	=	4.0f;
_AMIN	=	0.1f;
_AMAX	=	0.65f;
_ALPHA = 0.0f; //accelerateur de convergence
_ATTENUATOR = 0.1f; //attenuateur de uequ

_dt			=	dt;	//periode d'echantill.
_suivi	=	false; //pas de suivi de trajectoire

_m=0.0f; // karim 24 nov 2008
_sauv_alpha=_ALPHA; //karim 26 nov 2008
_k=0.0f;//karim 12 jan 2009
_sauv_k=_k; // karim 12 jan2009
_i=0.0f; // ajout karim
 _sauv_pression=0.0f; // ajout karim 17/02/2009
/* type de correcteur */
//------------------------
_type	=	_TW;


// karim ajout le 12 nov 09//
//-------------------------------

_j=0; // ajout karim le 12/11/2009
_id=0.0f; // ajout karim
 psdot= new float[35]; 

for(int pp=0;pp<=35;pp++)   
{	
	psdot[pp]=0.0f;
}


float pcof[]={-0.006, 0.002, 0.007, 0.014, 0.019, 0.020, 0.015, 0.003, -0.012, -0.028, -0.036, -0.032, -0.010, 0.026, 0.075, 0.125, 0.166, 0.190, 0.190, 0.166, 0.125, 0.075, 0.026, -0.010, -0.032, -0.036, -0.028, -0.012, 0.003, 0.015, 0.020, 0.019, 0.014, 0.007, 0.002, -0.006};

//-------------------------------



}

CorrecteurTW :: ~CorrecteurTW()
{
}

float	CorrecteurTW :: calculerCommande(float theta_desiree, float vitesse_desiree, float acceleration_desiree)
{
	float	u;
	u =0.0f;
	u	=	commandeTW(theta_desiree, vitesse_desiree, acceleration_desiree);
	return u;
}

float	CorrecteurTW :: commandeTW(float theta_desiree, float vitesse_desiree, float acceleration_desiree)
{
	float commande = 0.0f;
	float	theta_mesuree	= 0.0f;
	theta_mesuree =	(float)(_pCapteur->lirePositionRAD());
 
	
//if(momo)	{
	//_erreur	=	theta_mesuree	-	theta_desiree;

//}	else {
		//				_erreur	=0.0f;
				//			}//fin Ueq

   _erreur	=	theta_mesuree	-	theta_desiree;

	_erreur_derivee	=	(_erreur	-	_erreur_precedente)/_dt;
	_erreur_precedente	=	_erreur;





/* commande equivalente */
//-------------------------
if(_suivi)	{
		_uEQ = (((_A2-_C)*_erreur_derivee)
															+(_A1*_erreur)+(_A1*theta_desiree)
															+(_A2*vitesse_desiree)	+(acceleration_desiree)) / _B;

}	else {
							_uEQ = (((_A2-_C)*_erreur_derivee)
															+(_A1*_erreur)+(_A1*theta_desiree)) / _B;
							}//fin Ueq


_uEQ = _uEQ * (_ATTENUATOR);

/* surface et spoint */
//----------------------
_surface	=	(_C*_erreur) + _erreur_derivee;
_surface_derivee	=	(_surface - _surface_precedente)/_dt;
_surface_precedente	=	_surface;


/* terme discontinu */
//-----------------------


if(absOf(_uDELTA) <= _ULIM) {
		if((_surface * _surface_derivee) > 0.0) { _uDELTA_derivee	=	-(_AMAX * signOf(_surface));
}	else	{ _uDELTA_derivee	=	-(_AMIN * signOf(_surface));}
}	else	{ _uDELTA_derivee	=	-(_uDELTA);}

//avec acceleration de convergence





/*terme commande equivalente*/ //ajout karim le 13 nov09
//------------------------------

//_uEQ = _uEQ +_surface_derivee/_B;


/*for(int pp=35;pp>0;pp--)   
{	
psdot[1,pp]=psdot[1,pp-1];

}

psdot[0]=_surface_derivee/_B;

_id=0.0f;

for(int pp=0;pp<36;pp++)   
{	
	_id=_id+psdot[pp]*pcof[pp];
}

_uEQ=_uEQ+_id;*/


	
// karim ajout accelerateur 22 nov 2008

/*if (fabs(_erreur)>=0.3)	
					{									
					_uEQ = _uEQ +_surface_derivee/_B;
 					}

if (fabs(_erreur)>0.15)	
					{									
						_ALPHA=_sauv_alpha;

 					}*/

//_uDELTA_derivee = _uDELTA_derivee - _ALPHA*(_surface)*exp(-_m);		
							

// karim fin ajout accelerateur 22 nov 2008



/*if (fabs(_erreur)<=0.05)	
					{									
						_k=0.0f;//_sauv_k*exp(_i*50);

 					}*/

//_i=_i+1; // ajout karim 


_uDELTA_derivee = _uDELTA_derivee - _ALPHA*(_surface);//mourade chettouh originale

//fin acceleration de convergence


_uDELTA	=	(( _uDELTA_derivee * _dt) + _uDELTA_precedente); 

_uDELTA=_uDELTA-_k*absOf(_erreur)*signOf(_surface); // karim ajout acclerateur 

_uDELTA_precedente	=	_uDELTA;



/**************************ajout karim 17/02/2009*************************/
if (fabs(_erreur)<=0.05 )	// ancienne valeur 0.155 / 0.1
					{									
					_uDELTA_precedente= _sauv_pression;	
          _uDELTA=_sauv_pression;
          // momo=false;     
          //_k=0.0f;//_k/1000;
          //_ALPHA=0.0f;//_ALPHA/1000;

				}
else 
{_sauv_pression=_uDELTA; }

/************************************************************************/

//_uDELTA=-_AMAX* signOf(_surface); // premiere ordre

/* commande= ueq + u1 */
//-----------------------
commande = 	_uEQ + _uDELTA;


return	commande;

}//fin TW

/* fonction signOf */
//--------------------
float	CorrecteurTW :: signOf(const	float	value)
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
float CorrecteurTW :: absOf(const	float	value)
{
	if(value < 0.0f)
				return (value * -1.0f);
	else
				return	value;
}

/* INITialisation des variables */
//---------------------------------
void	CorrecteurTW :: initialise(float consigne)
{
	float	theta_mesuree	=	(float)(_pCapteur->lirePositionRAD());
	_erreur_precedente	=	theta_mesuree	-	consigne;
	_surface_precedente = _C * _erreur_precedente;

	_uEQ = 0.0f;
	_uDELTA_precedente	=	0.0f;
	_uDELTA	=	0.0f;
 
  _m=0.0f;  // karim ajout 24 nov 2008

}


/* lecture GET des variables du twist */
//------------------------------------
float	CorrecteurTW :: getC(void) {return _C;}
float	CorrecteurTW :: getUlim(void) {return _ULIM;}
float	CorrecteurTW :: getAmin(void) {return _AMIN;}
float	CorrecteurTW :: getAmax(void) {return _AMAX;}
//acceleration de convergence
float CorrecteurTW :: getAlpha(void) {_ALPHA=_sauv_alpha; return _ALPHA;}
//attenuateur de ueq
float CorrecteurTW :: getAttenuator(void) {return _ATTENUATOR;}
float CorrecteurTW :: getB(void) {return _B;} // ajout karim 17 nov

float	CorrecteurTW :: getUeq(void) {return _uEQ;}
float	CorrecteurTW :: getUdelta(void) {return _uDELTA;}
float	CorrecteurTW :: getSurface(void) {return _surface;}
float	CorrecteurTW :: getSurfacePoint(void) {return _surface_derivee;}
float	CorrecteurTW :: getErreur(void) {return _erreur;}


/* reglage des variables du twist */
//----------------------------------
void	CorrecteurTW	::	setC(float c){_C = c;}
void	CorrecteurTW	::	setUlim(float ulim){_ULIM = ulim;}
void	CorrecteurTW	::	setAmin(float amin){_AMIN = amin;}
void	CorrecteurTW	::	setAmax(float amax){_AMAX = amax;}
///////
void	CorrecteurTW	::	setAlpha(float alpha){_ALPHA = alpha;   _sauv_alpha=alpha;}
void	CorrecteurTW	::	setAttenuator(float attenuator){_ATTENUATOR = attenuator;}

void	CorrecteurTW	::	setSuivi(bool s){_suivi = s;}


float	CorrecteurTW :: getuDEUX(void)	{return 0.0f;}  //  karim ajout 18/12/08
float	CorrecteurTW :: get_k(void) {return _sauv_k;} // karim 12/01/09
void	CorrecteurTW ::	set_k(float k){_k = k; _sauv_k=k;} // karim 12/01/09