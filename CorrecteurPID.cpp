								/*CORRECTEURPID.CPP*/

#include "CorrecteurPID.h"


// ============================================================================= //
CorrecteurPID::CorrecteurPID(CapteurDePosition* pCapteurDePosition,
                             const float p,
                             const float i,
                             const float d,
														 const	float dt)
{
	_pCapteur = pCapteurDePosition;
  
  
  /* valeurs constants*/
 	_p = p;
	_i = i;
 	_d = d;
	_dt	=	dt;
  	
  _alpha	=	0.1f;

  ipid = 0.0f;
  //_alpha = 0.0f;
   kiki = 0.0f;
   
   _f = 0.0f;
  
  /* type de correcteur */
  _type = _PID;
}



// ============================================================================= //
CorrecteurPID::~CorrecteurPID()
{
}

// ============================================================================= //
float CorrecteurPID::calculerCommande(float theta_desiree, float vitesse_desiree, float acceleration_desiree)
{
  float	theta_mesuree	= 0.0f;
	theta_mesuree =	(float)(_pCapteur->lirePositionRAD());
  _erreur	=	theta_desiree	-	theta_mesuree;
	_erreur_derivee	=	(_erreur	-	_erreur_precedente)/_dt;
	_erreur_integree	=	_erreur_integree_precedente	+	(_erreur*_dt);

	   /*mise a jour */
  _erreur_precedente = _erreur;
	_erreur_integree_precedente	=	_erreur_integree;
     /* I PID */
    
    



	/* commande */
	float	commande	= (_p * _erreur) 
                 + (_i * _erreur_integree)
                 + (_d * _erreur_derivee);
  
  

  
    return commande;
}


//==================================================================================//


/* INItialisation du PID */
//-------------------------
void  CorrecteurPID::initialise(float consigne) 
{
//	_erreur	=	consigne;
	_erreur_precedente		=	consigne;

  _erreur_integree	 = 0.0f;
  _erreur_integree_precedente = 0.0f;

}





/* GET: Lecture du PID */
//------------------------
float CorrecteurPID::getP(void){	 return _p;  }

float CorrecteurPID::getI(void){  return _i;  }

float CorrecteurPID::getD(void){  return _d;  }

/* SET: Reglage du PID */
//------------------------
void CorrecteurPID::setP(float p){	 _p = p;  }
void CorrecteurPID::setI(float i){	 _i = i;  }
void CorrecteurPID::setD(float d){	 _d = d;  }


/* GET : Lecture de l'erreur */
//-----------------------------
float CorrecteurPID::getErreur(void){  return _erreur;  }
float	CorrecteurPID::getUeq(void){return	0.0;}
float	CorrecteurPID::getUdelta(void){return	0.0;}
float	CorrecteurPID::getSurface(void){return	0.0;}
float	CorrecteurPID::getSurfacePoint(void){return	0.0;}

float	CorrecteurPID::getuDEUX(void)	{return 0.0f;}  //  karim ajout 18/12/08




/* suivi de trajectoire */
//-------------------------
void	CorrecteurPID::setSuivi(bool){};

//void	CorrecteurPID::setSuivi(bool	s){_suivi=s;}



// fin de la classe CorrecteurPID