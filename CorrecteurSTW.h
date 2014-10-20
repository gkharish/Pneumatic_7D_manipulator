		/*	CORRECTEURSTW.h	*/

#include	"Correcteur.h"

class Duree;

#ifndef	CORRECTEURSTW_H
#define CORRECTEURSTW_H

class	CorrecteurSTW : public Correcteur
{
	public:

	CorrecteurSTW(CapteurDePosition*, float, float, float, float);
	virtual	~CorrecteurSTW();

	float	calculerCommande(float,float, float);
	float	commandeSTW(float, float, float);
	
/* lecture via GET */
//-------------------
float	getC(void);
float	getUlim(void);
float	getW(void);
float	getLambda(void);
float	getSo(void);

float	getUeq(void);
float	getUdelta(void);
float	getSurface(void);
float	getSurfacePoint(void);
float	getErreur(void);
float getAlpha(void);//acceleration de convergence ajout karim 16/12/08
// karim function //
float	getuUN(void);
float	getuDEUX(void);
float	getAttenuator(void); //attenuateur de ueq zajout karim 26/05/09
 float	get_k(void);// karim ajout 26/05/09

/* ecriture via SET */
//---------------------
void	setC(float);
void	setUlim(float);
void	setW(float);
void	setLambda(float);
void	setSo(float);

void	setSuivi(bool);
void	initialise(float);
void	setAlpha(float);//acceleration de convergence ajout karim 
void	setAttenuator(float); //attenuateur de ueq
void	set_k(float ); // ajout karim 25/05/09

private:
	CapteurDePosition* _pCapteur;
	
	float	_B;
	float _A2;
	float _A1;
	
	float _C;
	float _ULIM;
	float	_W;
	float	_LAMBDA;
	float _S0;
	float _ALPHA;//acceleration de convergence ajout karim 16/12/08
  float _sauv_alpha; // karim ajout 17/12/2008
	float _ATTENUATOR;//attenuateur de ueq
  float _k; // karim ajout 25/05/2009
  float _sauv_pression; // ajout karim le 04/06/2009
	
	float _dt;
	bool	_suivi;

	float	_uEQ;
	float	_uDELTA;
	float	_uUN;
	float	_uDEUX;
	float	_uUN_precedente;
	float	_uUN_derivee;
	
	float	_surface;
	float _surface_precedente;
	float _surface_derivee;
	
	float	_erreur;
	float	_erreur_precedente;
	float	_erreur_derivee;

  

float	signOf(const	float	value);
//double
float	absOf(const	float	value);

};

#endif
