	/*	CORRECTEUR TW.h	*/

#include	"Correcteur.h"

class	Duree;

#ifndef	CORRECTEURTW_H
#define	CORRECTEURTW_H


class	CorrecteurTW	:	public	Correcteur
{
	public:

		CorrecteurTW(CapteurDePosition*, float, float, float, float);

		virtual ~CorrecteurTW();

		float	calculerCommande(float, float, float);
		float	commandeTW(float, float, float);

	//--------------------
	float	getC(void);
	float	getUlim(void);
	float	getAmin(void);
	float	getAmax(void);
	float getAlpha(void);//acceleration de convergence
	float	getAttenuator(void); //attenuateur de ueq
  float getB(void); // ajout 17 nov 09
	float	getUeq(void);
	float	getUdelta(void);
	float	getSurface(void);
	float	getSurfacePoint(void);
	float	getErreur(void);

  float	getuDEUX(void);// karim ajout 
  float	get_k(void);// karim ajout 12/01/09


	//--------------------
	void	setC(float);
	void	setUlim(float);
	void	setAmin(float);
	void	setAmax(float);
	void	setAlpha(float);//acceleration de convergence
	void	setAttenuator(float); //attenuateur de ueq
  void	set_k(float ); // ajout karim 12/01/09
	void	setSuivi(bool);
	void	initialise(float consigne);
  

  private:

		CapteurDePosition* _pCapteur;
    float *psdot; //ajout karim le 18/07/2008
    float *pcof; // ajout karim

		float	_B;
		float _A2;
		float _A1;
	
		float _C;
		float _ULIM;
		float _AMIN;
		float _AMAX;
		float _ALPHA;//acceleration de convergence
		float _ATTENUATOR;//attenuateur de ueq
		float _dt;
		bool	_suivi;

		float _uEQ;
		float _uDELTA;
		float _uDELTA_derivee;
		float _uDELTA_precedente;

		float _surface;
		float _m; // karim 24 nov 2008
    float _sauv_pression; // ajout karim le 17/02/2009

		float _surface_derivee;
		float _surface_precedente;

		float _erreur;
		float _erreur_precedente;
		float _erreur_derivee;
    float _sauv_alpha; // karim ajout 27/11/2008
    float _k; // karim ajout 12/01/2009
    float _sauv_k; // karim ajout 12/01/2009
    float _i; // ajout karim
    int _j; // ajout karim
    float _id; // ajout karim
    

		float signOf(const	float value);
		float absOf(const	float value);


};

#endif