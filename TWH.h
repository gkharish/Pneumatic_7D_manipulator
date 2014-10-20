	/*	CORRECTEUR TWI.h	*/

#include	"Correcteur.h"

class	Duree;

#ifndef	CORRECTEURTWI_H
#define	CORRECTEURTWI_H



class	CorrecteurTWI	:	public	Correcteur
{
	public:
		CorrecteurTWI(CapteurDePosition*, float, float, float, float);

		virtual ~CorrecteurTWI();

		float	calculerCommande(float, float, float);
		float	commandeTWI(float, float, float);

	//--------------------
	float	getC(void);
	float	getUlim(void);
	float	getAmin(void);
	float	getAmax(void);
	float getP(void);
	float getI(void);

	float	getUeq(void);
	float	getUdelta(void);
	float	getSurface(void);
	float	getSurfacePoint(void);
	float	getErreur(void);


	//--------------------
	void	setC(float);
	void	setUlim(float);
	void	setAmin(float);
	void	setAmax(float);
	void	setP(float);
	void	setI(float);

	void	setSuivi(bool);
	void	initialise(float consigne);


	private:

		CapteurDePosition* _pCapteur;

		float	_B;
		float _A2;
		float _A1;

		float _C;
		float _ULIM;
		float _AMIN;
		float _AMAX;
		float _p;
		float _i;


		float _dt;
		bool	_suivi;

		float _uEQ;
		float _uDELTA;
		float _uDELTA_derivee;
		float _uDELTA_precedente;

		float _surface;
		float _surface_derivee;
		float _surface_precedente;

		float _erreur;
		float _erreur_precedente;
		float _erreur_derivee;


		float signOf(const	float value);
		float absOf(const	float value);


};

#endif