//#include "Correcteur.h"
class Duree;
// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/


#ifndef CORRECTEURPID2_H_HEADER_INCLUDED_BF1C714F
#define CORRECTEURPID2_H_HEADER_INCLUDED_BF1C714F



class CorrecteurPID : public Correcteur
{
  public:
    CorrecteurPID(CapteurDePosition* pCapteurDePosition,
                  const float p,
                  const float i,
                  const float d,
									const	float	dt);
    
    virtual ~CorrecteurPID();
    
    float calculerCommande(float theta_desiree,float vitesse_desiree,float acceleration_desiree);   
   
    
    //get : VALEURS du PID
    //------------------------------
		float getP(void);
		float getI(void);
		float getD(void);
    
    void setP(float);
    void setI(float);
    void setD(float);
   
    
    //get : VALEURS de l'ERREUR
    //------------------------------
    float getErreur(void);
		float getUeq(void);
		float getUdelta(void);
		float getSurface(void);
		float getSurfacePoint(void);

		float	getuDEUX(void); // Ajout Karim
   
    

		//INITIALISATION
		//-----------------
		void	initialise(float);
		void	setSuivi(bool);
       
    
 protected:

    CapteurDePosition *_pCapteur;
    
    
  private:
		/* variables constantes */
    //------------------------------
    float _dt;


    float _p;
    float _i;
    float _d;
    float ipid;
    float _alpha;
    float kiki;
    float _f;
		bool	_suivi;    
//		bool	_equiv;
    
    /* valeurs de l'erreur */
    //------------------------------
  	float	_erreur;  
		float _erreur_precedente;
    float _erreur_derivee;
    float _erreur_integree;    
    float _erreur_integree_precedente;


};



#endif /* CORRECTEURPID2_H_HEADER_INCLUDED_BF1C714F */