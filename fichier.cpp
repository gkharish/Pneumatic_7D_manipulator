/****************************************
 * Fichier fichier.cpp          	*
 * Jeremie Guiochet 			*
 * cree le 17/07/2002 par M. Seffar	*
 ****************************************/

#include "fichier.h"

//---------------------------------------------------------------------------

//600 est le nbre de points a enregistrer

fichier::fichier(char * chemin) {
	acces = (char *) malloc (1000); //malloc (600);
	strcpy(acces,chemin);
	}


/***************************************/
/* Méthodes de test pour l'ecriture dans un fichier txt*/
/***************************************/

//---------------------------------------------------------------------------
/***************************************/
/* Methodes pour l'ecriture au format Matlab*/
/* A partir des sources de Augustin Sanchez-1999*/
/***************************************/

 // ==============================================
 // ===== SAUVEGARDE DES DONNEES AU FORMAT MATLAB
 // ==============================================

 typedef struct
 {
  long type;         // == type
  long mrows;        // == row dimension
  long ncols;        // == column dimension
  long imagf;        // == flag indicating imag part
  long namlen;       // == name length (including NULL)
 } Fmatrix;


	void fichier::Sauve_Format_MATLAB(FILE *fp, long type, char *pname, long mrows,long ncols, long imagf, double *preal, float *pimag)

  {
  Fmatrix x;
  long mn;

  x.type     = type;
  x.mrows    = mrows;
  x.ncols    = ncols;
  x.imagf    = imagf;
  x.namlen   = strlen(pname) + 1;
  mn         = (long)(x.mrows * x.ncols);

  fwrite(&x, sizeof(Fmatrix), 1, fp);
  fwrite(pname, sizeof(char), (long)(x.namlen), fp);
  fwrite(preal, sizeof(double), mn, fp);

  if (imagf)
  { fwrite(pimag, sizeof(float), mn, fp); }
  
 }
 // == FIN PROCEDURE SAUVE_FORMAT_MATLAB()
/*****************************************************************************************/
void fichier::sauvegarde_trajectoire(char	*nom_fichier,
																	double *releve_temps,
																	double	*X,
																	double	*Y,
																	double **angle,
																	double **vitang,
																	double **accang)
{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;

	int	n_points = 1000; //600;//300;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);
  
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
    //releve du temps                   : (milliseconds)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
		this->Sauve_Format_MATLAB(FICH, 0, "XCART", 1, n_points, 0, X, (float*)0);
		this->Sauve_Format_MATLAB(FICH, 0, "YCART", 1, n_points, 0, Y, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "q2", 1, n_points, 0, angle[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "q4", 1, n_points, 0, angle[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "qdot2", 1, n_points, 0, vitang[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "qdot4", 1, n_points, 0, vitang[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "qddot2", 1, n_points, 0, accang[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "qddot4", 1, n_points, 0, accang[1],(float*)0);
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()

/****************************************************************************************/
void fichier::sauvegarde_identification(char   *nom_fichier,  //nom du fichier MATLAB
                                         int    n_axe,         //numero d'axe
                                        float  p_echelon,
															double *releve_temps, //releve du temps
															double *releve_theta, //releve du theta en RAD
															double *releve_pDelta)

{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;

	int	n_points = 1000; //600;//300;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);
  
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
/*
//position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
*/    
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
/*    
    //periode d'echantionnage
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "p_echantion", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
*/    
    //echelon
    valeur = p_echelon;
    this->Sauve_Format_MATLAB(FICH, 0, "echelon", 1, 1, 0, &valeur, (float*)0);
    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve du theta en RAD
    this->Sauve_Format_MATLAB(FICH, 0, "releve_theta", 1, n_points, 0, releve_theta, (float*)0);
/*    
    //releve du delta pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pM1",      1, n_points, 0, releve_m1,       (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pM2",      1, n_points, 0, releve_m2,       (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pBase",    1, n_points, 0, releve_pBase,    (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pRepos",   1, n_points, 0, releve_pRepos,   (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pEchelon", 1, n_points, 0, releve_pEchelon, (float*)0);
*/
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pDelta",   1, n_points, 0, releve_pDelta,   (float*)0);
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()

/////////////////////////// FIN SAUVEGARDE ECHELON DE PRESSION POUR UN AXE //////////////////////////
/**********************************************************************************************************/
void fichier::sauvegarde_identification2(char   *nom_fichier,  //nom du fichier MATLAB
                              double *releve_temps, //releve du temps
															double *releve_pression2, //releve des pressions
															double *releve_pression4,
															double *releve_theta_mesuree2, //releve des angles en RAD
															double *releve_theta_mesuree4)

{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;

	int	n_points = 1000; //600;//300;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);
  
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
/*
//position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
    
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
    
    //periode d'echantionnage
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "p_echantion", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
    
    //echelon
    valeur = p_echelon;
    this->Sauve_Format_MATLAB(FICH, 0, "echelon", 1, 1, 0, &valeur, (float*)0);
*/    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
/*****************************************/   
    //releve de la commande        : pression2 (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression2", 1, n_points, 0, releve_pression2, (float*)0);
    
    //releve de la commande       : pression4 (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression4", 1, n_points, 0, releve_pression4, (float*)0);
    
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree2", 1, n_points, 0, releve_theta_mesuree2, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree4", 1, n_points, 0, releve_theta_mesuree4, (float*)0);

/*    
    //releve du delta pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pM1",      1, n_points, 0, releve_m1,       (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pM2",      1, n_points, 0, releve_m2,       (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pBase",    1, n_points, 0, releve_pBase,    (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pRepos",   1, n_points, 0, releve_pRepos,   (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pEchelon", 1, n_points, 0, releve_pEchelon, (float*)0);
*/

  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()

///////////////////////////////////////////////////////////////////////////////////////////
/************************ FIN SAUVEGARDE ECHELONS DE PRESSION POUR 2 AXES ***************/
/////////////////////////////////////////////////////////////////////////////////////////////
void fichier::sauvegarde_PID(char   *nom_fichier,            //nom du fichier MATLAB
                             int    n_axe,                   //numero d'axe
										double *releve_temps,           //releve du temps
										double *releve_pression,        //releve du delta pression
										double *releve_theta_desiree,		//releve detheta desiree
										double *releve_theta_mesuree,   //releve du theta mesuree
										double *releve_erreur,          //releve de l'erreur ================ (radians)
                             float  p,                       //le coefficient P
                             float  i,                       //le coefficient I
                             float  d)                       //le coefficient D
 
{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;

	int	n_points = 1000; //600;//300;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);  
	
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
/*
    //position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
*/  
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
/*    
    //periode d'echantionnage
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "p_echantion", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
*/    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve du delta pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression", 1, n_points, 0, releve_pression, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree", 1, n_points, 0, releve_theta_mesuree, (float*)0);
/*    
    //releve de la vitesse mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_vmesuree", 1, n_points, 0, releve_vitesse_mesuree, (float*)0);
*/    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree", 1, n_points, 0, releve_theta_desiree, (float*)0);
/*    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle1", 1, n_points, 0, releve_m1, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle2", 1, n_points, 0, releve_m2, (float*)0);
*/    
    //releve de l'erreur                : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur", 1, n_points, 0, releve_erreur, (float*)0);
    
    //le coefficient P
    valeur = p;
    this->Sauve_Format_MATLAB(FICH, 0, "P", 1, 1, 0, &valeur, (float*)0);

    //le coefficient I
    valeur = i;
    this->Sauve_Format_MATLAB(FICH, 0, "I", 1, 1, 0, &valeur, (float*)0);

    //le coefficient D
    valeur = d;
    this->Sauve_Format_MATLAB(FICH, 0, "D", 1, 1, 0, &valeur, (float*)0);

/*    
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);
*/
    
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()

/**************************************************************************************/
void fichier::sauvegarde_TW(char   *nom_fichier,            //nom du fichier MATLAB
                            int    n_axe,                   //numero d'axe
									double *releve_temps,           //releve du temps =================== (milliseconds)
									double *releve_pression,        //releve de la commande (pression) == U      (bars)
									double *releve_Ueq,             //releve de la commande equivalente = Ueq    (bars)
									double *releve_Udelta,          //releve de la commande delta ======= Udelta (bars)
									double *releve_surface,
									double *releve_surface_derivee,
									double *releve_theta_mesuree,   //releve du theta mesuree =========== (radians)
									double *releve_theta_desiree,   //releve du theta mesuree =========== (radians)
									double *releve_erreur,          //releve de l'erreur ================ (radians)
                            float  C,                       //C  : parametre du TW
                            float  ULIM,
                            float  AMIN,
														float	 AMAX)

{
  FILE *FICH;
  
  char cheminComplet[50];
  double valeur;

	int	n_points = 1000; //600;//300;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);
  
	
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
/*
    //position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
*/    
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
/*    
    //periode d'echantillonnage        : (milliseconds)
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "e_periode", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
*/    
    //releve du temps                   : (milliseconds)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve de la commande (pression)  : pression (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression", 1, n_points, 0, releve_pression, (float*)0);
    
    //releve de la commande equivalente : Ueq    (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq", 1, n_points, 0, releve_Ueq, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta", 1, n_points, 0, releve_Udelta, (float*)0);
/*    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle1", 1, n_points, 0, releve_m1, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle2", 1, n_points, 0, releve_m2, (float*)0);
*/
		this->Sauve_Format_MATLAB(FICH, 0, "releve_surface", 1, n_points, 0, releve_surface, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_sderivee", 1, n_points, 0, releve_surface_derivee, (float*)0);    

    //releve du theta mesuree           : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree", 1, n_points, 0, releve_theta_mesuree, (float*)0);
    
    //releve du theta mesuree           : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree", 1, n_points, 0, releve_theta_desiree, (float*)0);

/*    
    //releve de la vitesse mesuree      : (radians/s)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_vmesuree", 1, n_points, 0, releve_vitesse_mesuree, (float*)0);
*/    
    //releve de l'erreur                : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur", 1, n_points, 0, releve_erreur, (float*)0);

		//C : parametre du TW
    valeur = C;
    this->Sauve_Format_MATLAB(FICH, 0, "C", 1, 1, 0, &valeur, (float*)0);

		valeur = ULIM;
    this->Sauve_Format_MATLAB(FICH, 0, "ULIM", 1, 1, 0, &valeur, (float*)0);

		valeur = AMIN;
    this->Sauve_Format_MATLAB(FICH, 0, "AMIN", 1, 1, 0, &valeur, (float*)0);

		valeur = AMAX;
    this->Sauve_Format_MATLAB(FICH, 0, "AMAX", 1, 1, 0, &valeur, (float*)0);

/*    
    //releve de la vitesse de l'erreur  : (radians/s)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_derreur", 1, n_points, 0, releve_erreur_derivee, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface", 1, n_points, 0, releve_surface, (float*)0);
    
    //C : parametre de la SV
    valeur = C;
    this->Sauve_Format_MATLAB(FICH, 0, "C", 1, 1, 0, &valeur, (float*)0);
    
    //K : parametre de la SV
    valeur = K;
    this->Sauve_Format_MATLAB(FICH, 0, "K", 1, 1, 0, &valeur, (float*)0);

    valeur = PHI;
    this->Sauve_Format_MATLAB(FICH, 0, "PHI", 1, 1, 0, &valeur, (float*)0);


    //a1    : parametre de la SV
    valeur = a1;
    this->Sauve_Format_MATLAB(FICH, 0, "a1", 1, 1, 0, &valeur, (float*)0);
    
    //a2    : parametre de la SV
    valeur = a2;
    this->Sauve_Format_MATLAB(FICH, 0, "a2", 1, 1, 0, &valeur, (float*)0);
    
    //b     : parametre de la SV
    valeur = b;
    this->Sauve_Format_MATLAB(FICH, 0, "b", 1, 1, 0, &valeur, (float*)0);

    
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);
*/    
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()



/**************************************************************************************/
void fichier::sauvegarde_STW(char   *nom_fichier,            //nom du fichier MATLAB
                            int    n_axe,                   //numero d'axe
									double *releve_temps,           //releve du temps =================== (milliseconds)
									double *releve_pression,        //releve de la commande (pression) == U      (bars)
									double *releve_Ueq,             //releve de la commande equivalente = Ueq    (bars)
									double *releve_Udelta,          //releve de la commande delta ======= Udelta (bars)
									double *releve_surface,
									double *releve_surface_derivee,
									double *releve_theta_mesuree,   //releve du theta mesuree =========== (radians)
									double *releve_theta_desiree,   //releve du theta mesuree =========== (radians)
									double *releve_erreur,          //releve de l'erreur ================ (radians)
                            float  C,                       //C  : parametre du TW
                            float  ULIM,
														float	 W,
                            float  LAMBDA,
														float	 S0)

{
  FILE *FICH;
  
  char cheminComplet[50];
  double valeur;

	int	n_points = 1000; //600;//300;

  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);
  
	
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
/*
    //position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
*/    
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
/*    
    //periode d'echantillonnage        : (milliseconds)
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "e_periode", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
*/    
    //releve du temps                   : (milliseconds)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve de la commande (pression)  : pression (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression", 1, n_points, 0, releve_pression, (float*)0);
    
    //releve de la commande equivalente : Ueq    (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq", 1, n_points, 0, releve_Ueq, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta", 1, n_points, 0, releve_Udelta, (float*)0);
/*    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle1", 1, n_points, 0, releve_m1, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle2", 1, n_points, 0, releve_m2, (float*)0);
*/
		this->Sauve_Format_MATLAB(FICH, 0, "releve_surface", 1, n_points, 0, releve_surface, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_sderivee", 1, n_points, 0, releve_surface_derivee, (float*)0);    

    //releve du theta mesuree           : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree", 1, n_points, 0, releve_theta_mesuree, (float*)0);
    
    //releve du theta mesuree           : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree", 1, n_points, 0, releve_theta_desiree, (float*)0);

/*    
    //releve de la vitesse mesuree      : (radians/s)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_vmesuree", 1, n_points, 0, releve_vitesse_mesuree, (float*)0);
*/    
    //releve de l'erreur                : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur", 1, n_points, 0, releve_erreur, (float*)0);

	//	C : parametre du TW
    valeur = C;
    this->Sauve_Format_MATLAB(FICH, 0, "C", 1, 1, 0, &valeur, (float*)0);

		valeur = ULIM;
    this->Sauve_Format_MATLAB(FICH, 0, "ULIM", 1, 1, 0, &valeur, (float*)0);

		valeur = W;
    this->Sauve_Format_MATLAB(FICH, 0, "W", 1, 1, 0, &valeur, (float*)0);

		valeur = LAMBDA;
    this->Sauve_Format_MATLAB(FICH, 0, "LAMBDA", 1, 1, 0, &valeur, (float*)0);

		valeur = S0;
    this->Sauve_Format_MATLAB(FICH, 0, "S0", 1, 1, 0, &valeur, (float*)0);

/*    
    //releve de la vitesse de l'erreur  : (radians/s)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_derreur", 1, n_points, 0, releve_erreur_derivee, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface", 1, n_points, 0, releve_surface, (float*)0);
    
    //C : parametre de la SV
    valeur = C;
    this->Sauve_Format_MATLAB(FICH, 0, "C", 1, 1, 0, &valeur, (float*)0);
    
    //K : parametre de la SV
    valeur = K;
    this->Sauve_Format_MATLAB(FICH, 0, "K", 1, 1, 0, &valeur, (float*)0);

    valeur = PHI;
    this->Sauve_Format_MATLAB(FICH, 0, "PHI", 1, 1, 0, &valeur, (float*)0);


    //a1    : parametre de la SV
    valeur = a1;
    this->Sauve_Format_MATLAB(FICH, 0, "a1", 1, 1, 0, &valeur, (float*)0);
    
    //a2    : parametre de la SV
    valeur = a2;
    this->Sauve_Format_MATLAB(FICH, 0, "a2", 1, 1, 0, &valeur, (float*)0);
    
    //b     : parametre de la SV
    valeur = b;
    this->Sauve_Format_MATLAB(FICH, 0, "b", 1, 1, 0, &valeur, (float*)0);

    
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);
*/    
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()


/***************************************************************************************/
void fichier::sauvegarde_PID2(char   *nom_fichier,             //nom du fichier MATLAB
										double *releve_temps,            //releve du temps
										double *releve_pression2,        //releve du delta pression
										double *releve_pression4,        //releve du delta pression
										double *releve_theta_desiree2,   //releve du theta desiree(consigne)
										double *releve_theta_desiree4,   //releve du theta desiree2(consigne)
										double *releve_theta_mesuree2,   //releve du theta mesuree
										double *releve_theta_mesuree4,   //releve du theta mesuree2
										double *releve_erreur2,          //releve de l'erreur ================ (radians)
										double *releve_erreur4,          //releve de l'erreur2 ================ (radians)
                             float  p2,                       //le coefficient P
                             float  i2,                       //le coefficient I
                             float  d2,                       //le coefficient D
                             float  p4,                       //le coefficient P2
                             float  i4,                       //le coefficient I2
                             float  d4)                       //le coefficient D2

{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;

	int		n_points = 1000; //600;//300;

  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);  
	
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {

/*
    //nombre de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);


/*
    //numero d'axe
    valeur = n_axe2;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe2", 1, 1, 0, &valeur, (float*)0);
    
    //numero d'axe
    valeur = n_axe4;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe4", 1, 1, 0, &valeur, (float*)0);
*/    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve de la pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression2", 1, n_points, 0, releve_pression2, (float*)0);
    
    //releve de la pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression4", 1, n_points, 0, releve_pression4, (float*)0);
    
    //releve du theta desiree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree2", 1, n_points, 0, releve_theta_desiree2, (float*)0);
    
    //releve du theta desiree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree4", 1, n_points, 0, releve_theta_desiree4, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree2", 1, n_points, 0, releve_theta_mesuree2, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree4", 1, n_points, 0, releve_theta_mesuree4, (float*)0);
    
    //releve de l'erreur
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur2", 1, n_points, 0, releve_erreur2, (float*)0);
    
    //releve de l'erreur
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur4", 1, n_points, 0, releve_erreur4, (float*)0);
    
    //le coefficient P
    valeur = p2;
    this->Sauve_Format_MATLAB(FICH, 0, "P_2", 1, 1, 0, &valeur, (float*)0);
    
    //le coefficient I
    valeur = i2;
    this->Sauve_Format_MATLAB(FICH, 0, "I_2", 1, 1, 0, &valeur, (float*)0);
    
    //le coefficient D
    valeur = d2;
    this->Sauve_Format_MATLAB(FICH, 0, "D_2", 1, 1, 0, &valeur, (float*)0);
    
    //le coefficient P
    valeur = p4;
    this->Sauve_Format_MATLAB(FICH, 0, "P_4", 1, 1, 0, &valeur, (float*)0);
    
    //le coefficient I
    valeur = i4;
    this->Sauve_Format_MATLAB(FICH, 0, "I_4", 1, 1, 0, &valeur, (float*)0);
    
    //le coefficient D
    valeur = d4;
    this->Sauve_Format_MATLAB(FICH, 0, "D_4", 1, 1, 0, &valeur, (float*)0);
    

/*
    //coordonnes cartesiennes theoriques
    this->Sauve_Format_MATLAB(FICH, 0, "ccXtheo", 1, n_points, 0, releveCCtheo[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccYtheo", 1, n_points, 0, releveCCtheo[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccZtheo", 1, n_points, 0, releveCCtheo[2],(float*)0);
    //coordonnes cartesiennes reelles
    this->Sauve_Format_MATLAB(FICH, 0, "ccXreel", 1, n_points, 0, releveCCreel[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccYreel", 1, n_points, 0, releveCCreel[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccZreel", 1, n_points, 0, releveCCreel[2],(float*)0);
    
    //axes du robot
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);
*/
    
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()

/***************************************************************************************/
void fichier::sauvegarde_TW2(char   *nom_fichier,             //nom du fichier MATLAB
											double *releve_temps,            //releve du temps
											double *releve_pression2,        //releve du delta pression
											double *releve_pression4,
 											double *releve_theta_desiree2,   //releve du theta desiree(consigne)
											double *releve_theta_desiree4,   
											double *releve_theta_mesuree2,   //releve du theta mesuree
											double *releve_theta_mesuree4,
 											double *releve_erreur2,          //releve de l'erreur ================ (radians)
											double *releve_erreur4,               
											double *releve_Ueq2,             //releve de la commande equivalente = Ueq    (bars)
											double *releve_Ueq4,             
											double *releve_Udelta2,          //releve de la commande delta ======= Udelta (bars)
											double *releve_Udelta4,          

/*											double *releve_vitesse_desiree2,
											double *releve_vitesse_desiree4, 
											double *releve_vitesse_mesuree2,
											double *releve_vitesse_mesuree4,   
*/									
											double *releve_surface2,         //releve de la surface
											double *releve_surface4,
											double *releve_surface_derivee2,	//releve de la derivee de la surface
											double *releve_surface_derivee4,
                                float  C2,                        //C  : parametre de la SV
                                float  ULIM2,
                                float  AMIN2,
																float  AMAX2,
																float		ALPHA2,
																float		ATTENUATOR2,
                                float  C4,                        //C  : parametre de la SV
                                float  ULIM4,
                                float  AMIN4,
																float  AMAX4,
																float		ALPHA4,
																float		ATTENUATOR4)

{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;

	int		n_points = 1000; //600;//300;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);  
	
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
/*
		//nombre de points enregistres
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);

/*
    //numero d'axe
    valeur = n_axe2;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe2", 1, 1, 0, &valeur, (float*)0);
    
    //numero d'axe
    valeur = n_axe4;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe4", 1, 1, 0, &valeur, (float*)0);
*/    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve de la pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression2", 1, n_points, 0, releve_pression2, (float*)0);
    
    //releve de la pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression4", 1, n_points, 0, releve_pression4, (float*)0);
    
    //releve de la commande equivalente : Ueq    (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq2", 1, n_points, 0, releve_Ueq2, (float*)0);
    
    //releve de la commande equivalente : Ueq    (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq4", 1, n_points, 0, releve_Ueq4, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta2", 1, n_points, 0, releve_Udelta2, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta4", 1, n_points, 0, releve_Udelta4, (float*)0);
    
    //releve du theta desiree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree2", 1, n_points, 0, releve_theta_desiree2, (float*)0);
    
    //releve du theta desiree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree4", 1, n_points, 0, releve_theta_desiree4, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree2", 1, n_points, 0, releve_theta_mesuree2, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree4", 1, n_points, 0, releve_theta_mesuree4, (float*)0);
/* 
		//releve des vitesses desirees
    this->Sauve_Format_MATLAB(FICH, 0, "releve_vitdesiree2", 1, n_points, 0, releve_vitesse_desiree2, (float*)0);

    this->Sauve_Format_MATLAB(FICH, 0, "releve_vitdesiree4", 1, n_points, 0, releve_vitesse_desiree4, (float*)0);

		//releve des vitesses mesurees
    this->Sauve_Format_MATLAB(FICH, 0, "releve_vitmesuree2", 1, n_points, 0, releve_vitesse_mesuree2, (float*)0);

    this->Sauve_Format_MATLAB(FICH, 0, "releve_vitmesuree4", 1, n_points, 0, releve_vitesse_mesuree4, (float*)0);
*/
    //releve de l'erreur
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur2", 1, n_points, 0, releve_erreur2, (float*)0);
    
    //releve de l'erreur
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur4", 1, n_points, 0, releve_erreur4, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface2", 1, n_points, 0, releve_surface2, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface4", 1, n_points, 0, releve_surface4, (float*)0);

    this->Sauve_Format_MATLAB(FICH, 0, "releve_sderivee2", 1, n_points, 0, releve_surface_derivee2, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_sderivee4", 1, n_points, 0, releve_surface_derivee4, (float*)0);




    
    //C : parametre de la SV
    valeur = C2;
    this->Sauve_Format_MATLAB(FICH, 0, "C_2", 1, 1, 0, &valeur, (float*)0);
    
    //ULIM : parametre de la SV
    valeur = ULIM2;
    this->Sauve_Format_MATLAB(FICH, 0, "ULIM_2", 1, 1, 0, &valeur, (float*)0);
    
    valeur = AMIN2;
    this->Sauve_Format_MATLAB(FICH, 0, "AMIN_2", 1, 1, 0, &valeur, (float*)0);

    valeur = AMAX2;
    this->Sauve_Format_MATLAB(FICH, 0, "AMAX_2", 1, 1, 0, &valeur, (float*)0);

		valeur = ALPHA2;
    this->Sauve_Format_MATLAB(FICH, 0, "ALPHA_2", 1, 1, 0, &valeur, (float*)0);

		valeur = ATTENUATOR2;
    this->Sauve_Format_MATLAB(FICH, 0, "ATTENUATOR_2", 1, 1, 0, &valeur, (float*)0);
    
    //C : parametre de la SV
    valeur = C4;
    this->Sauve_Format_MATLAB(FICH, 0, "C_4", 1, 1, 0, &valeur, (float*)0);
    
    //ULIM : parametre de la SV
    valeur = ULIM4;
    this->Sauve_Format_MATLAB(FICH, 0, "ULIM_4", 1, 1, 0, &valeur, (float*)0);
    
    valeur = AMIN4;
    this->Sauve_Format_MATLAB(FICH, 0, "AMIN_4", 1, 1, 0, &valeur, (float*)0);

    valeur = AMAX4;
    this->Sauve_Format_MATLAB(FICH, 0, "AMAX_4", 1, 1, 0, &valeur, (float*)0);		

		valeur = ALPHA4;
    this->Sauve_Format_MATLAB(FICH, 0, "ALPHA_4", 1, 1, 0, &valeur, (float*)0);

		valeur = ATTENUATOR4;
    this->Sauve_Format_MATLAB(FICH, 0, "ATTENUATOR_4", 1, 1, 0, &valeur, (float*)0);

/*    
    //coordonnes cartesiennes theoriques
    this->Sauve_Format_MATLAB(FICH, 0, "ccXtheo", 1, n_points, 0, releveCCtheo[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccYtheo", 1, n_points, 0, releveCCtheo[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccZtheo", 1, n_points, 0, releveCCtheo[2],(float*)0);
    //coordonnes cartesiennes reelles
    this->Sauve_Format_MATLAB(FICH, 0, "ccXreel", 1, n_points, 0, releveCCreel[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccYreel", 1, n_points, 0, releveCCreel[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccZreel", 1, n_points, 0, releveCCreel[2],(float*)0);
    
    //axes du robot
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);


*/    
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()



/******************************************************************************************/
void fichier::sauvegarde_STW2(char   *nom_fichier,             //nom du fichier MATLAB
											double *releve_temps,            //releve du temps
											double *releve_pression2,        //releve du delta pression
											double *releve_pression4,        
											double *releve_Ueq2,             //releve de la commande equivalente = Ueq    (bars)
											double *releve_Ueq4,             
											double *releve_Udelta2,          //releve de la commande delta ======= Udelta (bars)
											double *releve_Udelta4,          
											double *releve_theta_desiree2,   //releve du theta desiree(consigne)
											double *releve_theta_desiree4,   
											double *releve_theta_mesuree2,   //releve du theta mesuree
											double *releve_theta_mesuree4,   
											double *releve_erreur2,          //releve de l'erreur ================ (radians)
											double *releve_erreur4,          
											double *releve_surface2,         //releve de la surface
											double *releve_surface4,
											double *releve_surface_derivee2,	//releve de la derivee de la surface
											double *releve_surface_derivee4,
                                float  C2,                        //C  : parametre de la SV
                                float  ULIM2,
                                float  W2,
																float  LAMBDA2,
																float  S02,
                                float  C4,                        //C  : parametre de la SV
                                float  ULIM4,
                                float  W4,
																float  LAMBDA4,
																float  S04)

{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;

	int		n_points = 1000; //600;//300;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);  
	
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {

/*
		//nombre de points enregistres
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);

/*
    //numero d'axe
    valeur = n_axe2;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe2", 1, 1, 0, &valeur, (float*)0);
    
    //numero d'axe
    valeur = n_axe4;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe4", 1, 1, 0, &valeur, (float*)0);
*/    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve de la pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression2", 1, n_points, 0, releve_pression2, (float*)0);
    
    //releve de la pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression4", 1, n_points, 0, releve_pression4, (float*)0);
    
    //releve de la commande equivalente : Ueq    (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq2", 1, n_points, 0, releve_Ueq2, (float*)0);
    
    //releve de la commande equivalente : Ueq    (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq4", 1, n_points, 0, releve_Ueq4, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta2", 1, n_points, 0, releve_Udelta2, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta4", 1, n_points, 0, releve_Udelta4, (float*)0);
    
    //releve du theta desiree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree2", 1, n_points, 0, releve_theta_desiree2, (float*)0);
    
    //releve du theta desiree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree4", 1, n_points, 0, releve_theta_desiree4, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree2", 1, n_points, 0, releve_theta_mesuree2, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree4", 1, n_points, 0, releve_theta_mesuree4, (float*)0);
    
    //releve de l'erreur
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur2", 1, n_points, 0, releve_erreur2, (float*)0);
    
    //releve de l'erreur
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur4", 1, n_points, 0, releve_erreur4, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface2", 1, n_points, 0, releve_surface2, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface4", 1, n_points, 0, releve_surface4, (float*)0);

    this->Sauve_Format_MATLAB(FICH, 0, "releve_sderivee2", 1, n_points, 0, releve_surface_derivee2, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_sderivee4", 1, n_points, 0, releve_surface_derivee4, (float*)0);




    
    //C : parametre de la SV
    valeur = C2;
    this->Sauve_Format_MATLAB(FICH, 0, "C_2", 1, 1, 0, &valeur, (float*)0);
    
    //ULIM : parametre de la SV
    valeur = ULIM2;
    this->Sauve_Format_MATLAB(FICH, 0, "ULIM_2", 1, 1, 0, &valeur, (float*)0);
    
    valeur = W2;
    this->Sauve_Format_MATLAB(FICH, 0, "W_2", 1, 1, 0, &valeur, (float*)0);

    valeur = LAMBDA2;
    this->Sauve_Format_MATLAB(FICH, 0, "LAMBDA_2", 1, 1, 0, &valeur, (float*)0);

		valeur = S02;
    this->Sauve_Format_MATLAB(FICH, 0, "S0_2", 1, 1, 0, &valeur, (float*)0);
    
    //C : parametre de la SV
    valeur = C4;
    this->Sauve_Format_MATLAB(FICH, 0, "C_4", 1, 1, 0, &valeur, (float*)0);
    
    //ULIM : parametre de la SV
    valeur = ULIM4;
    this->Sauve_Format_MATLAB(FICH, 0, "ULIM_4", 1, 1, 0, &valeur, (float*)0);
    
    valeur = W4;
    this->Sauve_Format_MATLAB(FICH, 0, "W_4", 1, 1, 0, &valeur, (float*)0);

    valeur = LAMBDA4;
    this->Sauve_Format_MATLAB(FICH, 0, "LAMBDA_4", 1, 1, 0, &valeur, (float*)0);

		valeur = S04;
    this->Sauve_Format_MATLAB(FICH, 0, "S0_4", 1, 1, 0, &valeur, (float*)0);
		


/*    
    //coordonnes cartesiennes theoriques
    this->Sauve_Format_MATLAB(FICH, 0, "ccXtheo", 1, n_points, 0, releveCCtheo[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccYtheo", 1, n_points, 0, releveCCtheo[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccZtheo", 1, n_points, 0, releveCCtheo[2],(float*)0);
    //coordonnes cartesiennes reelles
    this->Sauve_Format_MATLAB(FICH, 0, "ccXreel", 1, n_points, 0, releveCCreel[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccYreel", 1, n_points, 0, releveCCreel[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccZreel", 1, n_points, 0, releveCCreel[2],(float*)0);
    
    //axes du robot
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);


*/    
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()



//***********************************************************************************************************************************************//
//********************************partie ajoutee par Amar REZOUG le 12-04-2012 pour sauvgarder les donnees issues de lexpirimentation de TSMC***//
//**********************************************************************************************************************************************//
/*
void fichier::sauvegarde_TSMC(char   *nom_fichier,             //nom du fichier MATLAB
											double *releve_temps,            //releve du temps
											double *releve_pression2,        //releve du delta pression
											double *releve_pression4,        
											double *releve_Ueq2,             //releve de la commande equivalente = Ueq    (bars)
											double *releve_Ueq4,             
											double *releve_Udelta2,          //releve de la commande delta ======= Udelta (bars)
											double *releve_Udelta4,          
											double *releve_theta_desiree2,   //releve du theta desiree(consigne)
											double *releve_theta_desiree4,   
											double *releve_theta_mesuree2,   //releve du theta mesuree
											double *releve_theta_mesuree4,   
											double *releve_erreur2,          //releve de l'erreur ================ (radians)
											double *releve_erreur4,          
											double *releve_surface2,         //releve de la surface
											double *releve_surface4,
											double *releve_surface_derivee2,	//releve de la derivee de la surface
											double *releve_surface_derivee4,
                                int  q2,                        //C  : parametre de la SV
                                int  p2,
                                float  Mbar2,
																float  Lamda2,
																float  KWs2,
                                int  q4,                        //C  : parametre de la SV
                                int  p4,
                                float  Mbar2,
																float  Lamda4,
																float  KWs4)

{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;

	int		n_points = 1000; //600;//300;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);  
	
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
*/
/*
		//nombre de points enregistres
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);

/*
    //numero d'axe
    valeur = n_axe2;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe2", 1, 1, 0, &valeur, (float*)0);
    
    //numero d'axe
    valeur = n_axe4;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe4", 1, 1, 0, &valeur, (float*)0);
    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve de la pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression2", 1, n_points, 0, releve_pression2, (float*)0);
    
    //releve de la pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression4", 1, n_points, 0, releve_pression4, (float*)0);
    
 //   //releve de la commande equivalente : Ueq    (bars)
   // this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq2", 1, n_points, 0, releve_Ueq2, (float*)0);
    
    //releve de la commande equivalente : Ueq    (bars)
  //  this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq4", 1, n_points, 0, releve_Ueq4, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
 //   this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta2", 1, n_points, 0, releve_Udelta2, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
   // this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta4", 1, n_points, 0, releve_Udelta4, (float*)0);
    
    //releve du theta desiree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree2", 1, n_points, 0, releve_theta_desiree2, (float*)0);
    
    //releve du theta desiree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree4", 1, n_points, 0, releve_theta_desiree4, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree2", 1, n_points, 0, releve_theta_mesuree2, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree4", 1, n_points, 0, releve_theta_mesuree4, (float*)0);
    
    //releve de l'erreur
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur2", 1, n_points, 0, releve_erreur2, (float*)0);
    
    //releve de l'erreur
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur4", 1, n_points, 0, releve_erreur4, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface2", 1, n_points, 0, releve_surface2, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface4", 1, n_points, 0, releve_surface4, (float*)0);

    this->Sauve_Format_MATLAB(FICH, 0, "releve_sderivee2", 1, n_points, 0, releve_surface_derivee2, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_sderivee4", 1, n_points, 0, releve_surface_derivee4, (float*)0);

*/


    /*
    //C : parametre de la SV
    valeur = q2;
    this->Sauve_Format_MATLAB(FICH, 0, "q_2", 1, 1, 0, &valeur, (float*)0);
    
    //ULIM : parametre de la SV
    valeur = p2;
    this->Sauve_Format_MATLAB(FICH, 0, "p_2", 1, 1, 0, &valeur, (float*)0);
    
    valeur = Mbar2;
    this->Sauve_Format_MATLAB(FICH, 0, "Mbar_2", 1, 1, 0, &valeur, (float*)0);

    valeur = Lamda2;
    this->Sauve_Format_MATLAB(FICH, 0, "Lamda_2", 1, 1, 0, &valeur, (float*)0);

		valeur = KWs2;
    this->Sauve_Format_MATLAB(FICH, 0, "KWs_2", 1, 1, 0, &valeur, (float*)0);
    
    //C : parametre de la SV
    valeur = p4;
    this->Sauve_Format_MATLAB(FICH, 0, "q_4", 1, 1, 0, &valeur, (float*)0);
    
    //ULIM : parametre de la SV
    valeur = q4;
    this->Sauve_Format_MATLAB(FICH, 0, "p_4", 1, 1, 0, &valeur, (float*)0);
    
    valeur = Mbar4;
    this->Sauve_Format_MATLAB(FICH, 0, "Mbar_4", 1, 1, 0, &valeur, (float*)0);

    valeur = Lamda4;
    this->Sauve_Format_MATLAB(FICH, 0, "LAMBDA_4", 1, 1, 0, &valeur, (float*)0);

		valeur =KWs4;
    this->Sauve_Format_MATLAB(FICH, 0, "KWs_4", 1, 1, 0, &valeur, (float*)0);
		
*/

/*    
    //coordonnes cartesiennes theoriques
    this->Sauve_Format_MATLAB(FICH, 0, "ccXtheo", 1, n_points, 0, releveCCtheo[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccYtheo", 1, n_points, 0, releveCCtheo[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccZtheo", 1, n_points, 0, releveCCtheo[2],(float*)0);
    //coordonnes cartesiennes reelles
    this->Sauve_Format_MATLAB(FICH, 0, "ccXreel", 1, n_points, 0, releveCCreel[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccYreel", 1, n_points, 0, releveCCreel[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "ccZreel", 1, n_points, 0, releveCCreel[2],(float*)0);
    
    //axes du robot
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);


*/   
/*
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()

*/

//***********************************************************************************************************************************************//
//********************************partie ajoutee par Amar REZOUG le 12-04-2012 pour sauvgarder les donnees issues de lexpirimentation de TSMC***//
//**********************************************************************************************************************************************//









/*


// ======================================================
// = SAUVEGARDE DES VALEURS ACQUISES PAR L'IDENTIFICATION
// = DANS UN FICHIER MATLAB (ruth)
// ======================================================
void fichier::sauvegarde_identification(char   *nom_fichier,  //nom du fichier MATLAB
                                        double *theta_art,    //position initial de chaque articulation du robot
                                        int    n_axe,         //numero d'axe
                                        int    e_periode,     //periode d'echantionnage
                                        int    n_points,      //numero de points
                                        float  p_echelon,
                                        double *releve_temps, //releve du temps
                                        double *releve_theta, //releve du theta en RAD
                                        double *releve_m1, 
                                        double *releve_m2,
                                        double *releve_pBase,
                                        double *releve_pRepos,
                                        double *releve_pEchelon,
                                        double *releve_pDelta)

{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);
  
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
    //position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
    
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
    
    //periode d'echantionnage
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "p_echantion", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
    
    //echelon
    valeur = p_echelon;
    this->Sauve_Format_MATLAB(FICH, 0, "echelon", 1, 1, 0, &valeur, (float*)0);
    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve du theta en RAD
    this->Sauve_Format_MATLAB(FICH, 0, "releve_theta", 1, n_points, 0, releve_theta, (float*)0);
    
    //releve du delta pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pM1",      1, n_points, 0, releve_m1,       (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pM2",      1, n_points, 0, releve_m2,       (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pBase",    1, n_points, 0, releve_pBase,    (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pRepos",   1, n_points, 0, releve_pRepos,   (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pEchelon", 1, n_points, 0, releve_pEchelon, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pDelta",   1, n_points, 0, releve_pDelta,   (float*)0);
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()



// ======================================================
// = SAUVEGARDE DES VALEURS ACQUISES PAR L'IDENTIFICATION
// = DANS UN FICHIER MATLAB (ruth)
// ======================================================
void fichier::sauvegarde_identification(char   *nom_fichier,  //nom du fichier MATLAB
                                        double *theta_art,    //position initial de chaque articulation du robot
                                        int    n_axe,         //numero d'axe
                                        int    e_periode,     //periode d'echantionnage
                                        int    n_points,      //numero de points
                                        float  creneau1,
                                        float  creneau2,
                                        double *releve_temps, //releve du temps
                                        double *releve_theta, //releve du theta en RAD
                                        double *releve_m1, 
                                        double *releve_m2,
                                        double *releve_pBase,
                                        double *releve_pRepos,
                                        double *releve_pEchelon,
                                        double *releve_pDelta,
                                        double **axes)
{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);
  
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
    //position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
    
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
    
    //periode d'echantionnage
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "p_echantion", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
    
    //echelon
    valeur = creneau1;
    this->Sauve_Format_MATLAB(FICH, 0, "creneau1", 1, 1, 0, &valeur, (float*)0);
    //echelon
    valeur = creneau2;
    this->Sauve_Format_MATLAB(FICH, 0, "creneau2", 1, 1, 0, &valeur, (float*)0);
    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve du theta en RAD
    this->Sauve_Format_MATLAB(FICH, 0, "releve_theta", 1, n_points, 0, releve_theta, (float*)0);
    
    //releve du delta pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pM1",      1, n_points, 0, releve_m1,       (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pM2",      1, n_points, 0, releve_m2,       (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pBase",    1, n_points, 0, releve_pBase,    (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pRepos",   1, n_points, 0, releve_pRepos,   (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pEchelon", 1, n_points, 0, releve_pEchelon, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pDelta",   1, n_points, 0, releve_pDelta,   (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()





// ========================================================================
// = SAUVEGARDE DES VALEURS ACQUISES PAR LE TEST DU CORRECTEUR PID
// = DANS UN FICHIER MATLAB (ruth)
// ========================================================================
void fichier::sauvegarde_PID(char   *nom_fichier,            //nom du fichier MATLAB
                             double *theta_art,              //position initial de chaque articulation
                             int    n_axe,                   //numero d'axe
                             int    e_periode,               //periode d'echantionnage
                             int    n_points,                //numero de points
                             double *releve_temps,           //releve du temps
                             double *releve_pression,        //releve du delta pression
                             double *releve_theta_mesuree,   //releve du theta desiree(consigne)
                             double *releve_vitesse_mesuree, //releve de vitesse
                             double *releve_theta_desiree,   //releve du theta desiree(consigne)
                             double *releve_erreur,          //releve de l'erreur ================ (radians)
                             double *releve_m1,
                             double *releve_m2,
                             float  p,                       //le coefficient P
                             float  i,                       //le coefficient I
                             float  d,                       //le coefficient D
                             double **axes)
{
  FILE *FICH;
  
  char cheminComplet[50];
	double valeur;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);  
	
  
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
    //position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
    
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
    
    //periode d'echantionnage
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "p_echantion", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
    
    //releve du temps
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve du delta pression
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression", 1, n_points, 0, releve_pression, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree", 1, n_points, 0, releve_theta_mesuree, (float*)0);
    
    //releve de la vitesse mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_vmesuree", 1, n_points, 0, releve_vitesse_mesuree, (float*)0);
    
    //releve du theta mesuree
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree", 1, n_points, 0, releve_theta_desiree, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle1", 1, n_points, 0, releve_m1, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle2", 1, n_points, 0, releve_m2, (float*)0);
    
    //releve de l'erreur                : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur", 1, n_points, 0, releve_erreur, (float*)0);
    
    //le coefficient P
    valeur = p;
    this->Sauve_Format_MATLAB(FICH, 0, "P", 1, 1, 0, &valeur, (float*)0);

    //le coefficient I
    valeur = i;
    this->Sauve_Format_MATLAB(FICH, 0, "I", 1, 1, 0, &valeur, (float*)0);

    //le coefficient D
    valeur = d;
    this->Sauve_Format_MATLAB(FICH, 0, "D", 1, 1, 0, &valeur, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);
    
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()



// ========================================================================
// = SAUVEGARDE DES VALEURS ACQUISES PAR LE TEST DU CORRECTEUR SV
// = DANS UN FICHIER MATLAB (ruth)
// ========================================================================
void fichier::sauvegarde_SV(char   *nom_fichier,            //nom du fichier MATLAB
                            double *theta_art,              //position initial de chaque articulation
                            int    n_axe,                   //numero d'axe
                            int    e_periode,               //periode d'echantionnage
                            int    n_points,                //numero de points
                            double *releve_temps,           //releve du temps =================== (milliseconds)
                            double *releve_pression,        //releve de la commande (pression) == U      (bars)
                            double *releve_Ueq,             //releve de la commande equivalente = Ueq    (bars)
                            double *releve_Udelta,          //releve de la commande delta ======= Udelta (bars)
                            double *releve_m1,
                            double *releve_m2,
                            double *releve_theta_mesuree,   //releve du theta mesuree =========== (radians)
                            double *releve_theta_desiree,   //releve du theta mesuree =========== (radians)
                            double *releve_vitesse_mesuree, //releve de la vitesse mesuree ====== (radians)
                            double *releve_erreur,          //releve de l'erreur ================ (radians)
                            double *releve_erreur_derivee,  //releve de la vitesse de l'erreur == (radians/s)
                            double *releve_surface,         //
                            float  C,                       //C  : parametre de la SV
                            float  K,
                            float  PHI,
                            double **axes)
{
  FILE *FICH;
  
  char cheminComplet[50];
  double valeur;
  
  strcpy(cheminComplet,acces);
  strcat(cheminComplet,nom_fichier);
  strcat(cheminComplet,".mat");
  printf("\n Sauvegarde en cours dans : \n%s\nCela peut prendre plusieurs minutes...", acces);
  
	
  if((FICH = fopen(cheminComplet,"w+b")) != NULL)
  {
    //position initial de chaque articulation
    this->Sauve_Format_MATLAB(FICH, 0, "art_initial", 1, 7, 0, theta_art, (float*)0);
    
    //numero d'axe
    valeur = n_axe;
    this->Sauve_Format_MATLAB(FICH, 0, "n_axe", 1, 1, 0, &valeur, (float*)0);
    
    //periode d'echantillonnage        : (milliseconds)
    valeur = e_periode * 0.001;
    this->Sauve_Format_MATLAB(FICH, 0, "e_periode", 1, 1, 0, &valeur, (float*)0);
    
    //numero de points
    valeur = n_points;
    this->Sauve_Format_MATLAB(FICH, 0, "n_points", 1, 1, 0, &valeur, (float*)0);
    
    //releve du temps                   : (milliseconds)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_temps", 1, n_points, 0, releve_temps, (float*)0);
    
    //releve de la commande (pression)  : pression (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_pression", 1, n_points, 0, releve_pression, (float*)0);
    
    //releve de la commande equivalente : Ueq    (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Ueq", 1, n_points, 0, releve_Ueq, (float*)0);
    
    //releve de la commande delta       : Udelta (bars)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_Udelta", 1, n_points, 0, releve_Udelta, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle1", 1, n_points, 0, releve_m1, (float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "releve_muscle2", 1, n_points, 0, releve_m2, (float*)0);
    
    //releve du theta mesuree           : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tmesuree", 1, n_points, 0, releve_theta_mesuree, (float*)0);
    
    //releve du theta mesuree           : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_tdesiree", 1, n_points, 0, releve_theta_desiree, (float*)0);
    
    //releve de la vitesse mesuree      : (radians/s)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_vmesuree", 1, n_points, 0, releve_vitesse_mesuree, (float*)0);
    
    //releve de l'erreur                : (radians)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_erreur", 1, n_points, 0, releve_erreur, (float*)0);
    
    //releve de la vitesse de l'erreur  : (radians/s)
    this->Sauve_Format_MATLAB(FICH, 0, "releve_derreur", 1, n_points, 0, releve_erreur_derivee, (float*)0);
    
    this->Sauve_Format_MATLAB(FICH, 0, "releve_surface", 1, n_points, 0, releve_surface, (float*)0);
    
    //C : parametre de la SV
    valeur = C;
    this->Sauve_Format_MATLAB(FICH, 0, "C", 1, 1, 0, &valeur, (float*)0);
    
    //K : parametre de la SV
    valeur = K;
    this->Sauve_Format_MATLAB(FICH, 0, "K", 1, 1, 0, &valeur, (float*)0);

    valeur = PHI;
    this->Sauve_Format_MATLAB(FICH, 0, "PHI", 1, 1, 0, &valeur, (float*)0);

    /*
    //a1    : parametre de la SV
    valeur = a1;
    this->Sauve_Format_MATLAB(FICH, 0, "a1", 1, 1, 0, &valeur, (float*)0);
    
    //a2    : parametre de la SV
    valeur = a2;
    this->Sauve_Format_MATLAB(FICH, 0, "a2", 1, 1, 0, &valeur, (float*)0);
    
    //b     : parametre de la SV
    valeur = b;
    this->Sauve_Format_MATLAB(FICH, 0, "b", 1, 1, 0, &valeur, (float*)0);
    */
/*    
    this->Sauve_Format_MATLAB(FICH, 0, "axe1", 1, n_points, 0, axes[0],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe2", 1, n_points, 0, axes[1],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe3", 1, n_points, 0, axes[2],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe4", 1, n_points, 0, axes[3],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe5", 1, n_points, 0, axes[4],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe6", 1, n_points, 0, axes[5],(float*)0);
    this->Sauve_Format_MATLAB(FICH, 0, "axe7", 1, n_points, 0, axes[6],(float*)0);
    
  } else {
    perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH");
  }
  
  fclose(FICH);
}
// == FIN PROCEDURE SAUVEGARDE()



 // ======================================================
 // ===== renommer l'accès au fichier MATLAB
 // ======================================================

void fichier::renommer(char* chemin)
{
   printf ("L'acces de votre fichier Matlab est renomme\n"); 
   acces = chemin; // récupération du chemin d'accès au fichier MATLAB
}
*/

