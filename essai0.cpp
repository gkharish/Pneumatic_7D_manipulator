
/* CECI EST le travail de K BRAIKIA*/
/* projet these */
/* kB*/
// la commande est saturee a 1 bar dans SATMAX
//la vitesse de gonflement est limitee a 0.2bar/cycle dans ECHELON_MAX
//de actionneur.h
//la procedure de degonflement est actionnee au demarrage
 
/* longueurs des segments du bras */
#define l1	0.352f //longueur bras en m
#define l2	0.309f //longueur avant bras en m

/* nbre d axes */
#define NB_AXES 	7

/* affectation des registres des cartes I/O */
#define BASE_REG_CIODAC16		0x200
#define BASE_REG_CIODAS64		0x250


/* plage des variations des axes 2 et 4 */

#define AXE_2_MIN   	0.0f
#define AXE_2_MAX   	80.0f //90.0
#define AXE_4_MIN     0.0f
#define AXE_4_MAX     100.0f //110.0


/* karim ajout axe 1 le 29 oct 08 */
#define AXE_1_MIN     0.0f
#define AXE_1_MAX     90.0f //110.0
/* karim ajout axe 1 le 29 oct 08 */

/* gains en BO deltaP = gain*position (en deg.)+ DeltaInit */
#define GAIN_BO_DEGRE_AXE4	0.030f
#define GAIN_BO_DEGRE_AXE2	0.039f
//#define GAIN_BO_DEGRE_AXE1	0.039f // karim ajout a calculer pour l'axe 1

/* affectation des correcteurs */
#define PID	1
#define TW	2
#define STW	3 

/***************************/
/** TOUTES LES INCLUSIONS **/
/***************************/ 
#include "vxWorks.h"
#include "syslib.h"
#include "taskLib.h"
#include "wdLib.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "iostream.h"
#include "kernelLib.h"
#include "msgQLib.h"
#include "tickLib.h"    
#include "semLib.h"
#include "logLib.h"
#include "math.h"

#include "CarteCIODAC16.cpp"
#include "CarteCIODAS64.cpp"

#include "Actionneur.cpp"
#include "CapteurDePosition.cpp"
#include "Correcteur.cpp"
#include "CorrecteurPID.cpp"
#include "CorrecteurTW.cpp"
#include "CorrecteurSTW.cpp"
#include "ControleurDAxe.cpp"


#include "fichier.cpp"
#include "BibliothequeTR\ChienDeGarde.cpp"
#include "BibliothequeTR\Evenement.cpp"
#include "BibliothequeTR\HorlogeSynchronisee.cpp"
#include "BibliothequeTR\Tache.cpp"
#include "BibliothequeTR\Temps.cpp"

// #include "Robot7Axes\ModeleGeometrique2Axes.cpp"	CECI servira plus tard je pense 





/**********************************/
/*     TOUTES LES DEFINITIONS     */
/**********************************/

/* pressions maxi. et mini. numeriques envoyees par le DA vers l'IP */
#define PRESSION_MAX_NUM 	4095
#define PRESSION_MIN_NUM    0 

/* pression correspondant a P0 */
#define PRESSION_BASE 	2.5 //cette pression peut etre changee an accord avec 
														//actionneur.h et actionneur.cpp

/* valeurs des pressions*100 bars correspondant a la position initiale du bras */
#define DELTA_INIT_AXE1  -400 //  -400  // ancienne valeut -200  // -200 //mourade donne // a ete changer par karim le 02/02/09 il y a eu un incident sur l'axe 1
#define DELTA_INIT_AXE2  -230 // old value -230 //en realite ces pressions sont a rajouter
#define DELTA_INIT_AXE3  -035 // a la pression de base pour configurer
#define DELTA_INIT_AXE4  -195  // old value -195 //-180 le bras. par exemple pour l axe2

#define DELTA_INIT_AXE5   035 //035//-015 //dans le muscle m1, pression=2.5+(-2.3)=+0.2 (BLANC M1)
#define DELTA_INIT_AXE6   015 //015 //dans le muscle m2, pression=2.5-(-2.3)=+4.8 (NOIR M2)
#define DELTA_INIT_AXE7  -020    // -020 valeur initial a remettre apres repaaration 

//pressions de saturation.ceci correspond a 2.5-(-1.8)
#define	SATMIN		0.0f	
#define	SATMAX2		4.2f //SATMAX2		4.0f //ulim2 4.5 valeur max
#define SATMAX4		3.5f //ulim4 4.45 valeur max
#define SATMAX1		2.5f //ulim1 3.5 valeur max karim ajout 



#define PressionDeRepos1	0 // ces pressions sont celles auxquelles se
#define PressionDeRepos2	0 //rajoute une commande pour constituer
#define PressionDeRepos3	0 //un delta pression (voir actionneur.cpp)
#define PressionDeRepos4	0
#define PressionDeRepos5	0
#define PressionDeRepos6	0
#define PressionDeRepos7	0


/* affectation des ports de sorties de la carte (CIODAC16) aux actionneurs */
#define VOIE_ACTIONNEUR1_MUSCLE1  2
#define VOIE_ACTIONNEUR1_MUSCLE2  3
#define VOIE_ACTIONNEUR2_MUSCLE1  0
#define VOIE_ACTIONNEUR2_MUSCLE2  1
#define VOIE_ACTIONNEUR3_MUSCLE1  4
#define VOIE_ACTIONNEUR3_MUSCLE2  5
#define VOIE_ACTIONNEUR4_MUSCLE1  6
#define VOIE_ACTIONNEUR4_MUSCLE2  7
#define VOIE_ACTIONNEUR5_MUSCLE1 12
#define VOIE_ACTIONNEUR5_MUSCLE2 13
#define VOIE_ACTIONNEUR6_MUSCLE1  8
#define VOIE_ACTIONNEUR6_MUSCLE2  9
#define VOIE_ACTIONNEUR7_MUSCLE1 10
#define VOIE_ACTIONNEUR7_MUSCLE2 11

/* affectation des ports d'entrees de la carte (COIDAS64) aux capteurs */
#define VOIE_CAPTEUR_AXE1 20
#define VOIE_CAPTEUR_AXE2 18
#define VOIE_CAPTEUR_AXE3 22
#define VOIE_CAPTEUR_AXE4 16
#define VOIE_CAPTEUR_AXE5 17
#define VOIE_CAPTEUR_AXE6 19
#define VOIE_CAPTEUR_AXE7 21

/* Rapports de reduction axe/capteur
	 le signe - indique une inversion de sens de rotation */
#define RAP_MECA_CAP_1    1.5
#define RAP_MECA_CAP_2   -1.0
#define RAP_MECA_CAP_3    1.0
#define RAP_MECA_CAP_4   -1.0
#define RAP_MECA_CAP_5    2.0
#define RAP_MECA_CAP_6    1.0
#define RAP_MECA_CAP_7   -1.0

/* nombre de points a enregistrer */
/***********************************************/
//#define NB_POINTS  1000 //600//300
/***********************************************/
/* pente du capteur de position */
#define PENTE_CAPTEUR       1.2566f   //5volts // 2pi/5.13volts=1.2248radians par volt

/* offset des capteurs releves le jeudi 24/01/08
non utilises car les offsets sont lus a l'initialisation*/

#define OFFSET_CAPTEUR_AXE1	1.60684f	//2.5038f
#define OFFSET_CAPTEUR_AXE2	2.52992f	//2.6724f
#define OFFSET_CAPTEUR_AXE3	2.53724f	//2.5978f
#define OFFSET_CAPTEUR_AXE4	2.20024f	//2.2905f
#define OFFSET_CAPTEUR_AXE5	2.35531f	//2.5490f
#define OFFSET_CAPTEUR_AXE6	0.97924f	//2.6098f
#define OFFSET_CAPTEUR_AXE7	2.47375f	//2.5388f


/* definition de pi */
#define pi	3.14159f

int nbMuscleGonfle;
int NB_POINTS;

/* vecteur positions initiales  */
//double
float	theta_origine[7];


/** periode d'echantillonage en ms **/
#define maPeriode  10 



/******************************************************/

/******************************************************/
/*  InitialiseBras permet d'initialiser le bras       */
/*  i.e le positionner dans une configuration desiree */
/*  Le gonflement s'effectue par paliers de 1e-2 bars */
/******************************************************/
void initialiseMuscle(int muscleAInitialiser,int deltaInitm,int capteurMuscleInitialise)

{

	Actionneur * pMuscleAInitialiser = (Actionneur *) muscleAInitialiser;

	CapteurDePosition * pCapteurMuscleInitialise =(CapteurDePosition*) capteurMuscleInitialise;
  float i;
	float j;
  float deltaInit = (float) deltaInitm / 100.0;
 
	/* affectation des pressions presentes dans les 2 muscles */
  i = pMuscleAInitialiser->pressionMuscle1();	//renvoi la pression de M1
  j = pMuscleAInitialiser->pressionMuscle2();	//renvoi la pression de M2

  while (i < (PRESSION_BASE + deltaInit)|| j < (PRESSION_BASE - deltaInit))
    {
      if (i	<(PRESSION_BASE + deltaInit))		i = i + 0.01;
      else i = PRESSION_BASE + deltaInit;

      if (j	<(PRESSION_BASE - deltaInit))		j = j + 0.01;
      else j = PRESSION_BASE - deltaInit;
			
      pMuscleAInitialiser->envoyerCommandeDecouplee(i,j);
			
      taskDelay(sysClkRateGet()/50);	//tempo.de 20ms apres injection
    } //FIN while

	nbMuscleGonfle ++;
}		//FIN initialiseMuscle


/********************************************************/
/* degonfle permet de degonfler tous les muscles	      */
/*	le degonflement s'effectue par paliers de 5e-2bars  */
/********************************************************/
 
void degonfleMuscle(int muscleADegonfler)

{
  float pm1, pm2;

  Actionneur * pMuscleADegonfler = (Actionneur *) muscleADegonfler;
  
	/* affectation des pressions presentes dans les 2 muscles */
	pm1 = pMuscleADegonfler->pressionMuscle1();	//renvoi la pression du muscle1
	pm2 = pMuscleADegonfler->pressionMuscle2();	//renvoi la pression du muscle2

  while(pm1 > 0.0 || pm2 > 0.0)
  { 
   	if (pm1 > 0.05)	pm1 = (pm1-0.05) ;
    else	pm1 = 0.0;

   	if (pm2 > 0.05)	pm2 = (pm2-0.05) ;
    else  pm2 = 0.0;
  
	pMuscleADegonfler->envoyerCommandeDecouplee(pm1,pm2);	

	taskDelay(sysClkRateGet()/10);	//tempo.de 100ms apres chaque injection
  }//FIN while 
}		//FIN degonfleMuscle




/*************************************************************/
/*************************************************************/


/****************************************************/
/*	echelonPression permet d'envoyer un echelon de	*/
/*	pression a un axe																*/
/****************************************************/

void	echelonAxe	(Actionneur* actionneur,
											CapteurDePosition* capteur,
											int	n_axe,
											float	echelon_pression,
						double	*	releveTemporel,
						double	*	releveThetaMesuree,
						double	*	relevePression)
{
	int	deltaT	=	maPeriode;
	float dt	=	(Duree(deltaT,MS)).toSecondes();
  
  

NB_POINTS = 1000;

float	theta_mesuree	=	0.0;
Horloge monHorlogeEchelon;
monHorlogeEchelon.demarrer(Duree(deltaT,MS));


for(int i=0; i<NB_POINTS; i++)	{
	monHorlogeEchelon.attendre();
	releveTemporel[i]	=	i*dt;
	theta_mesuree	= (float) (capteur->lirePositionRAD());

  // ajout karim ce code permet d'injecter un sinus
  //echelon_pression=2.5*sin(2*pi*(1/0.5)*releveTemporel[i])+2;
  // fin ajout

	releveThetaMesuree[i]	=	theta_mesuree;
	relevePression[i]	=	(float) echelon_pression;


	
	actionneur->envoyerCommande(echelon_pression);

																	}//	FIN i

monHorlogeEchelon.arreter();
}//	FIN echelonAxe (sur 1 axe (2 ou 4))

/*************************************************************/
/* 2 echelons de pression sur les axes 2 et 4 */
/**********************************************/

void	echelonDeuxAxes	(Actionneur* actionneurAxe2,
											Actionneur* actionneurAxe4,
											CapteurDePosition* capteur2,
											CapteurDePosition* capteur4,
											float	echelon_pression2,
											float	echelon_pression4,
												double	*	releveTemporel,
												double	*	relevePression2,
												double	*	relevePression4,
												double	*	releveThetaMesuree2,
												double	*	releveThetaMesuree4)

{
	int	deltaT	=	maPeriode;
	float dt	=	(Duree(deltaT,MS)).toSecondes();

NB_POINTS = 1000;

float	theta_mesuree2	=	0.0;
float	theta_mesuree4	=	0.0;
Horloge monHorlogeEchelon;
monHorlogeEchelon.demarrer(Duree(deltaT,MS));

for(int i=0; i<NB_POINTS; i++)	{
	monHorlogeEchelon.attendre();

	theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
	theta_mesuree4	=	(float) (capteur4->lirePositionRAD());
	
	actionneurAxe2->envoyerCommande(echelon_pression2);
	actionneurAxe4->envoyerCommande(echelon_pression4);

releveTemporel[i]	=	i*dt;
relevePression2[i]	=	echelon_pression2;
relevePression4[i]	=	echelon_pression4;
releveThetaMesuree2[i]	=	theta_mesuree2;
releveThetaMesuree4[i]	=	theta_mesuree4;


																			}//	FIN i

monHorlogeEchelon.arreter();
}//	FIN echelonDeuxAxes (Pressions sur 2 axes (2 et 4))


/*****************************************************************/
/* suiviTW commande les axes 2 et 4 en suivi de trajectoire ******/
/**** avec l'algorithme du twisting          *********************/
/*****************************************************************/

void	suiviTW(ControleurDAxe*	controleurAxe2,
										ControleurDAxe*	controleurAxe4,
										CapteurDePosition*	capteur2,
										CapteurDePosition*	capteur4,
										Actionneur*	actionneur2,
										Actionneur*	actionneur4,
					int NB_POINTS,
					double	*	releveTemporel,
					double	*	relevePression2,
					double	*	relevePression4,
					double	**angle,
					double	**vitang,
					double	**accang,
					double	*	releveThetaDesiree2,
					double	*	releveThetaDesiree4,
					double	*	releveThetaMesuree2,
					double	*	releveThetaMesuree4,
					double	*	releveErreur2,
					double	*	releveErreur4,
					double	*	releveUeq2,
					double	*	releveUeq4,
					double	*	releveUdelta2,
					double	*	releveUdelta4,
					double	*	releveSurFace2,
					double	*	releveSurFace4,
					double	*	releveSurfaceDot2,
					double	*	releveSurfaceDot4)

{
int	T_suivi	=	50;//50ms pour rafraichir la consigne

float	tau	=	(Duree(T_suivi,MS)).toSecondes();
float dt = (Duree(maPeriode,MS)).toSecondes();

float commande2, commande4, commande_precedente2, commande_precedente4;
float theta_desiree2, theta_desiree4, theta_mesuree2, theta_mesuree4;
float  theta_mesuree_precedente2, theta_mesuree_precedente4;
float vitesse_desiree2,vitesse_desiree4, vitesse_mesuree2, vitesse_mesuree4;
float acceleration_desiree2, acceleration_desiree4;

int hybrid;
float erreur2, erreur4, comi2, comi4;
float integr2, integr4, largeur2, largeur4;
float erreur_integ2_preced,erreur_integ4_preced, erreur_integ2, erreur_integ4;

/// initialisation des variables ////
/////////////////////////////////////

commande2 = 0.0;
commande4 = 0.0;

theta_desiree2	= angle[0][0];
theta_desiree4	=	angle[1][0];

vitesse_desiree2	=	vitang[0][0];
vitesse_desiree4	=	vitang[1][0];

acceleration_desiree2 = accang[0][0];
acceleration_desiree4 = accang[1][0];


//////////////////////////////////////////////
/// fin initialisation des variables ////
///////////////////////////////////////////////

/**********pour l'action hybride TW+I*****************/


/*******************************************************/
hybrid=0;
printf("\n pour ajouter une action integrale, entrer 1\n");
scanf("%d",&hybrid);
if(hybrid==1) {
								printf("entrer le gain integral pour l'axe 2\n>");
								scanf("%f",&integr2);
								printf("entrer le gain integral pour l'axe 4\n>");
								scanf("%f",&integr4);
								printf("entrer la valeur absolue de l'erreur (DEG.) de l'axe 2 pour actionner le I\n>");
								scanf("%f",&largeur2);
				largeur2 = largeur2*pi/180.0f;
								printf("entrer la valeur absolue de l'erreur (DEG.) de l'axe 4 pour actionner le I\n>");
								scanf("%f",&largeur4);
				largeur4 = largeur4*pi/180.0f;
							}


/***************************************/

	printf("POUR DEMARRER ENTRER 1\n");
int	validation = 0;
	scanf("%d",&validation);
	while (validation	!=1)	{
				printf("POUR DEMARRER ENTRER 1\n");
				scanf("%d",&validation);	
													}//FIN while
printf("\nOFF WE GO\n");

//asservissement tous les 10ms
controleurAxe2->asservirAxe (Duree(10,MS));
controleurAxe4->asservirAxe (Duree(10,MS));

Horloge monHorlogeEchelon;
monHorlogeEchelon.demarrer(Duree(T_suivi,MS));


controleurAxe2->getCorrecteur()->initialise(theta_desiree2);
controleurAxe2->getCorrecteur()->setSuivi(true);
controleurAxe4->getCorrecteur()->initialise(theta_desiree4);
controleurAxe4->getCorrecteur()->setSuivi(true);


/*

////// positionnement a (posxi,posyi) //////////////////////
for(int i=0;i<300;i++)
{
monHorlogeEchelon.attendre();

commande2	=	controleurAxe2->getCorrecteur()->calculerCommande(theta_desiree2, vitesse_desiree2, acceleration_desiree2);

commande4	=	controleurAxe4->getCorrecteur()->calculerCommande(theta_desiree4, vitesse_desiree4, acceleration_desiree4);

//saturation des commandes
								if(commande2 <= SATMIN)	commande2 = SATMIN;
								if(commande2 >= SATMAX2)	commande2 = SATMAX2;

								if(commande4 <= SATMIN)	commande4 = SATMIN;
								if(commande4 >= SATMAX4)	commande4 = SATMAX4;



actionneur2->envoyerCommande(commande2);
actionneur4->envoyerCommande(commande4);

commande_precedente2 = commande2;
commande_precedente4 = commande4;
}//FIN i<300



printf("\nOK\n");
for (int i=1; i<3;i++)
			taskDelay(sysClkRateGet()); //tempo de 1s*2

*/
theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());

erreur_integ2_preced = 0.0;
erreur_integ2 = 0.0;
erreur_integ4_preced = 0.0;
erreur_integ4 = 0.0;
theta_mesuree_precedente2 = theta_mesuree2;
theta_mesuree_precedente4 = theta_mesuree4;

//////////////////////////////////////////////////////

for(int i=0; i<NB_POINTS; i++)	{
monHorlogeEchelon.attendre();

/******************************************************/
theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());

vitesse_mesuree2	=	(theta_mesuree2	-	theta_mesuree_precedente2)/tau;
vitesse_mesuree4	=	(theta_mesuree4	-	theta_mesuree_precedente4)/tau;

theta_mesuree_precedente2 = theta_mesuree2;
theta_mesuree_precedente4 = theta_mesuree4;
/*******************************************/

					theta_desiree2 = angle[0][i];
					theta_desiree4 = angle[1][i];
					vitesse_desiree2 = vitang[0][i];
					vitesse_desiree4 = vitang[1][i];
					acceleration_desiree2 = accang[0][i];
					acceleration_desiree4 = accang[1][i];				

	erreur2 = theta_desiree2 - theta_mesuree2;
	erreur4 = theta_desiree4 - theta_mesuree4;


comi2 = 0.0f; comi4 = 0.0f;
//pid  //

	if (hybrid==1)	
					{
										if ( fabs(erreur2) <= largeur2 )
												 {

							erreur_integ2 = erreur_integ2_preced + (erreur2 * dt);

							comi2 =(integr2 * erreur_integ2); 
									erreur_integ2_preced = erreur_integ2;

													} //fin I axe2 sans saturation


										if ( fabs(erreur4) <= largeur4 ) 
													{

							erreur_integ4 = erreur_integ4_preced + (erreur4 * dt);

							comi4 =(integr4 * erreur_integ4); 
									erreur_integ4_preced = erreur_integ4;
														}

					}//FIN hybrid


//calcul des commandes
commande2	=	controleurAxe2->getCorrecteur()->calculerCommande(theta_desiree2, vitesse_desiree2, acceleration_desiree2);

commande4	=	controleurAxe4->getCorrecteur()->calculerCommande(theta_desiree4, vitesse_desiree4, acceleration_desiree4);

// on rajoute  l'action I //
//l'acceleration de convergence
// et l'attenuation de ueq est dans le correcteur


commande2 = commande2 + comi2;
commande4 = commande4 + comi4;

//saturation des commandes
								if(commande2 <= SATMIN)	commande2 = SATMIN;
								if(commande2 >= SATMAX2)	commande2 = SATMAX2;

								if(commande4 <= SATMIN)	commande4 = SATMIN;
								if(commande4 >= SATMAX4)	commande4 = SATMAX4;

if(i==(NB_POINTS/2)-1) {
										commande2 = commande_precedente2;
										commande4 = commande_precedente4;
									}//FIN if

actionneur2->envoyerCommande(commande2);
actionneur4->envoyerCommande(commande4);

commande_precedente2 = commande2;
commande_precedente4 = commande4;


//releve des donnees
releveTemporel[i]	=	i*tau;//50ms
relevePression2[i]	=	commande2;
relevePression4[i]	=	commande4;
releveUeq2[i]	=	controleurAxe2->getCorrecteur()->getUeq();
releveUeq4[i]	=	controleurAxe4->getCorrecteur()->getUeq();
releveUdelta2[i]	=	controleurAxe2->getCorrecteur()->getUdelta();
releveUdelta4[i]	=	controleurAxe4->getCorrecteur()->getUdelta();


releveThetaDesiree2[i]	=	angle[0][i];
releveThetaDesiree4[i]	=	angle[1][i];

releveThetaMesuree2[i]	=	theta_mesuree2;
releveThetaMesuree4[i]	=	theta_mesuree4;


releveErreur2[i]	=	controleurAxe2->getCorrecteur()->getErreur();
releveErreur4[i]	=	controleurAxe4->getCorrecteur()->getErreur();

releveSurFace2[i]	=	controleurAxe2->getCorrecteur()->getSurface();
releveSurFace4[i]	=	controleurAxe4->getCorrecteur()->getSurface();
releveSurfaceDot2[i]	=	controleurAxe2->getCorrecteur()->getSurfacePoint();
releveSurfaceDot4[i]	=	controleurAxe4->getCorrecteur()->getSurfacePoint();


//affichage des erreurs en degres
printf("\nerreur2=%f\terreur4=%f",releveErreur2[i]*180.0/pi,releveErreur4[i]*180.0f/pi);
}//FIN boucle de i

controleurAxe2->arreterAsservissement();
controleurAxe4->arreterAsservissement();
monHorlogeEchelon.arreter();

//complement du tableau par des zeros pour matlab
for(int i=NB_POINTS; i<1000; i++)	{
releveTemporel[i]	=	i*tau;//50ms
relevePression2[i]	=0.0;
relevePression4[i]	=	0.0;
releveUeq2[i]	=	0.0;
releveUeq4[i]	=	0.0;
releveUdelta2[i]	=	0.0;
releveUdelta4[i]	=	0.0;


releveThetaDesiree2[i]	=	0.0;
releveThetaDesiree4[i]	=	0.0;

releveThetaMesuree2[i]	=	0.0;
releveThetaMesuree4[i]	=	0.0;


releveErreur2[i]	=	0.0;
releveErreur4[i]	=	0.0;

releveSurFace2[i]	=	0.0;
releveSurFace4[i]	=	0.0;
releveSurfaceDot2[i]	=	0.0;
releveSurfaceDot4[i]	=	0.0;}

} //FIN suiviTW



/*****************************************************************/
/* sinusTW permet de commander les axes 2 et 4 par un Twist   ****/
/* pour suivre des sinusoides etablies par la routine sinusoide  */
/*****************************************************************/
/********************************************************/
/********************************************************/
/*	sinusTW applique l'alogorithme du TWISTING pour un  */
/*	suivi de sinususoides par les axes	2 et 4  				*/
/********************************************************/
/********************************************************/
void	sinusTW(ControleurDAxe*	controleurAxe2,
										ControleurDAxe*	controleurAxe4,
										CapteurDePosition*	capteur2,
										CapteurDePosition*	capteur4,
										Actionneur*	actionneur2,
										Actionneur*	actionneur4,
						float amplitude2,
						float pDepart2,
						float omega2,
						float amplitude4,
						float pDepart4,
						float omega4,
						float dephase2,
						float dephase4,
					double	*	releveTemporel,
					double	*	relevePression2,
					double	*	relevePression4,
					double	*	releveThetaDesiree2,
					double	*	releveThetaDesiree4,
					double	*	releveThetaMesuree2,
					double	*	releveThetaMesuree4,
					double	*	releveErreur2,
					double	*	releveErreur4,
					double	*	releveUeq2,
					double	*	releveUeq4,
					double	*	releveUdelta2,
					double	*	releveUdelta4,
					double	*	releveSurFace2,
					double	*	releveSurFace4,
					double	*	releveSurfaceDot2,
					double	*	releveSurfaceDot4)

{

printf("\n********************\n");
printf("\nMODE SUIVI SINUS TW");
printf("\n********************\n");

NB_POINTS = 1000;

int	T_suivi	=	50;//50ms pour rafraichir la consigne

float	tau	=	(Duree(T_suivi,MS)).toSecondes();

float	commande2, commande_precedente2;
float	commande4, commande_precedente4;
float theta_desiree_precedente2, theta_desiree_precedente4;
float theta_mesuree2, theta_mesuree4;
float vitesse_desiree_precedente2, vitesse_desiree_precedente4;
float theta_desiree2, theta_desiree4, vitesse_desiree2, vitesse_desiree4;
float acceleration_desiree2, acceleration_desiree4;
int validation;
//**************************ajout karim 22 oct******************************************
int hybrid; 
float integr2,integr4;
float largeur2, largeur4;

float erreur2, erreur4; 
float erreur_integ2, erreur_integ4; 
float erreur_integ2_preced, erreur_integ4_preced;
float comi2, comi4; 

erreur_integ2_preced = 0.0f;
erreur_integ4_preced = 0.0f;
erreur_integ2 = 0.0f;
erreur_integ4 = 0.0f;


//**************************************************************


//asservissement tous les 10ms
controleurAxe2->asservirAxe (Duree(10,MS));
controleurAxe4->asservirAxe (Duree(10,MS));

Horloge monHorlogeEchelon;
monHorlogeEchelon.demarrer(Duree(T_suivi,MS));

/// initialisation des variables ////
commande2 = 0.0;
commande4 = 0.0;
commande_precedente2 = 0.0;
commande_precedente4 = 0.0;

theta_desiree2	= pDepart2;
theta_desiree4	=	pDepart4;
theta_desiree_precedente2 = theta_desiree2;
theta_desiree_precedente4 = theta_desiree4;

vitesse_desiree2	=	0.0;
vitesse_desiree4	=	0.0;
vitesse_desiree_precedente2	=	0.0;
vitesse_desiree_precedente4	=	0.0;

acceleration_desiree2	=	0.0;
acceleration_desiree4	=	0.0;

theta_mesuree2	=	0.0;
theta_mesuree4	=	0.0;


/// fin initialisation des variables ////
///////////////////////////////////////////////

validation = 0;
while(validation!=1) {
theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());
printf("\nANGLE 2 = %f\n",theta_mesuree2*180.0f/pi);
printf("\nANGLE 4 = %f\n",theta_mesuree4*180.0f/pi);

printf("\nvalider par 1\n");
scanf("%d",&validation);
											}//FIN validation


controleurAxe2->getCorrecteur()->initialise(theta_desiree2);
controleurAxe2->getCorrecteur()->setSuivi(true);
controleurAxe4->getCorrecteur()->initialise(theta_desiree4);
controleurAxe4->getCorrecteur()->setSuivi(true);




/******************** ajout kb 22 oct 09***********************************/

validation = 0;
while(validation!=1) {
printf("\n********************\n");
printf("\nAJOUT INTEGRATEUR");
printf("\n********************\n");

hybrid=0;
printf("\n pour ajouter une action integrale, entrer 1\n");
scanf("%d",&hybrid);
if(hybrid==1) {
								printf("entrer le gain integral pour l'axe 2\n>");
								scanf("%f",&integr2);
								printf("entrer le gain integral pour l'axe 4\n>");
								scanf("%f",&integr4);
								printf("entrer la valeur absolue de l'erreur (DEG.) de l'axe 2 pour actionner le I\n>");
								scanf("%f",&largeur2);
				largeur2 = largeur2*pi/180.0f;
								printf("entrer la valeur absolue de l'erreur (DEG.) de l'axe 4 pour actionner le I\n>");
								scanf("%f",&largeur4);
				largeur4 = largeur4*pi/180.0f;}


printf("\nvalider par 1\n");
scanf("%d",&validation);
											}//FIN validation




/***************************************/






for(int i=0; i<NB_POINTS; i++)	{
monHorlogeEchelon.attendre();

theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());

theta_desiree2	=	(amplitude2 * sin((omega2 * i * tau) + dephase2)) + pDepart2;
theta_desiree4	=	(amplitude4 * sin((omega4 * i * tau) + dephase4)) + pDepart4;

vitesse_desiree2	=	(theta_desiree2	-	theta_desiree_precedente2)/tau;
vitesse_desiree4	=	(theta_desiree4	-	theta_desiree_precedente4)/tau;
acceleration_desiree2	=	(vitesse_desiree2	-	vitesse_desiree_precedente2)/tau;
acceleration_desiree4	=	(vitesse_desiree4	-	vitesse_desiree_precedente4)/tau;


//re actualisation des variables
theta_desiree_precedente2 = theta_desiree2;
theta_desiree_precedente4 = theta_desiree4;
vitesse_desiree_precedente2	=	vitesse_desiree2;
vitesse_desiree_precedente4	=	vitesse_desiree4;

//calcul des commandes
commande2	=	controleurAxe2->getCorrecteur()->calculerCommande(theta_desiree2, vitesse_desiree2, acceleration_desiree2);
commande4	=	controleurAxe4->getCorrecteur()->calculerCommande(theta_desiree4, vitesse_desiree4, acceleration_desiree4);

//saturation des commandes
								if(commande2 <= SATMIN)	commande2 = SATMIN;
								if(commande2 >= SATMAX2)	commande2 = SATMAX2;

								if(commande4 <= SATMIN)	commande4 = SATMIN;
								if(commande4 >= SATMAX4)	commande4 = SATMAX4;


if(i==(NB_POINTS/2)-1) {
										commande2 = commande_precedente2;
										commande4 = commande_precedente4;
									}//FIN if


// ajout karim integrateur+tw le 22 oct 2009 ********************************************


erreur2 = theta_desiree2 - theta_mesuree2;
erreur4 = theta_desiree4 - theta_mesuree4;


comi2 = 0.0f; comi4 = 0.0f;




	if (hybrid==1)	
					{
										if ( fabs(erreur2) <= largeur2 )
												 {

							erreur_integ2 = erreur_integ2_preced + (erreur2 * tau);

							comi2 =(integr2 * erreur_integ2); 
									erreur_integ2_preced = erreur_integ2;

													} //fin I axe2 sans saturation


										if ( fabs(erreur4) <= largeur4 ) 
													{

							erreur_integ4 = erreur_integ4_preced + (erreur4 * tau);

							comi4 =(integr4 * erreur_integ4); 
									erreur_integ4_preced = erreur_integ4;
														}

					}//FIN hybrid




commande2=commande2+comi2;
commande4=commande4+comi4;

//fin ajout ******************************************************************************



// filtrage commande karim le 05/12/2008***************************************************




//if (i>=6)
//{

//commande4= (9.87e-2*commande4)+(17.04e-2*relevePression4[i-1])+(22.93e-2*relevePression4[i-2])+(22.93e-2*relevePression4[i-3])+(17.04e-2*relevePression4[i-4])+(9.87e-2*relevePression4[i-5]);


//}




// fin filtrage commande **********************************************************************








actionneur2->envoyerCommande(commande2);
actionneur4->envoyerCommande(commande4);

commande_precedente2 = commande2;
commande_precedente4 = commande4;

//releve des donnees
releveTemporel[i]	=	i* tau;

releveUdelta2[i]	=	controleurAxe2->getCorrecteur()->getUdelta();
releveUdelta4[i]	=	controleurAxe4->getCorrecteur()->getUdelta();
relevePression2[i]	=	commande2;
relevePression4[i]	=	commande4;
releveUeq2[i]	=	controleurAxe2->getCorrecteur()->getUeq();
releveUeq4[i]	=	controleurAxe4->getCorrecteur()->getUeq();

releveThetaDesiree2[i]	=	theta_desiree2;
releveThetaDesiree4[i]	=	theta_desiree4;
releveThetaMesuree2[i]	=	theta_mesuree2;
releveThetaMesuree4[i]	=	theta_mesuree4;


releveErreur2[i]	=	controleurAxe2->getCorrecteur()->getErreur();
releveErreur4[i]	=	controleurAxe4->getCorrecteur()->getErreur();
releveSurFace2[i]	=	controleurAxe2->getCorrecteur()->getSurface();
releveSurFace4[i]	=	controleurAxe4->getCorrecteur()->getSurface();
releveSurfaceDot2[i]	=	controleurAxe2->getCorrecteur()->getSurfacePoint();
releveSurfaceDot4[i]	=	controleurAxe4->getCorrecteur()->getSurfacePoint();


//affichage des erreurs en degres
printf("\nerreur2=%f\terreur4=%f",releveErreur2[i]*180.0/pi,releveErreur4[i]*180.0f/pi);

											}	//FIN i

controleurAxe2->arreterAsservissement();
controleurAxe4->arreterAsservissement();
monHorlogeEchelon.arreter();
}	//FIN sinusTW

/*********************************************************************/
/*****************************************************************/

void	suiviPID(ControleurDAxe*	controleurAxe2,
										ControleurDAxe*	controleurAxe4,
										CapteurDePosition*	capteur2,
										CapteurDePosition*	capteur4,
										Actionneur*	actionneur2,
										Actionneur*	actionneur4,
					int NB_POINTS,
					double	**angle,
					double	**vitang,
					double	**accang,
					double	*	releveTemporel,
					double	*	relevePression2,
					double	*	relevePression4,
					double	*	releveThetaDesiree2,
					double	*	releveThetaDesiree4,
					double	*	releveThetaMesuree2,
					double	*	releveThetaMesuree4,
					double	*	releveErreur2,
					double	*	releveErreur4)


{
int	T_suivi	=	50;//50ms pour rafraichir la consigne

float	tau	=	(Duree(T_suivi,MS)).toSecondes();


float commande2, commande4, commande_precedente2, commande_precedente4;
float theta_desiree2, theta_desiree4, theta_mesuree2, theta_mesuree4;
float vitesse_desiree2, vitesse_desiree4, acceleration_desiree2, acceleration_desiree4, d_theta_mesuree2_ancienne;



//asservissement tous les 10ms
controleurAxe2->asservirAxe (Duree(10,MS));
controleurAxe4->asservirAxe (Duree(10,MS));

Horloge monHorlogeEchelon;
monHorlogeEchelon.demarrer(Duree(T_suivi,MS));


/// initialisation des variables ////
/////////////////////////////////////

commande2 = 0.0;
commande4 = 0.0;

theta_desiree2	= angle[0][0];
theta_desiree4	=	angle[1][0];

vitesse_desiree2	=	vitang[0][0];
vitesse_desiree4	=	vitang[1][0];

acceleration_desiree2 = accang[0][0];
acceleration_desiree4 = accang[1][0];

theta_mesuree2	=	0.0;
theta_mesuree4	=	0.0;

//////////////////////////////////////////////
/// fin initialisation des variables ////

///////////////////////////////////////////////
printf("\nOFF WE GO!\n");

controleurAxe2->getCorrecteur()->initialise(theta_desiree2);
controleurAxe2->getCorrecteur()->setSuivi(true);
controleurAxe4->getCorrecteur()->initialise(theta_desiree4);
controleurAxe4->getCorrecteur()->setSuivi(true);


////// positionnement a (posxi,posyi) //////////////////////
for(int i=0;i<300;i++)
{
monHorlogeEchelon.attendre();

commande2	=	controleurAxe2->getCorrecteur()->calculerCommande(theta_desiree2, vitesse_desiree2, acceleration_desiree2);

commande4	=	controleurAxe4->getCorrecteur()->calculerCommande(theta_desiree4, vitesse_desiree4, acceleration_desiree4);

//saturation des commandes
								if(commande2 <= SATMIN)	commande2 = SATMIN;
								if(commande2 >= SATMAX2)	commande2 = SATMAX2;

								if(commande4 <= SATMIN)	commande4 = SATMIN;
								if(commande4 >= SATMAX4)	commande4 = SATMAX4;

actionneur2->envoyerCommande(commande2);
actionneur4->envoyerCommande(commande4);

commande_precedente2 = commande2;
commande_precedente4 = commande4;
}//FIN i<300



for(int i=1; i<NB_POINTS; i++)	{
monHorlogeEchelon.attendre();

//////////////////////////////////////////////////////
theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());


/*******************/


//calcul des commandes
commande2	=	controleurAxe2->getCorrecteur()->calculerCommande(angle[0][i], vitang[0][i], accang[0][i]);
commande4	=	controleurAxe4->getCorrecteur()->calculerCommande(angle[1][i], vitang[1][i], accang[1][i]);

//saturation des commandes
								if(commande2 <= SATMIN)	commande2 = SATMIN;
								if(commande2 >= SATMAX2)	commande2 = SATMAX2;

								if(commande4 <= SATMIN)	commande4 = SATMIN;
								if(commande4 >= SATMAX4)	commande4 = SATMAX4;



actionneur2->envoyerCommande(commande2);
actionneur4->envoyerCommande(commande4);

commande_precedente2 = commande2;
commande_precedente4 = commande4;

//releve des donnees
releveTemporel[i]	=	i*tau; //50ms

relevePression2[i]	=	commande2;
relevePression4[i]	=	commande4;

releveThetaDesiree2[i]	=	theta_desiree2;
releveThetaDesiree4[i]	=	theta_desiree4;
releveThetaMesuree2[i]	=	theta_mesuree2;
releveThetaMesuree4[i]	=	theta_mesuree4;


releveErreur2[i]	=	controleurAxe2->getCorrecteur()->getErreur();
releveErreur4[i]	=	controleurAxe4->getCorrecteur()->getErreur();


//affichage des erreurs en degres
printf("\nerreur2=%f\terreur4=%f",releveErreur2[i]*180.0/pi,releveErreur4[i]*180.0f/pi);

														}	//FIN i

controleurAxe2->arreterAsservissement();
controleurAxe4->arreterAsservissement();
monHorlogeEchelon.arreter();

for(int i=NB_POINTS; i<1000; i++)	{
releveTemporel[i]	=	i*tau; //50ms

relevePression2[i]	=	0.0;
relevePression4[i]	=	0.0;

releveThetaDesiree2[i]	=	0.0;
releveThetaDesiree4[i]	=	0.0;
releveThetaMesuree2[i]	=	0.0;
releveThetaMesuree4[i]	=	0.0;


releveErreur2[i]	=	0.0;
releveErreur4[i]	=	0.0;}

} //FIN suiviPID


/******************************************/

/*************PID	PID	PID	PID	PID	PID	*****/

/**********************************************************/
/*	regulationPID permet de commander les axes 2 et 4			*/
/*	en stabilisation (regulation) par un PID							*/
/**********************************************************/

void	regulationPID	(ControleurDAxe*	controleurAxe2,
											ControleurDAxe*	controleurAxe4,
											CapteurDePosition*	capteur2,
											CapteurDePosition*	capteur4,
											Actionneur*	actionneur2,
											Actionneur*	actionneur4,
											float	theta_desiree2,
											float	theta_desiree4,
						double	*	releveTemporel,
						double	*	relevePression2,
						double	*	relevePression4,
						double	*	releveThetaDesiree2,
						double	*	releveThetaDesiree4,
						double	*	releveThetaMesuree2,
						double	*	releveThetaMesuree4,
						double	*	releveErreur2,
						double	*	releveErreur4)

{
	int	deltaT	=	maPeriode;
float dt	=	(Duree(deltaT,MS)).toSecondes();



float theta_mesuree2, theta_mesuree4;
float erreur2, erreur4;
float commande2, commande4, i_alpha_2, i_pid_2, i_alpha_2_att, i_alpha_4, i_pid_4, i_alpha_4_att;

NB_POINTS = 1000;

double * d_theta_mesuree2	= new (double)[NB_POINTS];//derivee de theta 2
double * dd_theta_mesuree2	= new (double)[NB_POINTS];//2 eme derivee de theta 2

double * d_theta_mesuree4	= new (double)[NB_POINTS];//derivee de theta 4
double * dd_theta_mesuree4	= new (double)[NB_POINTS];//2 eme derivee de theta 4




Horloge monHorlogeEchelon;
monHorlogeEchelon.demarrer(Duree(deltaT,MS));

controleurAxe2->getCorrecteur()->initialise(theta_desiree2);
controleurAxe2->getCorrecteur()->setSuivi(false);

controleurAxe4->getCorrecteur()->initialise(theta_desiree4);
controleurAxe4->getCorrecteur()->setSuivi(false);

/*******************************************************************/


printf("\nLa valeur actuelle du gain alpha 2 est:%f",i_alpha_2);
printf("\nEntrer la valeur de alpha 2 I\n>");
scanf("%f",&i_alpha_2);

printf("\nLa valeur actuelle de l'attenuateur de I PID 2 est:%f",i_alpha_2_att);
printf("\nEntrer la valeur de l'attenuateur de I PID 2 I\n>");
scanf("%f",&i_alpha_2_att);


printf("\nLa valeur actuelle du gain alpha 4 est:%f",i_alpha_4);
printf("\nEntrer la valeur de alpha 4 I\n>");
scanf("%f",&i_alpha_4);

printf("\nLa valeur actuelle de l'attenuateur de I PID 4 est:%f",i_alpha_4_att);
printf("\nEntrer la valeur de l'attenuateur de I PID 4 I\n>");
scanf("%f",&i_alpha_4_att);




/*********************************************************************/



for(int i=0; i<NB_POINTS; i++) {
	monHorlogeEchelon.attendre();


theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());

/*if (i<6)
{
theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());
}

if (i>=6)
{

theta_mesuree2	=	(float) (capteur2->lirePositionRAD2((float) (capteur2->lirePositionRAD()),releveThetaMesuree2[i-1],releveThetaMesuree2[i-2],releveThetaMesuree2[i-3],releveThetaMesuree2[i-4],releveThetaMesuree2[i-5]));
theta_mesuree4	=	(float) (capteur4->lirePositionRAD2((float) (capteur4->lirePositionRAD()),releveThetaMesuree4[i-1],releveThetaMesuree4[i-2],releveThetaMesuree4[i-3],releveThetaMesuree4[i-4],releveThetaMesuree4[i-5]));
}*/



erreur2	=	theta_desiree2 	-	theta_mesuree2;
erreur4	=	theta_desiree4 	-	theta_mesuree4;


//*********************************************filtre commande***************************************





commande2	=	controleurAxe2->getCorrecteur()->calculerCommande(theta_desiree2,0.0f,0.0f);
commande4	=	controleurAxe4->getCorrecteur()->calculerCommande(theta_desiree4,0.0f,0.0f);




//*********************************************fin filtre commande***************************************



if (i==0)
{
d_theta_mesuree2[i] = 0;
}

if (i>=1)
{
d_theta_mesuree2[i] = ( theta_mesuree2 - releveThetaMesuree2[i-1])/dt;
}


if (i>=5)
{

d_theta_mesuree2[i]	=(0.0987*d_theta_mesuree2[i])+(0.1704*d_theta_mesuree2[i-1])+(0.2293*d_theta_mesuree2[i-2])+(0.2293*d_theta_mesuree2[i-3])+(0.1704*d_theta_mesuree2[i-4])+(0.0987*d_theta_mesuree2[i-5]);
}


if (i==0)
{
dd_theta_mesuree2[i] = 0;
}

if (i>=1)
{
dd_theta_mesuree2[i] = ( d_theta_mesuree2[i] - d_theta_mesuree2[i-1])/dt;
}


if (i>=5)
{
dd_theta_mesuree2[i]	=(9.87e-2*dd_theta_mesuree2[i])+(17.04e-2*dd_theta_mesuree2[i-1])+(22.93e-2*dd_theta_mesuree2[i-2])+(22.93e-2*dd_theta_mesuree2[i-3])+(17.04e-2*dd_theta_mesuree2[i-4])+(9.87e-2*dd_theta_mesuree2[i-5]);
}


if (i>=1)
{

i_pid_2=dd_theta_mesuree2[i]-i_alpha_2*relevePression2[i-1]; // calcul F=dd y - alpha u

commande2=commande2-i_alpha_2_att*(i_pid_2/i_alpha_2);  // u = PID+ i pid
}



/***************************************************************************************************************/


if (i==0)
{
d_theta_mesuree4[i] = 0;
}

if (i>=1)
{
d_theta_mesuree4[i] = ( theta_mesuree4 - releveThetaMesuree4[i-1])/dt;
}


if (i>=5)
{

d_theta_mesuree4[i]	=(0.0987*d_theta_mesuree4[i])+(0.1704*d_theta_mesuree4[i-1])+(0.2293*d_theta_mesuree4[i-2])+(0.2293*d_theta_mesuree4[i-3])+(0.1704*d_theta_mesuree4[i-4])+(0.0987*d_theta_mesuree4[i-5]);
}


if (i==0)
{
dd_theta_mesuree4[i] = 0;
}

if (i>=1)
{
dd_theta_mesuree4[i] = ( d_theta_mesuree4[i] -d_theta_mesuree4[i-1])/dt;
}


if (i>=5)
{

dd_theta_mesuree4[i]	=(9.87e-2*dd_theta_mesuree4[i])+(17.04e-2*dd_theta_mesuree4[i-1])+(22.93e-2*dd_theta_mesuree4[i-2])+(22.93e-2*dd_theta_mesuree4[i-3])+(17.04e-2*dd_theta_mesuree4[i-4])+(9.87e-2*dd_theta_mesuree4[i-5]);
}


if (i>=5)
{

i_pid_4=dd_theta_mesuree4[i]-i_alpha_4*relevePression4[i-1]; // calcul F=dd y - alpha u

commande4=commande4-i_alpha_4_att*(i_pid_4/i_alpha_4);  // u = PID+ i pid
}


/***************************************************************************************************************************************/


//saturation
								if(commande2 <= SATMIN)	commande2 = SATMIN;
								if(commande2 >= SATMAX2)	commande2 = SATMAX2;

								if(commande4 <= SATMIN)	commande4 = SATMIN;
								if(commande4 >= SATMAX4)	commande4 = SATMAX4;

actionneur2->envoyerCommande(commande2);
actionneur4->envoyerCommande(commande4);

releveTemporel[i]	=	i*dt;
relevePression2[i]	=	commande2;
relevePression4[i]	= commande4;
releveThetaDesiree2[i]	= i_pid_2; //theta_desiree2;
releveThetaDesiree4[i]	= i_pid_4; //theta_desiree4;
releveThetaMesuree2[i]	=	theta_mesuree2;
releveThetaMesuree4[i]	=	theta_mesuree4;
releveErreur2[i]	=	erreur2;
releveErreur4[i]	=	erreur4;

//printf("\nERREURS:\n");
printf("erreur 2=	%f\t erreur 4 = %f\n",erreur2*180.0/pi,erreur4*180.0/pi);
						
								}//FIN	i

monHorlogeEchelon.arreter();

}	//FIN regulationPID



/**********************FIN PID	FIN PID	FIN PID	FIN PID***/


/***TW	TW	TW	TW	TW	****/

/****************************************************/
/*	regulationTW permet de reguler les axes 2 et 4	*/
/*	par l'utilisation d'un TWistseul ou un twist+pid*/
/****************************************************/

void	regulationTW	(ControleurDAxe*	controleurAxe2,
										ControleurDAxe*	controleurAxe4,
										CapteurDePosition*	capteur2,
										CapteurDePosition*	capteur4,
										Actionneur*	actionneur2,
										Actionneur*	actionneur4,
										float	theta_desiree2,
										float	theta_desiree4,
					double	*	releveTemporel,
					double	*	relevePression2,
					double	*	relevePression4,
					double	*	releveThetaDesiree2,
					double	*	releveThetaDesiree4,
					double	*	releveThetaMesuree2,
					double	*	releveThetaMesuree4,
					double	*	releveErreur2,
					double	*	releveErreur4,
					double	*	releveUeq2,
					double	*	releveUeq4,
					double	*	releveUdelta2,
					double	*	releveUdelta4,
					double	*	releveSurFace2,
					double	*	releveSurFace4,
					double	*	releveSurfaceDot2,
					double	*	releveSurfaceDot4)

{


int	deltaT	=	maPeriode;
float	dt	=	(Duree(deltaT,MS)).toSecondes();

int hybrid;
float integr2,propor2,deriv2, integr4, propor4, deriv4;
float theta_mesuree2, theta_mesuree4;
float erreur2, erreur4, comi2, comi4;
float commande2, commande4, commande_precedente2, commande_precedente4;

float erreur_integ2, erreur_integ4, erreur_integ2_preced, erreur_integ4_preced;
float largeur2, largeur4, erreur_prec2, erreur_prec4, erreur_deriv2, erreur_deriv4;
float Kueq4,Kcmd4,Kueq2,Kcmd2,hh; // karim ajout 28 oct 08

float Q_11, Q_12, Q_21, Q_22; // karim ajout 5 jan 11
float K1_11, K1_12, K2_11, K2_12; // karim ajout 5 jan 11
double * d_theta_mesuree4	= new (double)[NB_POINTS];//derivee de theta 2
float z1,z2,d_theta_mesuree4_temp,commande4_puma;

float jeton;

float pcof[]={-0.006, 0.002, 0.007, 0.014, 0.019, 0.020, 0.015, 0.003, -0.012, -0.028, -0.036, -0.032, -0.010, 0.026, 0.075, 0.125, 0.166, 0.190, 0.190, 0.166, 0.125, 0.075, 0.026, -0.010, -0.032, -0.036, -0.028, -0.012, 0.003, 0.015, 0.020, 0.019, 0.014, 0.007, 0.002, -0.006};


float acc_ueq,acc_ueq4,att2,att4;
int _cmpt,_pp;

jeton=0.0f;

	printf("entrer la valeur de att2\n>");
	scanf("%f",&att2);

  printf("entrer la valeur de att4\n>");
	scanf("%f",&att4);


//--------------------------------PUMA--------------------------------------------

K1_11=77.7319f;
K1_12=16.8482f;

K2_11=99.0438f;
K2_12=13.5695f;

Q_11=-387.3464f;
Q_12=-29.8850f;
Q_21=Q_12;
Q_22=-0.0185f;

//--------------------------------PUMA--------------------------------------------


NB_POINTS = 1000;


//initialisation des erreurs du I

erreur_integ2_preced = 0.0f;
erreur_integ4_preced = 0.0f;
erreur_integ2 = 0.0f;
erreur_integ4 = 0.0f;



/**********pour l'action hybride TW+I*****************/


/*******************************************************/
hybrid=0;
printf("\n pour ajouter une action integrale, entrer 1\n");
scanf("%d",&hybrid);
if(hybrid==1) {
								printf("entrer le gain integral pour l'axe 2\n>");
								scanf("%f",&integr2);
								printf("entrer le gain integral pour l'axe 4\n>");
								scanf("%f",&integr4);
								printf("entrer la valeur absolue de l'erreur (DEG.) de l'axe 2 pour actionner le I\n>");
								scanf("%f",&largeur2);
				largeur2 = largeur2*pi/180.0f;
								printf("entrer la valeur absolue de l'erreur (DEG.) de l'axe 4 pour actionner le I\n>");
								scanf("%f",&largeur4);
				largeur4 = largeur4*pi/180.0f;
							}


/***************************************/

Horloge monHorlogeEchelon;
monHorlogeEchelon.demarrer(Duree(deltaT,MS));

controleurAxe2->getCorrecteur()->initialise(theta_desiree2);
controleurAxe2->getCorrecteur()->setSuivi(false);
controleurAxe4->getCorrecteur()->initialise(theta_desiree4);
controleurAxe4->getCorrecteur()->setSuivi(false);

for(int i=0; i<NB_POINTS; i++)	{
monHorlogeEchelon.attendre();



if (i<6)
{
theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());
}

if (i>=6)
{

theta_mesuree2	=	(float) (capteur2->lirePositionRAD2((float) (capteur2->lirePositionRAD()),releveThetaMesuree2[i-1],releveThetaMesuree2[i-2],releveThetaMesuree2[i-3],releveThetaMesuree2[i-4],releveThetaMesuree2[i-5]));
theta_mesuree4	=	(float) (capteur4->lirePositionRAD2((float) (capteur4->lirePositionRAD()),releveThetaMesuree4[i-1],releveThetaMesuree4[i-2],releveThetaMesuree4[i-3],releveThetaMesuree4[i-4],releveThetaMesuree4[i-5]));
}



//calcul des erreurs pour action pid //
	erreur2 = theta_desiree2 - theta_mesuree2;
	erreur4 = theta_desiree4 - theta_mesuree4;


comi2 = 0.0f; comi4 = 0.0f;
//pid axe2 //

	if (hybrid==1)	
					{
										if ( fabs(erreur2) <= largeur2 )
												 {

							erreur_integ2 = erreur_integ2_preced + (erreur2 * dt);

							comi2 =(integr2 * erreur_integ2); 
									erreur_integ2_preced = erreur_integ2;

													} //fin I axe2 sans saturation


										if ( fabs(erreur4) <= largeur4 ) 
													{

							erreur_integ4 = erreur_integ4_preced + (erreur4 * dt);

							comi4 =(integr4 * erreur_integ4); 
									erreur_integ4_preced = erreur_integ4;
														}

					}//FIN hybrid








commande2	=	controleurAxe2->getCorrecteur()->calculerCommande(theta_desiree2, 0.0f, 0.0f);
commande4	=	controleurAxe4->getCorrecteur()->calculerCommande(theta_desiree4, 0.0f, 0.0f);











//******************************filtre de la commande karim 27 oct 08*********************************************************

/*

if (i>=6)
{

Kueq4=controleurAxe4->getCorrecteur()->getUeq();
Kcmd4= (9.87e-2*Kueq4)+(17.04e-2*releveUeq4[i-1])+(22.93e-2*releveUeq4[i-2])+(22.93e-2*releveUeq4[i-3])+(17.04e-2*releveUeq4[i-4])+(9.87e-2*releveUeq4[i-5]);
commande4=Kcmd4+(controleurAxe4->getCorrecteur()->getUdelta());

//Kueq2=controleurAxe2->getCorrecteur()->getUeq();
//Kcmd2= (9.87e-2*Kueq2)+(17.04e-2*releveUeq2[i-1])+(22.93e-2*releveUeq2[i-2])+(22.93e-2*releveUeq2[i-3])+(17.04e-2*releveUeq2[i-4])+(9.87e-2*releveUeq2[i-5]);
//commande2=Kcmd2+(controleurAxe2->getCorrecteur()->getUdelta());


}



/* ajout de correction command equivlente*/



//commande2=commande2+att2*acc_ueq;
//commande4=commande4+att4*acc_ueq4;



//----------------------------------------------------------------------------------------------------------------------------

if (fabs(erreur2)<=0.17)	//0.17 0.25
					{									
						commande2=relevePression2[i-1];
 					}//FIN test hh


if (fabs(erreur2)>0.17)	//0.2
					{									
					commande2 = commande2 + comi2;
					}//FIN test hh





if (fabs(erreur4)<=0.2)	//0.02
					{									
						commande4=relevePression4[i-1];
 					}//FIN test hh


if (fabs(erreur4)>0.2)	 //0.02
					{									
					commande4 = commande4 + comi4;
					}


//-----------------------------------------------

/*if (theta_mesuree4>1.39)	//0.02
					{									
						commande4=relevePression4[i-1];
            jeton=1;    
 					}//FIN test hh


if (jeton==1)	 //0.02
					{									
					commande4 = relevePression4[i-1];
					}*/





//	commande4 = commande4 + comi4;

//*****************************************************************************************************************************

//on essaie une acceleration de convergence  ////
// on rajoute -alpha*surface//

//commande2 = commande2 + comi2; modification karim j'ai mi dans la condition hh
//commande4 = commande4 + comi4;
// fin rajout acceleration de convergence//

//saturation des commandes

								if(commande2 <= SATMIN)	commande2 = SATMIN;
								if(commande2 >= SATMAX2)	commande2 = SATMAX2;

								if(commande4 <= SATMIN)	commande4 = SATMIN;
								if(commande4 >= SATMAX4)	commande4 = SATMAX4;


actionneur2->envoyerCommande(commande2);
actionneur4->envoyerCommande(commande4);

commande_precedente2 = commande2;
commande_precedente4 = commande4;

//releve des donnees

releveTemporel[i]	=	i*dt;//10ms

//releveUdelta2[i]	=	acc_ueq;//

releveUdelta2[i]	= controleurAxe2->getCorrecteur()->getUdelta();

//releveUdelta4[i]	=	acc_ueq4; //

releveUdelta4[i]= controleurAxe4->getCorrecteur()->getUdelta();
relevePression2[i]	= commande2;
relevePression4[i]	=	commande4;
releveUeq2[i]	=	controleurAxe2->getCorrecteur()->getUeq();
releveUeq4[i]	=	controleurAxe4->getCorrecteur()->getUeq();

releveThetaDesiree2[i]	=	theta_desiree2;
releveThetaDesiree4[i]	=	theta_desiree4;

releveThetaMesuree2[i]	=	theta_mesuree2;
releveThetaMesuree4[i]	=	theta_mesuree4;


releveErreur2[i]	=	controleurAxe2->getCorrecteur()->getErreur();
releveErreur4[i]	=	controleurAxe4->getCorrecteur()->getErreur();
releveSurFace2[i]	=	controleurAxe2->getCorrecteur()->getSurface();
releveSurFace4[i]	=	controleurAxe4->getCorrecteur()->getSurface();
releveSurfaceDot2[i]	=	controleurAxe2->getCorrecteur()->getSurfacePoint();
releveSurfaceDot4[i]	=	controleurAxe4->getCorrecteur()->getSurfacePoint();

//affichage des erreurs en degres
printf("\nerreur2=%f\terreur4=%f",releveErreur2[i]*180.0/pi,releveErreur4[i]*180.0/pi);


// determination des erreurs precedentes pour l'action pid //
erreur_prec2= - releveErreur2[i]; 
erreur_prec4= - releveErreur4[i];
////////////////////////////////////////////////////////////

				}	//FIN i

monHorlogeEchelon.arreter();


}	//FIN regulationTW

/********	FIN TW	FIN TW	FIN TW	FIN TW	*****/


/*********   STW	STW	STW	STW	STW	***/

/****************************************************/
/*	regulationSTW permet de reguler les axes 2 et 4	*/
/*	par l'utilisation d'un superTWHst										*/
/****************************************************/

void	regulationSTW	(ControleurDAxe*	controleurAxe2,
										ControleurDAxe*	controleurAxe4,
										CapteurDePosition*	capteur2,
										CapteurDePosition*	capteur4,
										Actionneur*	actionneur2,
										Actionneur*	actionneur4,
										float	theta_desiree2,
										float	theta_desiree4,
						double	*	releveTemporel,
						double	*	relevePression2,
						double	*	relevePression4,
						double	*	releveThetaDesiree2,
						double	*	releveThetaDesiree4,
						double	*	releveThetaMesuree2,
						double	*	releveThetaMesuree4,
						double	*	releveErreur2,
						double	*	releveErreur4,
						double	*	releveUeq2,
						double	*	releveUeq4,
						double	*	releveUdelta2,
						double	*	releveUdelta4,
						double	*	releveSurFace2,
						double	*	releveSurFace4,
						double	*	releveSurfaceDot2,
						double	*	releveSurfaceDot4)

{

int	deltaT	=	maPeriode;
float	dt	=	(Duree(deltaT,MS)).toSecondes();


float theta_mesuree2, theta_mesuree4;
float erreur2, erreur4;
float commande2, commande4;
//float erreur2, erreur4; // karim

NB_POINTS = 1000;

Horloge monHorlogeEchelon;
monHorlogeEchelon.demarrer(Duree(deltaT,MS));

controleurAxe2->getCorrecteur()->initialise(theta_desiree2);
controleurAxe2->getCorrecteur()->setSuivi(false);
controleurAxe4->getCorrecteur()->initialise(theta_desiree4);
controleurAxe4->getCorrecteur()->setSuivi(false);

for(int i=0; i<NB_POINTS; i++)	{
monHorlogeEchelon.attendre();

if (i<6)
{
theta_mesuree2	=	(float) (capteur2->lirePositionRAD());
theta_mesuree4	=	(float) (capteur4->lirePositionRAD());
}

if (i>=6)
{

theta_mesuree2	=	(float) (capteur2->lirePositionRAD2((float) (capteur2->lirePositionRAD()),releveThetaMesuree2[i-1],releveThetaMesuree2[i-2],releveThetaMesuree2[i-3],releveThetaMesuree2[i-4],releveThetaMesuree2[i-5]));
theta_mesuree4	=	(float) (capteur4->lirePositionRAD2((float) (capteur4->lirePositionRAD()),releveThetaMesuree4[i-1],releveThetaMesuree4[i-2],releveThetaMesuree4[i-3],releveThetaMesuree4[i-4],releveThetaMesuree4[i-5]));
}


commande2	=	controleurAxe2->getCorrecteur()->calculerCommande(theta_desiree2, 0.0f, 0.0f);
commande4	=	controleurAxe4->getCorrecteur()->calculerCommande(theta_desiree4, 0.0f, 0.0f);




//rajout de la pression a l axe 4 due a l axe 2
//if(theta_desiree4==0) commande4=commande4+(theta_mesuree2*1.80/pi);






//----------------------------------------------------------------------------------------------------------------------------


erreur2 = theta_desiree2 - theta_mesuree2;
erreur4 = theta_desiree4 - theta_mesuree4;





if (fabs(erreur2)<=0.15)	
					{									
						commande2=relevePression2[i-1];
 					}//FIN test hh


if (fabs(erreur2)>0.15)	
					{									
					commande2 = commande2;
					}//FIN test hh





if (fabs(erreur4)<=0.07)	
					{									
						commande4=relevePression4[i-1];
 					}//FIN test hh


if (fabs(erreur4)>0.07)	
					{									
					commande4 = commande4;
					}//FIN test hh







//*****************************************************************************************************************************













//saturation
								if(commande2 < SATMIN)	commande2 = SATMIN;
								if(commande2 > SATMAX2)	commande2 = SATMAX2;

								if(commande4 < SATMIN)	commande4 = SATMIN;
								if(commande4 > SATMAX4)	commande4 = SATMAX4;

actionneur2->envoyerCommande(commande2);
actionneur4->envoyerCommande(commande4);

releveTemporel[i]	=	i*dt;

relevePression2[i]	=	commande2;
relevePression4[i]	=	commande4;

releveThetaDesiree2[i]	=	theta_desiree2;
releveThetaDesiree4[i]	=	theta_desiree4;
releveThetaMesuree2[i]	=	theta_mesuree2;
releveThetaMesuree4[i]	=	theta_mesuree4;


// *****attention modififcation de getueq il rend la commande discontinue de U de la commande Stw*************
// ******************modifier dans la classe controleur STW par karim le 18/12/08*****************************

//releveUeq2[i]	=	controleurAxe2->getCorrecteur()->getUeq(); //ecriture de mourade (originale) enleve le 18/12/08
//releveUeq4[i]	=	controleurAxe4->getCorrecteur()->getUeq(); //ecriture de mourade (originale) enleve le 18/12/08



releveUeq2[i]	=	controleurAxe2->getCorrecteur()->getuDEUX(); //ecriture de mourade (originale) enleve le 18/12/08
releveUeq4[i]	=	controleurAxe4->getCorrecteur()->getuDEUX(); //ecriture de mourade (originale) enleve le 18/12/08


// ***************************************karim fin modification********************************************** 








releveUdelta2[i]	=	controleurAxe2->getCorrecteur()->getUdelta();
releveUdelta4[i]	=	controleurAxe4->getCorrecteur()->getUdelta();

releveErreur2[i]	=	controleurAxe2->getCorrecteur()->getErreur();
releveErreur4[i]	=	controleurAxe4->getCorrecteur()->getErreur();

releveSurFace2[i]	=	controleurAxe2->getCorrecteur()->getSurface();
releveSurFace4[i]	=	controleurAxe4->getCorrecteur()->getSurface();

releveSurfaceDot2[i]	=	controleurAxe2->getCorrecteur()->getSurfacePoint();
releveSurfaceDot4[i]	=	controleurAxe4->getCorrecteur()->getSurfacePoint();


//affichage des erreurs en degres
printf("\nerreur2=%f\terreur4=%f",releveErreur2[i]*180.0/pi,releveErreur4[i]*180.0/pi);


	}	//FIN i

monHorlogeEchelon.arreter();


}	//FIN regulationSTW

/******************FIN STW FIN STW	FIN STW	FIN STW	******/

/****************************************/
/*	finProgramme met fin a la fin				*/
/****************************************/

void	finProgramme()

{
	printf("\n\n\n*** TOUTE CHOSE A UNE FIN ***\n\n");

}	//FIN finProgramme

/**********************************************/
/*	demanderNumeroAxe entre l'axe selectionne	*/
/**********************************************/

void	demanderNumeroAxe(int &axeSelectionne)
{
	printf("\nSelectionner le numero de l'axe\n>");
fflush(stdin);
scanf("%d",&axeSelectionne);
printf("\nSelection de l'axe\t %d",axeSelectionne);
}	//FIN demanderNumeroAxe

/*********************************************/
/*	demanderEchelon fixe la pression desiree */
/*********************************************/

void	demanderEchelon(float &pressionEchelon)
{

	printf("\nEntrer l'echelon positif de pression desire en bars:\n>");

	scanf("%f",&pressionEchelon);
					
}	//FIN demanderEchelon


/*******************************************/
/* demanderArt permet l'entree des positions*/
/********************************************/
void demanderArt(float tminimum, float& tmaximum)
{
	printf("\n L'ANGLE INITIAL (DEG.) EST: %f", tminimum);
	printf("\n ENTRER L'ANGLE DESIRE (DEG.)\n>");
	scanf("%f", &tmaximum);
}//FIN demanderArt

/********************************************************/
/*	sinusoide permet d'entrer une fonction sinus*/
/********************************************************/

void sinusoide (float &amplitude, float &freq, float &dephase, float &pDepart)
{
	bool sortie	=	false;
	int	choice	=	0;

while(!sortie){
printf("\nEntrer l'amplitude de la sinusoide en deg.:\n");
scanf("%f",&amplitude);
printf("\nEntrer la frequence en hz:\n");
scanf("%f",&freq);
printf("\nEntrer le dephasage en deg.:\n");
scanf("%f",&dephase);
printf("\nEntrer le decalage initial en deg.:\n");
scanf("%f",&pDepart);
printf("\nPour valider, entrer 1 :\n");
scanf("%d",&choice);
if(choice==1)	sortie=true;
							}	//FIN while

}	//FIN sinusoide

/****************************/
/*	demanderPID fixe le PID	*/
/****************************/

void	demanderPID(float &P, float &I, float &D)
{
printf("\nLa valeur actuelle du gain P est:%f",P);
printf("\nEntrer le gain P\n>");
scanf("%f",&P);

printf("\nLa valeur actuelle du gain I est:%f",I);
printf("\nEntrer le gain I\n>");
scanf("%f",&I);

printf("\nLa valeur actuelle du gain D est:%f",D);
printf("\nEntrer le gain D\n>");
scanf("%f",&D);	

/*************************************************/





}	//FIN demanderPID

/**************************/
/*	demanderTW fixe le TW	*/
/**************************/
void	demanderTW(float &C, float &ULIM, float &AMIN, float &AMAX,	float	&ALPHA, float &ATTENUATOR, float &K) 
{
printf("\nLa valeur de C est:%f",C);
printf("\nEntrer la valeur de C\n>");
scanf("%f",&C);

printf("\nLa valeur de ULIM est:%f",ULIM);
printf("\nEntrer la valeur de ULIM\n>");
scanf("%f",&ULIM);

printf("\nLa valeur de AMIN est:%f",AMIN);
printf("\nEntrer la valeur de AMIN\n>");
scanf("%f",&AMIN);

printf("\nLa valeur de AMAX est:%f",AMAX);
printf("\nEntrer la valeur de AMAX\n>");
scanf("%f",&AMAX);

printf("\nLa valeur de ALPHA est:%f",ALPHA);//acceleration de convergence
printf("\nEntrer la valeur de ALPHA\n>");
scanf("%f",&ALPHA);

printf("\nLa valeur de K est:%f",K);//acceleration de premier ordre
printf("\nEntrer la valeur de K\n>");
scanf("%f",&K);

printf("\nLa valeur de l'ATTENUATEUR de Ueq est:%f",ATTENUATOR);//acceleration de convergence
printf("\nEntrer la valeur de ATTENUATOR\n>");
scanf("%f",&ATTENUATOR);

}	//FIN demanderTW

/****************************/


/****************************/
/*	demanderSTW fixe le STW */
/****************************/

void	demanderSTW(float &C, float &ULIM, float &W, float &LAMBDA, float	&ALPHA, float &ATTENUATOR, float &S0, float &K) // modifie karim 
{

printf("\nLa valeur de C est:%f",C);
printf("\nEntrer la valeur de C\n>");
scanf("%f",&C);

//********************************************karim ajout 17/12********************************

printf("\nLa valeur de ALPHA est:%f",ALPHA);//acceleration de convergence
printf("\nEntrer la valeur de ALPHA\n>");
scanf("%f",&ALPHA);

//********************************************fin karim ajout 17/12********************************

printf("\nLa valeur de ULIM est:%f",ULIM);
printf("\nEntrer la valeur de ULIM\n>");
scanf("%f",&ULIM);

printf("\nLa valeur de W est:%f",W);
printf("\nEntrer la valeur de W\n>");
scanf("%f",&W);

printf("\nLa valeur de LAMBDA est:%f",LAMBDA);
printf("\nEntrer la valeur de LAMBDA\n>");
scanf("%f",&LAMBDA);

printf("\nLa valeur de S0 est:%f",S0);
printf("\nEntrer la valeur de S0\n>");
scanf("%f",&S0);


//********************************************karim ajout 26/05/09********************************

printf("\nLa valeur de l'ATTENUATEUR de Ueq est:%f",ATTENUATOR);//acceleration de convergence
printf("\nEntrer la valeur de ATTENUATOR\n>");
scanf("%f",&ATTENUATOR);

printf("\nLa valeur de K est:%f",K);//acceleration de premier ordre
printf("\nEntrer la valeur de K\n>");
scanf("%f",&K);

//********************************************fin karim ajout 26/05/09********************************


}	//FIN demanderSTW

/***************************************************/
/***************************************************/
/*****************************/
/* modele geometrique direct */
/*****************************/
void  mgd(float q2, float q4)

{

float x,y;

	x = l1 * cos(q2) + l2 * cos(q2+q4);
	y = l1 * sin(q2) + l2 * sin(q2+q4);

printf("\nENCORE  1 MGD\n");
printf("**********************");
printf("\nLa position est:\n");
printf("X=	%f",x);
printf("\nY=	%f",y);

}	//FIN mgd

/******************************/
/* modele geometrique inverse */
/******************************/
void mgi(float posxf, float posyf)

{

float l, b, c, d, q2, q4;

	l = posxf*posxf + posyf*posyf;
	b = l1*l1;
	c = l2*l2;
	d = sqrt((l+b+c)*(l+b+c) - 2*(l*l + b*b + c*c));
	q2= atan2(posyf,posxf) - atan2(d,(l+b-c));
	q4= atan2(d,(l-b-c));

}//FIN mgi

/*************************************/
/*	demanderCorrecteur fixe le type  */
/*	de correcteur utilise						 */
/*************************************/

void	demanderCorrecteur (ControleurDAxe	*controleur,
													CorrecteurPID	*correcteurPID,
													CorrecteurTW	*correcteurTW,
													CorrecteurSTW	*correcteurSTW)
										

{
	int	choix	=	0;
	bool	sortie	=	false;

while(!sortie){


	printf("\nCHOISIR LE TYPE DE CORRECTEUR.");
	printf("\n1 PID");
	printf("\n2 TW");
	printf("\n3 STW\n");
	scanf("%d",&choix);


		if(choix==PID)	{
			printf("\n*** CORRECTEUR PID ***\n");
			controleur->setCorrecteur(correcteurPID);
			float	P=	correcteurPID->getP();
			float	I=	correcteurPID->getI();
			float	D=	correcteurPID->getD();
		demanderPID(P, I, D);
			correcteurPID->setP(P);
			correcteurPID->setI(I);
			correcteurPID->setD(D);
		sortie	=	true;
											}	//fin PID


		if(choix==TW)	{
			printf("\n*** CORRECTEUR TWISTING ***\n");
			controleur->setCorrecteur(correcteurTW);
			float C=	correcteurTW->getC();
			float ULIM=	correcteurTW->getUlim();
			float AMIN=	correcteurTW->getAmin();
			float AMAX=	correcteurTW->getAmax();
			float	ALPHA= correcteurTW->getAlpha();//acceleration de convergence
			float ATTENUATOR= correcteurTW->getAttenuator();//attenuateur de ueq
      float K=correcteurTW->get_k();// ajout karim 12/01/09

		demanderTW(C, ULIM, AMIN, AMAX, ALPHA, ATTENUATOR,K);
			correcteurTW->setC(C);
			correcteurTW->setUlim(ULIM);
			correcteurTW->setAmin(AMIN);
			correcteurTW->setAmax(AMAX);
			correcteurTW->setAlpha(ALPHA);
			correcteurTW->setAttenuator(ATTENUATOR);
      correcteurTW->set_k(K); // ajout karim 12/01/09

		sortie	=	true;
										}	//fin TW

		if(choix==STW)	{
			printf("\n*** CORRECTEUR SUPERTWISTING ***\n");
			controleur->setCorrecteur(correcteurSTW);
			float C=	correcteurSTW->getC();
			float ULIM=	correcteurSTW->getUlim();
			float W=	correcteurSTW->getW();
			float LAMBDA=	correcteurSTW->getLambda();
			float S0=	correcteurSTW->getSo();
      float ALPHA=correcteurSTW->getAlpha(); // karim ajout 17/12
      float K=correcteurSTW->get_k();// ajout karim 12/01/09

      float ATTENUATOR= correcteurSTW->getAttenuator();//attenuateur de ueq ajout le 26/05/09

     	demanderSTW(C, ULIM, W, LAMBDA, ALPHA, ATTENUATOR, S0,K); // karim modifie 17/12 et aussi le 26/05/09

			correcteurSTW->setC(C);
			correcteurSTW->setUlim(ULIM);
			correcteurSTW->setW(W); 
			correcteurSTW->setLambda(LAMBDA);
			correcteurSTW->setSo(S0);
      correcteurSTW->setAlpha(ALPHA);
      correcteurSTW->setAttenuator(ATTENUATOR); // karim ajout 26/05/09
      correcteurSTW->set_k(K); // ajout karim 25/05/09

     	sortie	=	true;
										}	//fin STW	

if(!sortie) printf("\nERREUR!!!!\n");

	} //FIN while not sortie

} //FIN demanderCorrecteur

/********************************/
/*	programmePrincipal					*/
/********************************/

void	programmePrincipal()

{

bool	sortie	=	true;
bool	refaire	=	true;
int	incrementation, validation;
int	numeroAxe, nbraxe; //l'identifiant et le nombre d'axes

char	fich[40];
char	cheminacces[40];

  hostAdd("hote", "172.16.136.12");
  netDevCreate("hote:", "hote", 1);
  fichier monFichier("hote:");

  //cartes
  CarteCIODAC16 ciodac16(0x200);
  CarteCIODAS64 ciodas64(0x250);


float	theta_initiale2, theta_initiale4,theta_initiale1;
float theta_desiree2	=	0.0;
float theta_desiree4	=	0.0;
float vitesse_desiree2 =0.0;
float vitesse_desiree4 =0.0;
float acceleration_desiree2 =0.0;
float acceleration_desiree4 =0.0;
float posit;
float amplitude2, amplitude4, freq2, freq4, pDepart2, pDepart4;
float omega2, omega4, dephase2, dephase4;


NB_POINTS = 1000;

double ** angle = new (double *)[2];//consigne articulaire
for(int i=0;i<2;i++)	angle[i] = new (double) [NB_POINTS];

double ** vitang = new (double *)[2];//consigne vitesse articulaire
for(int i=0;i<2;i++)	vitang[i] = new (double) [NB_POINTS];

double ** accang = new (double *)[2];//consigne acceleration articulaire
for(int i=0;i<2;i++)	accang[i] = new (double) [NB_POINTS];

double * releveTemporel	= new (double)[NB_POINTS];//temps

double * relevePression	= new (double)[NB_POINTS];//pression axe2

double * relevePression2	= new (double)[NB_POINTS];//pression axe4

double * releveUeq = new (double)[NB_POINTS];//uequivalent axe2

double * releveUeq2	= new (double)[NB_POINTS];//uequivalent axe4

double * releveUdelta = new (double)[NB_POINTS];//udiscontinu axe2

double * releveUdelta2 = new (double)[NB_POINTS];//udiscontinu axe4

double * releveSurFace = new (double)[NB_POINTS];//S axe2

double * releveSurFace2 = new (double)[NB_POINTS];//S axe4

double * releveSurfaceDot = new (double)[NB_POINTS];//Spoint axe2

double * releveSurfaceDot2 = new (double)[NB_POINTS];//Spoint axe4

double * releveThetaDesiree = new (double)[NB_POINTS];

double * releveThetaDesiree2 = new (double)[NB_POINTS];

double * releveVitesseDesiree = new (double)[NB_POINTS];

double * releveVitesseDesiree2 = new (double)[NB_POINTS];

double * releveAccelerationDesiree = new (double)[NB_POINTS];

double * releveAccelerationDesiree2 = new (double)[NB_POINTS];

double * releveThetaMesuree = new (double)[NB_POINTS];

double * releveThetaMesuree2 = new (double)[NB_POINTS];

double * releveVitesseMesuree = new (double)[NB_POINTS];

double * releveVitesseMesuree2 = new (double)[NB_POINTS];

double * releveErreur = new (double)[NB_POINTS];

double * releveErreur2 = new (double)[NB_POINTS];

double * X = new (double)[NB_POINTS];
double * Y = new (double)[NB_POINTS];

int	numeroMenu;
bool	choice= false;
  


  //Tableau des Delta_Pression
float deltaOrigine[]={ DELTA_INIT_AXE1, DELTA_INIT_AXE2, DELTA_INIT_AXE3, DELTA_INIT_AXE4,
                         DELTA_INIT_AXE5, DELTA_INIT_AXE6, DELTA_INIT_AXE7 };

  //pointeurs des actionneurs
  Actionneur* actionneurAxe1 = new Actionneur(&ciodac16,VOIE_ACTIONNEUR1_MUSCLE1,VOIE_ACTIONNEUR1_MUSCLE2);
  Actionneur* actionneurAxe2 = new Actionneur(&ciodac16,VOIE_ACTIONNEUR2_MUSCLE1,VOIE_ACTIONNEUR2_MUSCLE2);
  Actionneur* actionneurAxe3 = new Actionneur(&ciodac16,VOIE_ACTIONNEUR3_MUSCLE1,VOIE_ACTIONNEUR3_MUSCLE2);
  Actionneur* actionneurAxe4 = new Actionneur(&ciodac16,VOIE_ACTIONNEUR4_MUSCLE1,VOIE_ACTIONNEUR4_MUSCLE2);
  Actionneur* actionneurAxe5 = new Actionneur(&ciodac16,VOIE_ACTIONNEUR5_MUSCLE1,VOIE_ACTIONNEUR5_MUSCLE2);
  Actionneur* actionneurAxe6 = new Actionneur(&ciodac16,VOIE_ACTIONNEUR6_MUSCLE1,VOIE_ACTIONNEUR6_MUSCLE2);
  Actionneur* actionneurAxe7 = new Actionneur(&ciodac16,VOIE_ACTIONNEUR7_MUSCLE1,VOIE_ACTIONNEUR7_MUSCLE2);
  
   actionneurAxe2->setPressionRepos(DELTA_INIT_AXE2/100.0f);
	 actionneurAxe4->setPressionRepos(DELTA_INIT_AXE4/100.0f); 
   //actionneurAxe1->setPressionRepos(DELTA_INIT_AXE1/100.0f); // karim ajout le 30/08/08
 
  Actionneur* actionneurAxe[]={
    actionneurAxe1, actionneurAxe2, actionneurAxe3, actionneurAxe4,
    actionneurAxe5, actionneurAxe6, actionneurAxe7 };

  //pointeurs des capteurs
  CapteurDePosition* capteurAxe1 = new CapteurDePosition(&ciodas64,VOIE_CAPTEUR_AXE1, RAP_MECA_CAP_1*PENTE_CAPTEUR,OFFSET_CAPTEUR_AXE1);
  CapteurDePosition* capteurAxe2 = new CapteurDePosition(&ciodas64,VOIE_CAPTEUR_AXE2, RAP_MECA_CAP_2*PENTE_CAPTEUR,OFFSET_CAPTEUR_AXE2);
  CapteurDePosition* capteurAxe3 = new CapteurDePosition(&ciodas64,VOIE_CAPTEUR_AXE3, RAP_MECA_CAP_3*PENTE_CAPTEUR,OFFSET_CAPTEUR_AXE3);
  CapteurDePosition* capteurAxe4 = new CapteurDePosition(&ciodas64,VOIE_CAPTEUR_AXE4, RAP_MECA_CAP_4*PENTE_CAPTEUR,OFFSET_CAPTEUR_AXE4);
  CapteurDePosition* capteurAxe5 = new CapteurDePosition(&ciodas64,VOIE_CAPTEUR_AXE5, RAP_MECA_CAP_5*PENTE_CAPTEUR,OFFSET_CAPTEUR_AXE5);
  CapteurDePosition* capteurAxe6 = new CapteurDePosition(&ciodas64,VOIE_CAPTEUR_AXE6, RAP_MECA_CAP_6*PENTE_CAPTEUR,OFFSET_CAPTEUR_AXE6);
  CapteurDePosition* capteurAxe7 = new CapteurDePosition(&ciodas64,VOIE_CAPTEUR_AXE7, RAP_MECA_CAP_7*PENTE_CAPTEUR,OFFSET_CAPTEUR_AXE7);
  
	CapteurDePosition* capteurAxe[] = {
    capteurAxe1, capteurAxe2, capteurAxe3, capteurAxe4,
    capteurAxe5, capteurAxe6, capteurAxe7 };


/*********************************************************************/
/* INITIALISATION des CORRECTEURS   **********************************/
/*********************************************************************/

/* correcteur PID coeff :		capteur	  p, i, d, periode d echantill.			*/
/* correcteur TW coeff:			capteur,	b, a2, a1, periode d echantill.		*/
/* correcteur STW coeff:		capteur,	b, a2, a1, periode d echantill.		*/

	// PID

float	periodEch	=	(Duree(10,MS)).toSecondes();

CorrecteurPID* correcteurPIDAxe4 = new CorrecteurPID(capteurAxe4, 0.2865, 5.73, 0.0802, periodEch); //0.005, 0.100, 0.0014

CorrecteurPID* correcteurPIDAxe2 = new CorrecteurPID(capteurAxe2, 0.0573, 2.865, 0.2865, periodEch); //0.001, 0.05, 0.005

CorrecteurPID* correcteurPIDAxe1 = new CorrecteurPID(capteurAxe1, 0, 0, 0, periodEch); //ajout karim 





// TW (TWHsting algo.2sliding)
//axe 2 en deg/rad	3550	26	168			//18 au lieu de 26
//axe 4 en deg/rad	1500	5.596	49.72
/*	CorrecteurTW* correcteurTWAxe4   = new CorrecteurTW(capteurAxe4, 26.18, 5.596, 49.72, periodEch);//0.5265,   1500 etc

	CorrecteurTW* correcteurTWAxe2   = new CorrecteurTW(capteurAxe2, 61.96, 26.0, 168.0, periodEch);//0.37			3550 etc
*/
//CorrecteurTW* correcteurTWAxe4   = new CorrecteurTW(capteurAxe4, 52.39, 9.06, 129.67, periodEch); // mourade valeurs

//	CorrecteurTW* correcteurTWAxe2   = new CorrecteurTW(capteurAxe2, 17.44, 12.54, 61.94, periodEch); //mourade valeurs

CorrecteurTW* correcteurTWAxe4   = new CorrecteurTW(capteurAxe4, 43.47, 5.99, 89.77, periodEch); // karim valeurs 

CorrecteurTW* correcteurTWAxe2   = new CorrecteurTW(capteurAxe2, 12.341, 10.78, 40.1, periodEch); //karim valeur

CorrecteurTW* correcteurTWAxe1   = new CorrecteurTW(capteurAxe1, 0, 0, 0, periodEch); //karim valeurs pour l'axe 1






// STW (superTWHsting algo.2sliding)

  CorrecteurSTW* correcteurSTWAxe4   = new CorrecteurSTW(capteurAxe4, 43.47, 5.99, 89.77, periodEch);//1500 etc

	CorrecteurSTW* correcteurSTWAxe2   = new CorrecteurSTW(capteurAxe2, 12.341, 10.78, 40.1, periodEch);//3550 etc


	//CorrecteurSTW* correcteurSTWAxe4   = new CorrecteurSTW(capteurAxe4, 52.39, 9.06, 129.67, periodEch);//1500 etc ancienne valeurs

	//CorrecteurSTW* correcteurSTWAxe2   = new CorrecteurSTW(capteurAxe2, 17.44, 12.54, 61.94, periodEch);//3550 etc ancienne valeur
  
  CorrecteurSTW* correcteurSTWAxe1   = new CorrecteurSTW(capteurAxe1, 0, 0, 0, periodEch);//karim valeur pour l'axe 1




/* INITIALISATION CONTROLEURS D AXES */

	ControleurDAxe* controleurAxe4 = new ControleurDAxe(actionneurAxe4,
																										  correcteurPIDAxe4,
																										  deltaOrigine[3]/100.0,
																											GAIN_BO_DEGRE_AXE4);

	ControleurDAxe* controleurAxe2 = new ControleurDAxe(actionneurAxe2,
																										  correcteurPIDAxe2,
																										  deltaOrigine[1]/100.0,
																											GAIN_BO_DEGRE_AXE2);




//ControleurDAxe* controleurAxe1 = new ControleurDAxe(actionneurAxe1,
//																										  correcteurPIDAxe1,
//																										  deltaOrigine[0]/100.0,
//																											GAIN_BO_DEGRE_AXE1);  // karim ajout pb a regler au niveau de GAIN_BO_DEGRE_AXE1






/* ETABLISSEMENT DES BUTEES ARTICULAIRES */

	controleurAxe4->setPositionMin(AXE_4_MIN);
	controleurAxe4->setPositionMax(AXE_4_MAX);

	controleurAxe2->setPositionMin(AXE_2_MIN);
	controleurAxe2->setPositionMax(AXE_2_MAX);

/*	ETABLISSEMENT DES BUTEES DE PRESSION	*/
//un offset existe pour les 2 axes:
//axe2: -2.30 bars		(2.5-(-2.3))=4.8
//axe4: -1.80 bars		(2.5-(-1.8))=4.3

	controleurAxe4->setSaturationMin(SATMIN);// deltapression min
	controleurAxe4->setSaturationMax(SATMAX4);// deltapression max

	controleurAxe2->setSaturationMin(SATMIN);
	controleurAxe2->setSaturationMax(SATMAX2);

  //controleurAxe1->setSaturationMin(SATMIN); //ajout karim
  //controleurAxe1->setSaturationMax(SATMAX1); //ajout karim


 

////////////////////////////////////////////////////
//PROCEDURE DE DEGONFLEMENT AVANT TEST
//////////////////////////////////////////
		printf("DEGONFLEMENT.....PATIENCE\n");
					for	(incrementation=0; incrementation<NB_AXES ; incrementation++ )

						degonfleMuscle((int) (actionneurAxe[incrementation]));

		printf("DEGONFLEMENT OK......PRET\n");
/////////////////////////////////////////////////////////
//AFFICHAGE DES PRESSIONS INITIALES
////////////////////////////////////
printf("\nPRESSIONS INITIALES DANS LES MUSCLES:\n");
	for	(incrementation=0; incrementation<NB_AXES ; incrementation++ )
	printf("M1= %f		M2=	%f\n",(float)actionneurAxe[incrementation]->pressionMuscle1(),(float)actionneurAxe[incrementation]->pressionMuscle2());


/*******************************************/

/* DEBUT DES TESTS */
while(refaire && sortie)
{

nbMuscleGonfle	=	0;

//  printf("\033[2J");      //effacement de l'écran
	printf("\n\n\n");
	printf("POUR ACTIONNER LE BRAS, INITIALISEZ LE!!\n");
	printf("****************************************\n");
	printf("1-- INITIALISER LE BRAS\n");
	printf("2-- DEGONFLER LES MUSCLES\n");
	printf("3-- LIRE LES PRESSIONS DANS LES MUSCLES\n");
	printf("4-- INJECTION D UNE PRESSION POUR UN AXE\n");
	printf("5-- REGULATION POUR LES AXES 2 ET 4\n");
	printf("6-- SUIVI DE SINUSOIDES POUR LES AXES 2 ET 4\n");
	printf("7-- ENREGISTRER LES RESULTATS\n");
	printf("8-- LIRE LES POSITIONS (deg) DES CAPTEURS\n");
	printf("9-- INJECTER DES PRESSIONS SUR LES AXES 2 ET 4\n");
	printf("10- SUIVRE UNE TRAJECTOIRE ARTICULAIRE: AXES 2 et 4\n");
	printf("11- SUIVRE UNE TRAJECTOIRE OPERATIONNELLE: AXES 2 et 4\n");
  printf("0-- QUITTER\n");
  printf("  *******************\n");
  printf("\n>");

	numeroMenu	=	0;
	scanf("%d",&numeroMenu);
	switch(numeroMenu)
	{
/***********/
		case	0:{
						sortie = false;
						break;}//FIN Case 0

/***********/
		case	1:{

	/*********************************************************/
		printf("\nPORTER LA PRESSION D'ENTREE A 5 BARS.\n");
		printf("PATIENCE!!!\n");
	/*********************************************************/
				 for (incrementation=0; incrementation<NB_AXES ; incrementation ++ )
        {
          taskSpawn("initMuscle", 81, 0, 22000, (FUNCPTR) initialiseMuscle,
                    (int) (actionneurAxe[incrementation]), deltaOrigine[incrementation],0,0,0,0,0,0,0,0);
        }
        
        while (nbMuscleGonfle != NB_AXES)	taskDelay(sysClkRateGet());
        	/*************************************************/
						printf("INITIALISATION DU BRAS EN COURS.\n");
						printf("ATTENDRE QUELQUES INSTANTS...................\n");
					/*************************************************/

//initialisation des decalages capteurs pour utiliser lirePositionRAD()
//la configuration initiale est consideree comme la station ZERO
//ces offset sont a integrer dans le vecteur offset au tout debut

			taskDelay(sysClkRateGet()); //tempo de 1s



					/*********************************/
						printf("ENTRER 1 LORSQUE PRET.\n");			
					/**********************************/


			scanf("%d",&validation);
	while(validation !=1){scanf("%d",&validation);}

for(int i=0;i<NB_AXES;i++) {


	capteurAxe[i]->setOffset((float)(capteurAxe[i]->lirePosition()));//affectation dans _offset2
	printf("axe %d:position %f\n",(i+1),(float)(capteurAxe[i]->lirePosition()));


}//FIN i 
		
			/*************************************************/
						printf("** INITIALISATION DU BRAS TERMINEE **\n");
						printf("** INITIALISATION DES CAPTEURS TERMINEE **\n");
			/*************************************************/
						printf("\n** PRET **\n");

				break;}//FIN Case 1

/************/
	case	2:{




	/*********************************************************/
		printf("\nDEGONFLEMENT DES MUSCLES\n");
	/*********************************************************/
		printf("PATIENCE\n");
					for	(incrementation=0; incrementation<NB_AXES ; incrementation++ )

						degonfleMuscle((int) (actionneurAxe[incrementation]));

		printf("DEGONFLEMENT DES MUSCLES TERMINE\n");

				break;}//FIN Case 2

/***********/
	case	3:{


/////////////////////////////////////////////////////////
//AFFICHAGE DES PRESSIONS INITIALES
////////////////////////////////////
printf("\nPRESSIONS EXISTANTES:\n");
	for	(incrementation=0; incrementation<NB_AXES ; incrementation++ )
	printf("M1= %f		M2=	%f\n",(float)actionneurAxe[incrementation]->pressionMuscle1(),(float)actionneurAxe[incrementation]->pressionMuscle2());
	
	break;} // FIN Case 3 


/***********/
	case	4:{





		/*********************************************************/
			printf("\nINJECTION D UNE PRESSION CONSTANTE\n");
		/*********************************************************/

			demanderNumeroAxe(numeroAxe);
			float	pEchelon	=	0.0;
			float posit = 0.0;
		choice = false;
		while(!choice){
			demanderEchelon(pEchelon);

if(pEchelon>5.0	||	pEchelon<0.0) printf("\nPRESSION INCORRECTE\n");
	else choice = true; }//Fin while

			echelonAxe(actionneurAxe[(numeroAxe-1)],
												capteurAxe[(numeroAxe-1)],
												numeroAxe,
												pEchelon,
												releveTemporel,
												releveThetaMesuree,
												relevePression);

	printf("\nPRESSION INJECTEE (bars): %f",pEchelon);

	posit	=	(float) (capteurAxe[(numeroAxe-1)]->lirePositionRAD());

	printf("\nPOSITION (deg):     %f",(posit*180.0)/pi);

	printf("\nECHELON PRESSION TERMINE.\n");

	printf("\nENREGISTRER LES DONNEES?\n\n");
	printf("\nENTRER 0 POUR OUI, AUTRE POUR NON.\n");	
	validation	=	1;
	scanf("%d",&validation);
	if(validation==0){
	printf("\n\nEntrer le nom de fichier\n");
	scanf("%s",fich);	

	monFichier.sauvegarde_identification(fich, numeroAxe, pEchelon, releveTemporel,
		releveThetaMesuree, relevePression);
										} //FIN validation
	
	break;} // FIN case 4


/***********/
	case	5:{



		/*********************************************************/
	printf("	****	REGULATION EN POSITION POUR LES AXES 2 ET 4	****\n");
		/*********************************************************/

	printf("\nREGULATION EN POSITION\n");
	nbraxe = 2;//2 axes


for(int i=0;i<NB_AXES;i++)
	theta_origine[i]	=	(float) (capteurAxe[i]->lirePositionRAD());


			theta_initiale2	=	(theta_origine[1]*180.0f)/pi	;
			theta_initiale4	=	(theta_origine[3]*180.0f)/pi	;
      theta_initiale1	=	(theta_origine[0]*180.0f)/pi	; //karim ajout

			printf("\nENTRER LES ANGLES DESIRES ABSOLUS.\n");
	    printf("SEULS LES ANGLES POSITIFS SONT ACCEPTES.\n");

			printf("\n*** AXE 2 ***\n");
			demanderArt(theta_initiale2, theta_desiree2);
printf("angle initial2 (deg): %f\n",theta_initiale2);

			if(theta_desiree2 < AXE_2_MIN) theta_desiree2 =  AXE_2_MIN;
			if(theta_desiree2 > AXE_2_MAX) theta_desiree2 =  AXE_2_MAX;				

printf("angle desire 2 (deg) = %f\n",theta_desiree2);

			theta_initiale2	=	theta_origine[1];
			theta_desiree2 = (theta_desiree2	*	pi)/180.0f;

			printf("\n*** AXE 4 ***\n");
			demanderArt(theta_initiale4, theta_desiree4);
printf("angle initial4 (deg) = %f\n",theta_initiale4);

			if(theta_desiree4 < AXE_4_MIN) theta_desiree4 =  AXE_4_MIN;
			if(theta_desiree4 > AXE_4_MAX) theta_desiree4 =  AXE_4_MAX;				

printf("angle desire 4 (deg): %f\n",theta_desiree4);

			theta_initiale4	=	theta_origine[3];
			theta_desiree4 = (theta_desiree4	*	pi)/180.0f;
printf("OK POUR LES ANGLES\n");
/** choix des correcteurs **/
			printf("\nCHOISIR LA CORRECTION DES 2 AXES\n");
			printf("\nLE TYPE DE CORRECTION DOIT ETRE LE MEME POUR LES 2.\n");

			printf("\n*** CORRECTION AXE 2 ***\n");
			demanderCorrecteur(controleurAxe2, correcteurPIDAxe2, correcteurTWAxe2, correcteurSTWAxe2);
			printf("\n*** CORRECTION AXE 4 ***\n");
			demanderCorrecteur(controleurAxe4, correcteurPIDAxe4, correcteurTWAxe4, correcteurSTWAxe4);

	printf("POUR DEMARRER ENTRER 1\n");
	validation = 0;
	scanf("%d",&validation);
	while (validation	!=1)	{
				printf("POUR DEMARRER ENTRER 1\n");
				scanf("%d",&validation);	
													}//FIN while

if(controleurAxe2->getCorrecteur()->isPID() && controleurAxe4->getCorrecteur()->isPID()){

	printf("\n*** REGULATION PID EN ACTION ***\n");

	regulationPID(controleurAxe2, controleurAxe4, capteurAxe[1], capteurAxe[3],
							actionneurAxe[1], actionneurAxe[3], theta_desiree2, theta_desiree4,
							releveTemporel, relevePression, relevePression2, releveThetaDesiree,
							releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
				 			releveErreur, releveErreur2);

	printf("\n*** FIN DE REGULATION PID ***\n");


	}	else	if(controleurAxe2->getCorrecteur()->isTW() && controleurAxe4->getCorrecteur()->isTW()){

	printf("\n*** REGULATION TW EN ACTION ***\n");

	regulationTW(controleurAxe2, controleurAxe4, capteurAxe[1], capteurAxe[3],
						actionneurAxe[1], actionneurAxe[3], theta_desiree2, theta_desiree4,
						releveTemporel, relevePression, relevePression2, releveThetaDesiree,
						releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
						releveErreur, releveErreur2, releveUeq, releveUeq2,
						releveUdelta, releveUdelta2, releveSurFace, releveSurFace2,
						releveSurfaceDot, releveSurfaceDot2);

	printf("\n*** FIN DE REGULATION TW ***\n");


		} else	if(controleurAxe2->getCorrecteur()->isSTW() && controleurAxe4->getCorrecteur()->isSTW()){

	printf("\n*** REGULATION STW EN ACTION ***\n");

	regulationSTW(controleurAxe2, controleurAxe4, capteurAxe[1], capteurAxe[3],
						actionneurAxe[1], actionneurAxe[3], theta_desiree2, theta_desiree4,
						releveTemporel, relevePression, relevePression2, releveThetaDesiree,
						releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
						releveErreur, releveErreur2, releveUeq, releveUeq2,
						releveUdelta, releveUdelta2, releveSurFace, releveSurFace2,
						releveSurfaceDot, releveSurfaceDot2);

	printf("\n*** FIN DE REGULATION STW ***\n");

		}

else {printf("\n*** ERREUR DE CORRECTEURS ***\n");}

	printf("*** REGULATION TERMINEE ***\n");

	break;} //FIN Case 5 (regulation des 2 axes)




/***********/
	case	6:{




		/**********************************************************************/
			printf("SUIVI DE SINUSOIDES PAR LES AXES 2 ET 4.\n");
		/**********************************************************************/	


	nbraxe = 2;// 2 axes

/*****************************************/
	/** CHOIX DES SINUSOIDES **/
/****************************************/

/*AXE 2*/
choice = false;
while (!choice)
{
printf("\n** ENTRER LA SINUSOIDE DE L'AXE 2 **\n");
sinusoide(amplitude2, freq2, dephase2, pDepart2);

//affichage test
printf("AMPLITUDE= %f\n",amplitude2);
printf("FREQUENCE= %f\n",freq2);
printf("DEPHASAGE= %f\n",dephase2);
printf("OFFSET= %f\n",pDepart2);

//verification des butees articulaires
if( (pDepart2+amplitude2) <= AXE_2_MAX  || (pDepart2-amplitude2) >= AXE_2_MIN  ) choice = true;
	else printf("\nHORS LIMITES\n");

}//FIN while

//unites SI axe 2
pDepart2 = (pDepart2 * pi)/180.0f;
amplitude2 = (amplitude2 * pi)/180.0f;
omega2 = (2 * pi * freq2);
dephase2 = (dephase2 * pi)/180.0f;

//affichage du signal d'entree pour l'axe 2
printf("\n a2 sin ( w2t + phi2) + offset2= %f * sin (%f t + %f) +%f\n", amplitude2, omega2, dephase2, pDepart2);


printf("\n** OK POUR LA SINUSOIDE DE L'AXE 2 **\n");
///////////////////////////////////////////////////////
/*AXE 4*/
choice = false;
while (!choice)
{
printf("\n** ENTRER LA SINUSOIDE DE L'AXE 4 **\n");
sinusoide(amplitude4, freq4, dephase4, pDepart4);

//affichage test
printf("AMPLITUDE= %f\n",amplitude4);
printf("FREQUENCE= %f\n",freq4);
printf("DEPHASAGE= %f\n",dephase4);
printf("OFFSET= %f\n",pDepart4);

//verification des butees articulaires
if( (pDepart4+amplitude4) <= AXE_4_MAX || (pDepart4-amplitude4) >= AXE_4_MIN ) choice = true;
	else printf("\nHORS LIMITES\n");
}//FIN while

//unites SI axe 4
pDepart4 = (pDepart4 * pi)/180.0f;
amplitude4 = (amplitude4 * pi)/180.0f;
omega4 = (2 * pi * freq4);
dephase4 = (dephase4 * pi)/180.0f;

//affichage du signal d'entree pour l'axe 4
printf("\n a4 sin ( w4t + phi4) + offset4 = %f * sin (%f t + %f) + %f\n", amplitude4, omega4, dephase4, pDepart4);

printf("\n** OK POUR LA SINUSOIDE DE L'AXE 4 **\n");
/*************************************************************/

/***************************************/
/********* CHOIX DES CORRECTEURS ********/
/***************************************/

/////ici les correcteurs sont fixes a TWISTING ////
			printf("\nCHOISIR LA CORRECTION DES 2 AXES COMME TWISTING\n");


			printf("\n*** CORRECTION AXE 2 ***\n");
			demanderCorrecteur(controleurAxe2, correcteurPIDAxe2, correcteurTWAxe2, correcteurSTWAxe2);
			printf("\n*** CORRECTION AXE 4 ***\n");
			demanderCorrecteur(controleurAxe4, correcteurPIDAxe4, correcteurTWAxe4, correcteurSTWAxe4);


				sinusTW(controleurAxe2, controleurAxe4, capteurAxe[1], capteurAxe[3],
						actionneurAxe[1], actionneurAxe[3], amplitude2, pDepart2, omega2,
						amplitude4, pDepart4, omega4, dephase2, dephase4, releveTemporel,
						relevePression, relevePression2, releveThetaDesiree, releveThetaDesiree2, 
						releveThetaMesuree, releveThetaMesuree2, releveErreur, releveErreur2, 
						releveUeq, releveUeq2, releveUdelta, releveUdelta2, releveSurFace, releveSurFace2,
						releveSurfaceDot, releveSurfaceDot2);


		break;}//FIN Case 6
 

/**************/
		case	7:{
		/*********************************************************/
//			printf("ENREGISTREMENT DES DONNEES\n");
		/*********************************************************/	


printf("\nENREGISTRER LES DONNEES?\n\n");
printf("\nENTRER 1 POUR OUI.\n");	
validation	=	0;
scanf("%d",&validation);
if(validation==1){
	printf("\n\nEntrer le nom de fichier\n");
	scanf("%s",fich);

if(nbraxe==1){

	if(numeroAxe==2) {
			if(controleurAxe2->getCorrecteur()->isPID()) {
				monFichier.sauvegarde_PID(fich, numeroAxe, releveTemporel, relevePression,
								releveThetaDesiree, releveThetaMesuree, releveErreur,
				correcteurPIDAxe2->getP(), correcteurPIDAxe2->getI(),	correcteurPIDAxe2->getD());
																										}//fin PID

			if(controleurAxe2->getCorrecteur()->isTW()) {
				monFichier.sauvegarde_TW(fich, numeroAxe, releveTemporel, relevePression,
								releveUeq, releveUdelta, releveSurFace, releveSurfaceDot, releveThetaMesuree,
				releveThetaDesiree, releveErreur,correcteurTWAxe2->getC(),
correcteurTWAxe2->getUlim(), correcteurTWAxe2->getAmin(), correcteurTWAxe2->getAmax());
																									}//fin TW

			if(controleurAxe2->getCorrecteur()->isSTW()) {
				monFichier.sauvegarde_STW(fich, numeroAxe, releveTemporel, relevePression,
								releveUeq, releveUdelta, releveSurFace, releveSurfaceDot, releveThetaMesuree,
				releveThetaDesiree, releveErreur,correcteurSTWAxe2->getC(),
correcteurSTWAxe2->getUlim(), correcteurSTWAxe2->getW(), correcteurSTWAxe2->getLambda(),
	correcteurSTWAxe2->getSo());
																									}// fin STW
}// fin Axe2
	if(numeroAxe==4) {
			if(controleurAxe4->getCorrecteur()->isPID()) {
				monFichier.sauvegarde_PID(fich, numeroAxe, releveTemporel, relevePression,
								releveThetaDesiree, releveThetaMesuree, releveErreur,
				correcteurPIDAxe4->getP(), correcteurPIDAxe4->getI(),	correcteurPIDAxe4->getD());
																										}//fin PID

			if(controleurAxe4->getCorrecteur()->isTW()) {
				monFichier.sauvegarde_TW(fich, numeroAxe, releveTemporel, relevePression,
								releveUeq, releveUdelta, releveSurFace, releveSurfaceDot, releveThetaMesuree,
				releveThetaDesiree, releveErreur,correcteurTWAxe4->getC(),
correcteurTWAxe4->getUlim(), correcteurTWAxe4->getAmin(), correcteurTWAxe4->getAmax());
																									}// fin TW

			if(controleurAxe4->getCorrecteur()->isSTW()) {
				monFichier.sauvegarde_STW(fich, numeroAxe, releveTemporel, relevePression,
								releveUeq, releveUdelta, releveSurFace, releveSurfaceDot, releveThetaMesuree,
				releveThetaDesiree, releveErreur,correcteurSTWAxe4->getC(),
correcteurSTWAxe4->getUlim(), correcteurSTWAxe4->getW(), correcteurSTWAxe4->getLambda(),
	correcteurSTWAxe4->getSo());
																									}// fin STW
}//fin axe4

}//fin nbraxe=1

		else {
				if(controleurAxe4->getCorrecteur()->isPID()) {

monFichier.sauvegarde_PID2(fich, releveTemporel, relevePression, relevePression2,
releveThetaDesiree, releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2, 
releveErreur, releveErreur2, correcteurPIDAxe2->getP(), correcteurPIDAxe2->getI(), correcteurPIDAxe2->getD(), 
correcteurPIDAxe4->getP(), correcteurPIDAxe4->getI(),	correcteurPIDAxe4->getD());
																										}//fin PID


				if(controleurAxe4->getCorrecteur()->isTW()) {

monFichier.sauvegarde_TW2(fich, releveTemporel, relevePression, relevePression2,
releveThetaDesiree, releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
releveErreur, releveErreur2, releveUeq, releveUeq2, releveUdelta, releveUdelta2,
releveSurFace, releveSurFace2, releveSurfaceDot, releveSurfaceDot2, correcteurTWAxe2->getC(),
correcteurTWAxe2->getUlim(),correcteurTWAxe2->getAmin(), correcteurTWAxe2->getAmax(),
correcteurTWAxe2->getAlpha(),correcteurTWAxe2->getAttenuator(),
correcteurTWAxe4->getC(), correcteurTWAxe4->getUlim(), correcteurTWAxe4->getAmin(),
correcteurTWAxe4->getAmax(),correcteurTWAxe4->getAlpha(),correcteurTWAxe4->getAttenuator());
																										}//finTW

				if(controleurAxe4->getCorrecteur()->isSTW()) {

monFichier.sauvegarde_STW2(fich, releveTemporel, relevePression, relevePression2,
releveUeq, releveUeq2, releveUdelta, releveUdelta2,
releveThetaDesiree, releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
releveErreur, releveErreur2, releveSurFace, releveSurFace2,
releveSurfaceDot, releveSurfaceDot2, correcteurSTWAxe2->getC(), correcteurSTWAxe2->getUlim(),
correcteurSTWAxe2->getW(), correcteurSTWAxe2->getLambda(), correcteurSTWAxe2->getSo(),
correcteurSTWAxe4->getC(), correcteurSTWAxe4->getUlim(),
correcteurSTWAxe4->getW(), correcteurSTWAxe4->getLambda(), correcteurSTWAxe4->getSo());
																										}//finSTW

		}//FIN else 2axes

} //FIN validation (enregistrement)

		break;}//FIN Case 7

/***************/
		case	8:{
/*********/



			float klek=0.0;

			for(int i=0;i<NB_AXES;i++)
			{
				klek = (float)	((capteurAxe[i]->lirePositionRAD()));
				klek = klek*180.0/pi;
				printf("POSITION(deg.) DU CAPTEUR:	%d		%f\n",(i+1), klek);
			}//FIN i

						break;}//FIN Case 8

/**************/
		case	9:{
/************/



		/*********************************************************/
			printf("\nINJECTION DE PRESSIONS AXES 2 ET 4\n");
		/*********************************************************/

nbraxe = 2;
			
float	pEchelon2	=	0.0;
float	pEchelon4	=	0.0;
float posit2 = 0.0;
float posit4 = 0.0;
	choice = false;
		while(!choice){
			printf("AXE2\n");
			demanderEchelon(pEchelon2);
			printf("AXE4\n");
			demanderEchelon(pEchelon4);

if((pEchelon2>5.0	||	pEchelon2<0.0) || (pEchelon4>5.0	||	pEchelon4<0.0)) printf("\nPRESSION INCORRECTE\n");
	else choice = true; }//FIN while

			echelonDeuxAxes(actionneurAxe[1],
										actionneurAxe[3],
												capteurAxe[1],
												capteurAxe[3],
												pEchelon2,
												pEchelon4,
												releveTemporel,
												relevePression,
												relevePression2,
												releveThetaMesuree,
												releveThetaMesuree2);


	printf("\nPRESSIONS INJECTEES (bars):");
	printf("\nAXE2: %f",pEchelon2);
	printf("\nAXE4: %f",pEchelon4);

	posit2	=	(float) (capteurAxe[1]->lirePositionRAD());
	posit4	=	(float) (capteurAxe[3]->lirePositionRAD());


	printf("\nPOSITIONS (deg):");
	printf("\nAXE2: %f",(posit2*180.0)/pi);
	printf("\nAXE4: %f",(posit4*180.0)/pi);

	printf("\nECHELONS PRESSIONS TERMINES.\n");

	printf("\nENREGISTRER LES DONNEES?\n\n");
	printf("\nENTRER 0 POUR OUI, AUTRE POUR NON.\n");	
	validation	=	1;
	scanf("%d",&validation);
	if(validation==0){
	printf("\n\nEntrer le nom de fichier\n");
	scanf("%s",fich);	

	monFichier.sauvegarde_identification2(fich, releveTemporel, relevePression, relevePression2,
		releveThetaMesuree, releveThetaMesuree2);
		} //FIN validation (enregistrement)
	
	break;} //FIN Case 9 (echelons de pression pour 2 axes)

/**************/
		case	10:{
/************/


int di2, di4;
float posxi, posyi,posxf, posyf, theta2i, theta2f, theta4i, theta4f, vit2, vit4, acc2, acc4;
float q2, qdot2, qddot2, q4, qdot4, qddot4, tacc, tvit, ttot;
float tacc2, tacc4, tvit2, tvit4, ttot2, ttot4, dirrot2, dirrot4;
double x,y,l,b,c,d;
float t;
int T_suivi = 20; //50ms pour le suivi
float	tau	=	(Duree(T_suivi,MS)).toSecondes();

nbraxe = 2;

printf("\n A partir de deux positions cartesiennes dans le plan vertical,\n");
printf("celle de depart et celle d'arrivee, les variables articulaires\n");
printf("sont generees. Les trajectoires des axes 2 et 4 s'effectuent\n");
printf("de la position de depart a celle d'arrivee avec des vitesses et\n");
printf("et accelerations desirees.\n");

//lecture des positions
q2 = (float) (capteurAxe[1]->lirePositionRAD());
q4 = (float) (capteurAxe[3]->lirePositionRAD());
theta2i = q2; theta4i = q4;

/*****************************/
/* modele geometrique direct */
/*****************************/
/*
	x = l1 * cos(q2) + l2 * cos(q2+q4);
	y = l1 * sin(q2) + l2 * sin(q2+q4);

printf("\nLa position de depart est:\n");
printf("X=	%f",x);
printf("\nY=	%f",y);*/
/////////////////
mgd(q2,q4);
/////////////////
/*
choice=false;
while(!choice)
{
*/
printf("\n\nEntrer la position d'arrivee:\n");
printf("X=\n");
scanf("%f",&posxf);
printf("Y=\n");
scanf("%f",&posyf);
/******************************/
/* modele geometrique inverse */
/******************************/

	l = posxf*posxf + posyf*posyf;
	b = l1*l1;
	c = l2*l2;
	d = sqrt((l+b+c)*(l+b+c) - 2*(l*l + b*b + c*c));
	q2= atan2(posyf,posxf) - atan2(d,(l+b-c));
	q4= atan2(d,(l-b-c));
/*
if((q4<= AXE_4_MAX && q4>= AXE_4_MIN) && (q2<= AXE_2_MAX && q2>= AXE_2_MIN)) choice=true;

}//FIN while
*/
theta2f=q2; theta4f=q4;
printf("\nVariables articulaires initiales deg.:\n");
printf("q2= %f	q4= %f", theta2i*180.0/pi, theta4i*180.0/pi);
printf("\nVariables articulaires finales deg.:\n");
printf("q2= %f	q4= %f", theta2f*180.0/pi, theta4f*180.0/pi);

printf("\nEntrer les modules des vitesses articulaires en  rad/s:\n");
printf("VIT. AXE 2:>");
scanf("%f",&vit2);
printf("\nVIT. AXE 4:>");
scanf("%f",&vit4);
printf("\nEntrer les modules des accelerations articulaires en rad/s2:\n");
printf("ACC. AXE 2:>");
scanf("%f",&acc2);
printf("\nACC. AXE 4:>");
scanf("%f",&acc4);

/**********************************/
//generation des profils de vitesse 
/**********************************/
/*axe 2 */
/********/
	if(fabs( theta2f- theta2i) >= (vit2*vit2/acc2)) //profil trapezoidal
	{
		printf("\nAxe 2 :Profil trapezoidal.......");
		tacc2 = vit2/acc2;
		tvit2 = fabs(theta2f - theta2i)/vit2 -tacc2;
		ttot2 = tvit2 + 2*tacc2;
	}
	else { //profil triangulaire
					printf("\nAxe 2 :Profil triangulaire.......");
					vit2 = sqrt(fabs(theta2f - theta2i) * acc2);
					tacc2 = vit2/acc2;
					tvit2 = 0.0f;
					ttot2 = 2*tacc2;
				}
printf("tacc2= %f\n",tacc2);
printf("tvit2= %f\n",tvit2);
printf("ttot2= %f\n",ttot2);

/* axe 4 */
	if(fabs( theta4f- theta4i) >= (vit4*vit4/acc4)) //profil trapezoidal
	{
		printf("\nAxe 4 :Profil trapezoidal......");
		tacc4 = vit4/acc4;
		tvit4 = fabs(theta4f - theta4i)/vit4 -tacc4;
		ttot4 = tvit4 + 2*tacc4;
	}
	else { //profil triangulaire
					printf("\nAxe 4 :Profil triangulaire.....");
					vit4 = sqrt(fabs(theta4f - theta4i) * acc4);
					tacc4 = vit4/acc4;
					tvit4 = 0.0f;
					ttot4 = 2*tacc4;
				}
printf("tacc4= %f\n",tacc4);
printf("tvit4= %f\n",tvit4);
printf("ttot4= %f\n",ttot4);

/* ajustement des temps pour finir ensemble */
if(ttot2 >= ttot4) {
		printf("* l'axe 2 est le + lent\n");
										ttot=ttot2;
										vit4 = (theta4f-theta4i)/(ttot-tacc4);
										acc4 = vit4/tacc4;
										tvit4 = ttot - 2*tacc4;
										}
	else {	ttot=ttot4;
		printf("* l'axe 4 est le + lent\n");
					vit2 = (theta2f-theta2i)/(ttot-tacc2);
					acc2 = vit2/tacc2;
					tvit2 = ttot -2*tacc2;
				}
/*****************************************************/
printf("Les temps d'acceleration , de vitesse const. et total sont:\n");
printf("AXE 2: %f	%f	%f\n",tacc2,tvit2,ttot);
printf("AXE 4: %f	%f	%f\n",tacc4,tvit4,ttot);

/****************************************************/
//generation des trajectoires
/*******************************/
NB_POINTS = int (ttot/tau +1.0);
printf("nbre de points est :%d\n",NB_POINTS);

	int validation	=	0;
	while (validation!=1) {
	printf("\nENTRER 1 POUR CONTINUER.\n");	
	scanf("%d",&validation);
} //end validation



for(int i=0; i<NB_POINTS; i++)	{

/*******************/
t= i*tau; //temps


if (t==0.0) {
							q2=theta2i; q4=theta4i;
							qdot2=0.0; qddot2=0.0;
							qdot4=0.0; qddot4=0.0;}

else {//t<>0

/***********************************/
/* generation de trajectoire axe 2 */
/***********************************/

	if((theta2f - theta2i) < 0.0)	di2 = -1;//sens de rotation								

		else di2 = 1;

//generation des q, qpoint, q2points
if(t <= tacc2) {
							q2 = (0.5*(theta2f-theta2i)*t*t)/(tacc2*(ttot-tacc2)) + theta2i;
							qdot2 = di2*acc2*t;
							qddot2 = di2*acc2;
						}
	else if(t>= tacc2 && t <= (ttot-tacc2)) {
									q2 = ((theta2f-theta2i)*(t-tacc2/2))/(ttot-tacc2) + theta2i;
									qdot2 = di2*vit2;
									qddot2 = 0.0;
									}
					else if(t>= (ttot-tacc2) && t < ttot) {
														q2 = (-0.5*(theta2f-theta2i)*(t-ttot)*(t-ttot))/(tacc2*(ttot-tacc2)) + theta2f;
														qdot2 = di2*acc2*(ttot-t);
														qddot2 = -di2*acc2;
														}
									else	{q2=theta2f;
												qdot2=0.0; 
												qddot2=0.0;}


/***********************************/
/* generation de trajectoire axe 4 */
/***********************************/

	if((theta4f - theta4i) < 0.0)	di4 = -1;//sens de rotation

		else di4 = 1;

//generation des q, qpoint, q2points
if(t <= tacc4) {
							q4 =(0.5*(theta4f-theta4i)*t*t)/(tacc4*(ttot-tacc4)) + theta4i;
							qdot4 = di4*acc4*t;
							qddot4 = di4*acc4;
						}
	else if (t>= tacc4 && t <= (ttot-tacc4)) {
									q4 = ((theta4f-theta4i)*(t-tacc4/2))/(ttot-tacc4) + theta4i;
									qdot4 = di4*vit4;
									qddot4 = 0.0;
									}
					else if (t>= (ttot-tacc2) && t < ttot) {
									q4 = (-0.5*(theta4f-theta4i)*(t-ttot)*(t-ttot))/(tacc4*(ttot-tacc4)) + theta4f;
									qdot4 = di4*acc4*(ttot-t);
									qddot4 = -di4*acc4;
														}
									else {q4=theta4f;
												qdot4=0.0;
												qddot4=0.0;}

}//FIN else t<>0
/********************/
// enregistrement des trajectoires
angle[0][i] = q2; angle[1][i] = q4;
vitang[0][i] = qdot2; vitang[1][i] = qdot4;
accang[0][i] = qddot2; accang[1][i] = qddot4;

	x = l1 * cos(q2) + l2 * cos(q2+q4);
	y = l1 * sin(q2) + l2 * sin(q2+q4);

X[i] = x; Y[i] = y;
}//FIN i (enregistrement traj)

for(int i=NB_POINTS; i<1000; i++)	{

releveTemporel[i] = i*tau;
X[i]=x; Y[i]=y;

vitang[0][i] = 0.0;
vitang[1][i] = 0.0;
accang[0][i] = 0.0;
accang[1][i] = 0.0;} //complement matlab




/***********************************/
/**** fin de calcul de trajectoire */
/***********************************/
	printf("\nENREGISTRER LES DONNEES DANS MATLAB?\n\n");
	printf("\nENTRER 0 POUR OUI, AUTRE POUR NON.\n");	
/*	int*/ validation	=	1;
	scanf("%d",&validation);
	if(validation==0){
	printf("\n\nEntrer le nom de fichier\n");
	scanf("%s",fich);	

	monFichier.sauvegarde_trajectoire(fich, releveTemporel,
		X, Y, angle, vitang, accang);
printf("\nOK. DANS MATLAB.\n");

		} //FIN validation


/***************************************/
/********* CHOIX DES CORRECTEURS ********/
/***************************************/

			printf("\nCHOISIR LA CORRECTION DES 2 AXES COMME TWISTING\n");


			printf("\n*** CORRECTION AXE 2 ***\n");
			demanderCorrecteur(controleurAxe2, correcteurPIDAxe2, correcteurTWAxe2, correcteurSTWAxe2);
			printf("\n*** CORRECTION AXE 4 ***\n");
			demanderCorrecteur(controleurAxe4, correcteurPIDAxe4, correcteurTWAxe4, correcteurSTWAxe4);

	if(controleurAxe4->getCorrecteur()->isTW()) {
suiviTW(controleurAxe2, controleurAxe4, capteurAxe[1], capteurAxe[3],
						actionneurAxe[1], actionneurAxe[3],NB_POINTS,
						releveTemporel,	relevePression, relevePression2, angle, vitang, accang,
						releveThetaDesiree, releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
						releveErreur, releveErreur2, releveUeq, releveUeq2, releveUdelta, releveUdelta2,
						releveSurFace, releveSurFace2, releveSurfaceDot, releveSurfaceDot2);}


else if (controleurAxe4->getCorrecteur()->isPID()) {
suiviPID(controleurAxe2, controleurAxe4, capteurAxe[1], capteurAxe[3],
						actionneurAxe[1], actionneurAxe[3], NB_POINTS, angle, vitang, accang,
						releveTemporel,	relevePression, relevePression2,
						releveThetaDesiree, releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
						releveErreur, releveErreur2);}


	break;}//FIN Case 10

/**************************/
		case	11:{
/************/


float posxi, posyi,posxf, posyf, theta2i, theta4i, q2, q4, tacc, ttot;
float theta2d, theta4d;

double x,y,l,b,c,d;
float t,normvit,normacc;

int T_suivi = 50; //50ms pour le suivi
float	tau	=	(Duree(T_suivi,MS)).toSecondes();

nbraxe = 2;

printf("\n A partir de deux positions cartesiennes dans le plan vertical,\n");
printf("celle de depart et celle d'arrivee, les variables articulaires\n");
printf("sont generees. Les trajectoires des axes 2 et 4 s'effectuent\n");
printf("de la position de depart a celle d'arrivee avec des normes de vitesse et\n");
printf("acceleration desirees.\n");
/**********************************/
/* lecture des positions initiales */
q2 = (float) (capteurAxe[1]->lirePositionRAD());
q4 = (float) (capteurAxe[3]->lirePositionRAD());
theta2i = q2; theta4i = q4;
mgd(q2,q4);	//MODELE GEOMETRIQUE DIRECT

angle[0][0] = theta2i; angle[1][0] = theta4i;
vitang[0][0] = 0.0f; vitang[1][0] = 0.0f;
accang[0][0] = 0.0f; accang[1][0] = 0.0f;

posxi = l1 * cos(q2) + l2 * cos(q2+q4);
posyi = l1 * sin(q2) + l2 * sin(q2+q4);

X[0] = posxi; Y[0] = posyi;
releveTemporel[0] = 0.0;
/**********************************/
//entree des positions de depart et arrivee
/*
printf("\nEntrer la position de depart:\n");
printf("X=\n");
scanf("%f",&posxi);
printf("Y=\n");
scanf("%f",&posyi);
X[0] = posxi;
Y[0] = posyi;
/****************************/
/* modele geometrique inverse */
/******************************/
/*
x=posxi; y=posyi;
	l = x*x + y*y;
	b = l1*l1;
	c = l2*l2;
	d = sqrt((l+b+c)*(l+b+c) - 2*(l*l + b*b + c*c));
	q2= atan2(y,x) - atan2(d,(l+b-c));
	q4= atan2(d,(l-b-c));

angle[0][0] = q2; angle[1][0] = q4;
theta_desiree2 = q2; theta_desiree4 = q4;
vitang[0][0] = 0.0;
vitang[1][0] = 0.0;
accang[0][0] = 0.0;
accang[1][0] = 0.0;
releveTemporel[0] = 0.0;
/***************************/
printf("\nEntrer la position d'arrivee:\n");
printf("X=\n");
scanf("%f",&posxf);
printf("Y=\n");
scanf("%f",&posyf);
/************************/
printf("\nlongueur du segment:");
printf("\n%f",sqrt(((posxf-posxi)*(posxf-posxi))+((posyf-posyi)*(posyf-posyi))));


/****************************/
//entree des normes de vitesse et acceleration
printf("\nEntrer la norme de la vitesse maximale\n");
scanf("%f",&normvit);
printf("\nEntrer la norme de l'acceleration maximale\n");
scanf("%f",&normacc);
/************************************/
/* trajectoire supposee trapezoidale */

//determination du temps d'acceleration
tacc=normvit/normacc;
printf("temps d'acceleration = %f\n", tacc);

//determination du temps total
ttot=(sqrt((posxf-posxi)*(posxf-posxi) + (posyf-posyi)*(posyf-posyi))/normvit) + tacc;
printf("temps total de poursuite = %f\n", ttot);


/****************************************************/
//generation des trajectoires
/*******************************/
NB_POINTS = int (ttot/tau +1.0);
printf("nbre de points est :%d\n",NB_POINTS);

	int validation	=	0;
	while (validation!=1) {
	printf("\nENTRER 1 POUR CONTINUER.\n");	
	scanf("%d",&validation);
} //end validation


/*****************************************/
//generation de la trajectoire cartesienne
///////////////////////////////////////////
for(int i=1; i<NB_POINTS; i++)	{
/*******************/
t= i*tau; //temps

if(t <= tacc) {
							x = (0.5* (posxf-posxi)*t*t)/(tacc*(ttot-tacc)) + posxi;
							y = (0.5* (posyf-posyi)*t*t)/(tacc*(ttot-tacc)) + posyi;
							}
	else if((t>=tacc) && (t <= (ttot-tacc))) {
									x = ((posxf-posxi)*(t-(0.5*tacc)))/(ttot-tacc) + posxi;
									y = ((posyf-posyi)*(t-(0.5*tacc)))/(ttot-tacc) + posyi;
																}
					else if((t>=(ttot-tacc)) && (t < ttot)) {
									x = -(0.5*(posxf-posxi)*(t-ttot)*(t-ttot))/(tacc*(ttot-tacc)) + posxf;
									y = -(0.5*(posyf-posyi)*(t-ttot)*(t-ttot))/(tacc*(ttot-tacc)) + posyf;
														}
									else	{
												x = posxf; 
												y = posyf;}

X[i] = x; Y[i] = y;

/********************/
/******************************/
/* modele geometrique inverse */
/******************************/

	l = x*x + y*y;
	b = l1*l1;
	c = l2*l2;
	d = sqrt((l+b+c)*(l+b+c) - 2*(l*l + b*b + c*c));
	q2= atan2(y,x) - atan2(d,(l+b-c));
	q4= atan2(d,(l-b-c));

//if((q4<= AXE_4_MAX && q4>= AXE_4_MIN) && (q2<= AXE_2_MAX && q2>= AXE_2_MIN)) choice=true;

//modele geometrique inverse pour determiner q2 et q4
//mgi(x,y);


/** enregistrement des trajectoires **/
releveTemporel[i] = t;

angle[0][i] = q2; angle[1][i] = q4;
vitang[0][i] = (angle[0][i]-angle[0][i-1])/tau;
vitang[1][i] = (angle[1][i]-angle[1][i-1])/tau;
accang[0][i] = (vitang[0][i]-vitang[0][i-1])/tau;
accang[1][i] = (vitang[1][i]-vitang[1][i-1])/tau;

}//FIN i (enregistrement traj)

printf("\nAngles finaux:");
printf("Axe 2:%f	Axe 4:%f",q2,q4);

for(int i=NB_POINTS; i<1000; i++)	{
releveTemporel[i] = i*tau;
X[i]=x; Y[i]=y;
angle[0][i] = q2; angle[1][i] = q4;
vitang[0][i] = 0.0;
vitang[1][i] = 0.0;
accang[0][i] = 0.0;
accang[1][i] = 0.0;} //complement matlab

/***********************************/
/**** fin de calcul de trajectoire */
/***********************************/
	printf("\nENREGISTRER LES DONNEES DANS MATLAB!\n\n");
/*	printf("\nENTRER 0 POUR OUI, AUTRE POUR NON.\n");	
	int validation	=	1;
	scanf("%d",&validation);
if(validation==0){
*/
	printf("\n\nEntrer le nom de fichier\n");
	scanf("%s",fich);	

	monFichier.sauvegarde_trajectoire(fich, releveTemporel,
	X,Y,angle, vitang, accang);
printf("\nOK. DANS MATLAB.\n");

//		} //FIN validation (enregistrement dans Matlab


/***************************************/
/********* CHOIX DES CORRECTEURS ********/
/***************************************/

			printf("\nCHOISIR LA CORRECTION DES 2 AXES COMME TWISTING Pour le moment!");


			printf("\n*** CORRECTION AXE 2 ***\n");
			demanderCorrecteur(controleurAxe2, correcteurPIDAxe2, correcteurTWAxe2, correcteurSTWAxe2);
			printf("\n*** CORRECTION AXE 4 ***\n");
			demanderCorrecteur(controleurAxe4, correcteurPIDAxe4, correcteurTWAxe4, correcteurSTWAxe4);



	if(controleurAxe4->getCorrecteur()->isTW()) {
suiviTW(controleurAxe2, controleurAxe4, capteurAxe[1], capteurAxe[3],
						actionneurAxe[1], actionneurAxe[3], NB_POINTS,
						releveTemporel,	relevePression, relevePression2, angle, vitang, accang,
						releveThetaDesiree, releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
						releveErreur, releveErreur2, releveUeq, releveUeq2, releveUdelta, releveUdelta2,
						releveSurFace, releveSurFace2, releveSurfaceDot, releveSurfaceDot2);}


else if(controleurAxe4->getCorrecteur()->isPID()) {
suiviPID(controleurAxe2, controleurAxe4, capteurAxe[1], capteurAxe[3],
						actionneurAxe[1], actionneurAxe[3], NB_POINTS, angle, vitang, accang,
						releveTemporel,	relevePression, relevePression2,
						releveThetaDesiree, releveThetaDesiree2, releveThetaMesuree, releveThetaMesuree2,
						releveErreur, releveErreur2);}


	break;}//FIN Case 11


	} //FIN switch


printf("\n\nPOUR QUITTER DEFINITIVEMENT ENTRER 0.\n");
printf("\nPOUR REFAIRE UN TEST ENTRER UNE AUTRE VALEUR.\n");
scanf("%d",&numeroMenu);
if(numeroMenu==0)	refaire	=	false;


}	//FIN while refaire and sortie

//DEGONFLER

printf("DEGONFLEMENT DES MUSCLES\n");
printf("PATIENCE \n");

					for	(incrementation=0; incrementation<NB_AXES ; incrementation++ )

						degonfleMuscle((int) (actionneurAxe[incrementation]));


printf("\nFIN DE LA MANIP.\n");

}	//FIN programmePrincipal

/************************/
/*		page d'accueil		*/
/************************/

void	afficherPageAccueil()

{
	printf("\nCETTE MANIP. PERMET DE COMMANDER LES AXES EPAULE ET COUDE.\n");
	printf("\nCEUX CI SONT RESPECTIVEMENT NOTES 2 ET 4.\n");
	printf("\nLA COMMANDE PEUT SE FAIRE SOIT EN STABILISATION (REGULATION),\n");
	printf("\nSOIT EN SUIVI DE TRAJECTOIRE.\n");
	printf("\nTROIS ALGORITHMES DE COMMANDE SONT PROPOSES:\n");
	printf("\nUN PID CLASSIQUE, UN TWIST , UN SUPER TWIST , UN TWIST+PI.\n");
	printf("\nLES DEUX DERNIERS SONT DES ALGORITHMES EN REGIME GLISSANT,\n");
	printf("\nD'ORDRE 2.\n");
	printf("\n\nPOUR QUITTER ENTRER 0.\n");
	printf("\nPOUR POURSUIVRE ENTRER UNE AUTRE VALEUR\n");

}	//FIN afficherPageAccueil

/**************/
/*	MAIN			*/
/**************/

void	main()

{
char debut;
	sysClkRateSet(1000);
	afficherPageAccueil();
	scanf("%c",&debut);
	if(debut!='0'){
								programmePrincipal();
							}
	finProgramme();
}	//FIN main
