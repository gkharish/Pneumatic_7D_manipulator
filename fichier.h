/****************************************
 * Fichier fichier.h            	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef FICHIER
#define FICHIER 
 
 ///---------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
//#include <dos.h>
#include <string.h>
#include <time.h>

/********************************************************************
 *                          Classe fichier                          *
 ********************************************************************
 *                                                                  *
 *    gere la sauvegarde des donnees sous format matlab             *
 *                						    *
 *		  		        			    *
 ********************************************************************/
 

class fichier {
  
  public:
    
    //Constructeurs
    fichier (){}
    fichier(char*);
    
    //Sauvegarde sous format matlab
    void sauvegarde_matlab(char *,double*,double **,double **,int,int);

//////////////////////////////////////////////////////////////////////////////////////    
void sauvegarde_trajectoire(char *,	//nom du fichier
														double *,	//releve du temps
														double *,	//X
														double *,	//Y
														double **,	//positions angulaires desirees
														double **,	//vitesses angukaires desirees
														double **);	//accelerations angulaires desirees


void sauvegarde_identification(char *,    //nom du fichier MATLAB
                                   int,       //numero d'axe
                                   float,     //echelon de pression
															double *,  //releve du temps
															double *,  //releve du theta
															double *); //releve de la pression repos PLUS pression delta

void sauvegarde_identification2(char *,    //nom du fichier MATLAB
															double *,  //releve du temps
															double *,  //releve de la pression2
															double *,	//releve de la pression4
															double *,	//releve de la mesure d'angle2
															double *); //releve de la mesure d'angle4



void sauvegarde_PID( char  *,  //nom du fichier MATLAB
                         int,    //numero d'axe
												double *, //releve du temps
												double *, //releve du delta pression
												double *, //releve du theta desiree
												double *, //releve du theta mesuree
												double *, //releve de l'erreur
                         float,    //le coefficient P
                         float,    //le coefficient I
                         float); //le coefficient D

void sauvegarde_PID2( char  *,  //nom du fichier MATLAB
												double *, //releve du temps
												double *, //releve du delta pression
												double *, //releve du delta pression2
												double *, //releve du theta desiree
												double *, //releve du theta desiree2
												double *, //releve du theta mesuree
												double *, //releve du theta mesuree2
												double *, //releve de l'erreur
												double *, //releve de l'erreur2 
												 float,    //le coefficient P
                         float,    //le coefficient I
                         float, //le coefficient D
                         float,    //le coefficient P2
                         float,    //le coefficient I2
                         float); //le coefficient D2



void sauvegarde_TW(char   *, //nom du fichier MATLAB
                       int,      //numero d'axe
						double *, //releve du temps =================== (milliseconds)
						double *, //releve de la pression ============= U      (bars)
						double *, //releve de la commande equivalente = Ueq    (bars)
						double *, //releve de la commande delta ======= Udelta (bars)
						double *, //releve de la surface
						double *, //releve de surface point
						double *, //releve du theta mesuree =========== (radians)
						double *, //releve du theta desiree =========== (radians
						double *, //releve de l'erreur ================ (radians)
                       float,    //C     : parametre de la SV
                       float,			//ULIM
                       float,		//AMIN
											 float);		//AMAX


void sauvegarde_TW2(char   *, //nom du fichier MATLAB
						double *, //releve du temps =================== (milliseconds)
						double *, //releve de la pression ============= U      (bars)
						double *, //releve de la pression2 ============= U      (bars)
						double *, //releve de theta desiree
						double *, //releve de theta desiree2
						double *, //releve de theta mesuree
						double *, //releve de theta mesuree2	
						double *, //releve de erreur
						double *, //releve de erreur2
						double *, //releve de la commande equivalente = Ueq    (bars)
						double *, //releve de la commande equivalente = Ueq2    (bars)	
						double *, //releve de la commande delta ======= Udelta (bars)
						double *, //releve de la commande delta ======= Udelta2 (bars)

/*						double *, //releve de vitesse desiree
						double *, //releve de vitesse desiree2	
						double *, //releve de vitesse mesuree
						double *, //releve de vitesse mesuree2 
*/					
						double *, //releve de surface
						double *, //releve de surface2
						double *, //releve de surfacedot
						double *, //releve de surfacedot2 
                       float,    //C     : parametre de la SV
                       float,			//ULIM
                       float,		//AMIN
											 float,		//AMAX
											float,	//ALPHA Accelerateur de conv
											float,	//ATTENUATOR attenuateur de ueq
											 float,	//C2
											 float,	//ULIM2
											 float,	//AMIN2
											 float,		//AMAX2
												float,	//ALPHA2
												float);	//ATTENUATOR2



void sauvegarde_STW(char   *, //nom du fichier MATLAB
                       int,      //numero d'axe
							double *, //releve du temps =================== (milliseconds)
							double *, //releve de la pression ============= U      (bars)
							double *, //releve de la commande equivalente = Ueq    (bars)
							double *, //releve de la commande delta ======= Udelta (bars)
							double *, //releve de la surface
							double *, //releve de surface point
							double *, //releve du theta mesuree =========== (radians)
							double *, //releve du theta desiree =========== (radians)
							double *, //releve de l'erreur ================ (radians)
                       float,    //C     : parametre de la SV
                       float,			//ULIM
                       float,		//W
											 float,		//LAMBDA
											 float);		//S0


void sauvegarde_STW2(char   *, //nom du fichier MATLAB
							double *, //releve du temps =================== (milliseconds)
							double *, //releve de la pression ============= U      (bars)
							double *, //releve de la pression2 ============= U      (bars)
							double *, //releve de la commande equivalente = Ueq    (bars)
							double *, //releve de la commande equivalente = Ueq2    (bars)	
							double *, //releve de la commande delta ======= Udelta (bars)
							double *, //releve de la commande delta ======= Udelta2 (bars)
							double *, //releve de la surface
							double *, //releve de la surface2
							double *, //releve de surface point
							double *, //releve de surface point2	
							double *, //releve du theta mesuree =========== (radians)
							double *, //releve du theta mesuree2 =========== (radians)	
							double *, //releve du theta desiree =========== (radians)
							double *, //releve du theta desiree2 =========== (radians)
							double *, //releve de l'erreur ================ (radians)
							double *, //releve de l'erreur2 ================ (radians)
                       float,    //C     : parametre de la SV
                       float,			//ULIM
											 float,		//W
											 float,		//LAMBDA
											 float,		//S0
                       float,		//C2
											 float,		//ULIM2
											 float,	//W2
											 float,	//LAMBDA2
											 float);	//S02

// ******************************************************************************************************************************//
// ***********cette partie a ete ajoutee par Amar REZOUG le 12-04-2012 pour sauvgarder les resultats de TSMC*************************//
//*********************************************************************************************************************************//
/*
void sauvegarde_TSMC(char   *, //nom du fichier MATLAB
                       int,      //numero d'axe
							double *, //releve du temps =================== (milliseconds)
							double *, //releve de la pression ============= U      (bars)
							double *, //releve de la commande equivalente = Ueq    (bars)
							double *, //releve de la commande delta ======= Udelta (bars)
							double *, //releve de la surface
							double *, //releve de surface point
							double *, //releve du theta mesuree =========== (radians)
							double *, //releve du theta desiree =========== (radians)
							double *, //releve de l'erreur ================ (radians)
                       float,    //q     : parametre de la SV
                       float,			//p
                       float,		//Mbar
											 float,		//LAMBDA
											 float);		//KWs

*/
// **************************************fin de la partie ajoutee par Amar REZOUG pour le sauvegarde des results du TSMC*************//
// **********************************************************************************************************************************//











/*

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
    void sauvegarde_identification(char *,    //nom du fichier MATLAB
                                   double *,  //position initial de chaque articulation du robot
                                   int,       //numero d'axe
                                   int,       //periode d'echantionnage
                                   int,       //numero de points
                                   float,     //echelon
                                   double *,  //releve du temps
                                   double *,  //releve du theta
                                   double *,  //releve du muscle1
                                   double *,  //releve du muscle2
                                   double *,  //releve de la pression de base
                                   double *,  //releve de la pression de repos
                                   double *,  //releve de la pression delta
                                   double *); //releve de la pression repos PLUS pression delta

    void sauvegarde_identification(char *,    //nom du fichier MATLAB
                                   double *,  //position initial de chaque articulation du robot
                                   int,       //numero d'axe
                                   int,       //periode d'echantionnage
                                   int,       //numero de points
                                   float,     //creneau1
                                   float,     //creneau2
                                   double *,  //releve du temps
                                   double *,  //releve du theta
                                   double *,  //releve du muscle1
                                   double *,  //releve du muscle2
                                   double *,  //releve de la pression de base
                                   double *,  //releve de la pression de repos
                                   double *,  //releve de la pression delta
                                   double *,  //releve de la pression repos PLUS pression delta
                                   double **);
    
    void sauvegarde_PID( char  *,  //nom du fichier MATLAB
                         double *, //position initial de chaque articulation
                         int,    //numero d'axe
                         int,    //periode d'echantillonage
                         int,    //nombre de points
                         double *, //releve du temps
                         double *, //releve du delta pression
                         double *, //releve du theta mesuree
                         double *, //releve de vitesse mesuree
                         double *, //releve du theta desiree
                         double *, //releve de l'erreur
                         double *, //releve de m1
                         double *, //releve de m2
                         float,    //le coefficient P
                         float,    //le coefficient I
                         float,    //le coefficient D
                         double **); //positions de tts les axes

    
    void sauvegarde_SV(char   *, //nom du fichier MATLAB
                       double *, //position initial de chaque articulation
                       int,      //numero d'axe
                       int,      //periode d'echantionnage
                       int,      //numero de points
                       double *, //releve du temps =================== (milliseconds)
                       double *, //releve de la pression ============= U      (bars)
                       double *, //releve de la commande equivalente = Ueq    (bars)
                       double *, //releve de la commande delta ======= Udelta (bars)
                       double *, //releve du muscle1
                       double *, //releve du muscle2
                       double *, //releve du theta mesuree =========== (radians)
                       double *, //releve du theta desiree =========== (radians)
                       double *, //releve de la vitesse mesuree ====== (radians/s)
                       double *, //releve de l'erreur ================ (radians)
                       double *, //releve de la vitesse de l'erreur == (radians/s)
                       double *,
                       float,    //C     : parametre de la SV
                       float,
                       float,
                       double **);
*/
/*
,    //K : parametre de la SV
                       double,   //A1    : parametre de la SV
                       double,   //A2    : parametre de la SV
                       double);  //B     : parametre de la SV
*/


    void renommer (char *);			   
    
    
  private:
   void Sauve_Format_MATLAB(FILE *, long,char*,long,long,long,double*,float*);
    char * acces;     //chemin d'acces au fichier
};

#endif