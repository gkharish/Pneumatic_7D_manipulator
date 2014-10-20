#include "CarteCIODAC16.h"
/* refCarte : CIO-DAC16-I de Computer boards */

/*
Historique des modifications
----------------------------

*/

/******************************************************************************/
/*                    DEFINITION DES CONSTANTES                               */
/******************************************************************************/
#define OFFSET_CIODAC16 (4.0)  /* Utilisés pour la conversion 4-20mA -> mot de 12 bits */
#define PENTE_CIODAC16 (255.94)   /* Offset = Imin ; Pente = 4095 / (Imax - Imin) */


//##ModelId=409666C60385
CarteCIODAC16::CarteCIODAC16(int adresseDeBase)
{
	int i;
	
	_adresseBase0 = adresseDeBase;
	_adresseBase1 = adresseDeBase + 1;
	
	for(i=0; i<16; i++)
		daConv(i, OFFSET_CIODAC16);
}


//##ModelId=409666C60386
CarteCIODAC16::~CarteCIODAC16()
{
}


/******************************************************************************/
/*                                                                            */
/*		CONVERSION NUMERIQUE -> ANALOGIQUE			                          */
/*                                                                            */
/******************************************************************************/
/*                                                                            */
/*		Methode :						                                      */
/*                                                                            */
/*	1 - Decoupage des quatres bits de poids fort de la valeur	              */
/*	2 - Creation du mot de 8 bits contenant l'adresse(msb) et (1)	          */
/*	3 - Ecriture de 8 bits de poids faible de la valeur dans BASE_0	          */
/*	4 - Ecriture du mot de 8 bits dans BASE_1			                      */
/*                                                                            */
/******************************************************************************/
 
//##ModelId=407CF3D00174
bool CarteCIODAC16::daConv(char voie, float valeur)
{
	UINT16 buffer;
	
	// Vérification des données par saturation
	// AMELIORATION POSSIBLE : GESTION PAR EXCEPTION OU ERREUR PLUS PERFORMANTE
	if (valeur > 20.0) valeur = 20.0;
	if (valeur < 4.0) valeur = 4.0;
	
	// Conversion 4-20mA -> mot de 12 bits
	buffer = (UINT16) ((valeur - OFFSET_CIODAC16)* PENTE_CIODAC16);

	/* Passage en section critique */
	taskLock();
	
	// ecriture des 8 bits de poids faible
	sysOutByte (_adresseBase0,(UINT8) (buffer & 0xff));
	
	// prise des 8 bits de poids fort de valeur
	// filtrage pour n'avoir que les 4 bits de poids faible
	buffer = (buffer>>8) & 0x0f;
	// on récupére les 4 bits de poids faible de chan (contenant l'info)
	// que l'on décale de 4 bits pour l'additionner avec buffer
	buffer = ((voie<<4) & 0xf0) | buffer ;
	
	// ecriture des 4 derniers bits et lancement de la conversion
	sysOutByte (_adresseBase1, (UINT8) buffer);
	
	/* Fin de section critique */
	taskUnlock();
	
	return true;
}


