

/******************************************************************************/
/*				ADRESSES DES REGISTRES					                      */
/******************************************************************************/
#define AD_DATA_REG	(BASE_REG_CIODAS64)    // Registre des donnees et de
                                           // conversion
#define CHAN_LIMITS	(_adresseDeBase+2)  // Registre de reglage des adresses
#define DIGITAL_REG	(_adresseDeBase+3)  // Registre des donnees Numerique
#define DAC_OUT_0   (_adresseDeBase+4)  // Sortie Numerique DAC1
#define DAC_OUT_1   (_adresseDeBase+6)  // Sortie Numerique DAC2
#define STATUS_REG  (_adresseDeBase+8)  // Registre des status
#define PACER_CNTRL (_adresseDeBase+9)  // Control des Interruptions & Pacer
#define TRIG_CNTRL	(_adresseDeBase+10) // Registre de config du trigger
#define COMP_CNTRL  (_adresseDeBase+11) // Registre de config de la carte

#define COMPTEUR_0  (_adresseDeBase+12) // Registre du Compteur 0
#define COMPTEUR_1  (_adresseDeBase+13) // Registre du compteur 1
#define COMPTEUR_2	(_adresseDeBase+14) // Registre du compteur 2
#define COUNTER_CONTROL (_adresseDeBase+15) 	/* Registre de controle des
                                                   compteurs  	*/

/******************************************************************************/
/*				DETAILS DES REGISTRES                                         */
/******************************************************************************/

/******************************************************************************/
/*				REGISTRE DES STATUS                                           */
/******************************************************************************/
#define MHZ_1			0x00	  	// Reglage de clock pacer à 1 Mhz
#define MHZ_10			0x80 	  	// Reglage à 10 Mhz
#define POST_MODE 		0x40	  	// Post-mode
#define EXTEND			0x10		// Bit d'autorisation de configuration
#define CLRALL    		0x07	  	// efface toutes les interruptions
#define CLRINT			0x01	  	// efface les interruptions
#define CLRXTR			0x02	  	// efface les Triggers Externe
#define CLRXIN			0x04	  	// efface les interruptions externe


/******************************************************************************/
/*		REGISTRE DE CONTROL DES INTERRUPTIONS ET DU PACER                     */
/******************************************************************************/
#define NO_INTERRUPT	0x00		// Pas d'interruption
#define BURSTE_MODE		0x40	  	// mode Burste ON
#define INTE			0x80	  	// Interruptions analogiques autorisees
#define XINTE       	0x08		// Les Entrees Externes sont autorisees
/******************************************************************************/
/*              	  niveaux d'interruption 	                              */
/******************************************************************************/
#define NONE_LEVEL		0x00	  	// pas de niveau d'Interruption
#define LEVEL_11		0x10		// interrupt level = 11 in mode Eh.
#define LEVEL_2			0x20		// interrupt level = 2 in modes C/E
#define LEVEL_3			0x30		// interrupt level = 3 in modes C/E
#define LEVEL_10		0x40		// interrupt level = 4 in mode Eh.
#define LEVEL_5			0x50		// interrupt level = 5 in modes C/E
#define LEVEL_15  		0x60		// interrupt level = 15 in mode Eh.
#define LEVEL_6 		0x60		// interrupt level = 6 in mode Comp
#define LEVEL_7 		0x70		// interrupt level = 7 in modes E/C
/******************************************************************************/
/*  	                 	mode de conversion	                              */
/******************************************************************************/
#define SOFT_CONVERT	0x00		// conversion en Software
#define XFALL			0x01		// External Pacer Falling Edge
#define XRIS			0x02		// External Pacer Rising Edge
#define INT_PACER		0x02		// Internal Pacer
/******************************************************************************/
/**                  	specifique au mode COMPATIBLE 	                      */
/******************************************************************************/
#define DIN_IN			0x01		// Enable DIN0 input to gate the pacer
#define COUNT_CNTRL		0x02		// 100 khz en entreesur le compteur 0


/******************************************************************************/
/*		REGISTRE DE CONTROL DU TRIGGER ET REGLAGE DU NUMERIQUE		  	      */
/******************************************************************************/
#define OUT_5V			0x00 		/* Reglage à +- 5v les 2 sorties   	*/
#define OUT_10V_DAC0 	0x10		/* Reglage à +- 10v la sortie DAC0 	*/
#define OUT_0_5V_DAC0   0x20		/* Reglage de 0 à 5v la sortie DAC0	*/
#define OUT_0_10V_DAC0  0x30		/* Reglage de 0 à 5v la sortie DAC0	*/

#define OUT_10V_DAC1 	0x40		/* Reglage à +- 10v la sortie DAC1	*/
#define OUT_0_5V_DAC1  	0x80		/* Reglage de 0 à 5v la sortie DAC1	*/
#define OUT_0_10V_DAC1  0xC0		/* Reglage de 0 à 5v la sortie DAC1	*/

#define TG_EN 			0x01		/* Active la fonction Trigger/Gate 	*/
#define TG_SEL			0x03		/* Trigger/Gate select bit		*/
#define TG_POL			0x07		/* Trigger/Gate polarity bit		*/
#define PRETRIG			0x09		/* Stop pacing a certain number of conversion*/

/******************************************************************************/
/*			REGISTRE DE CONFIGURATION DE LA CARTE				              */
/******************************************************************************/
#define DMA_1			0x00		// Réglage sue le DMA 1
#define DMA_3			0x80		// Reglage sur le DMA 3
#define UNIPOLAR_RANGE  0x40		// MODE UNIPOLAIRE
#define BIPOLAR_RANGE   0x00		// MODE BIPOLAIRE
#define SE_MODE    		0x20		// mode Single-ended
#define DIFF_MODE		0x00		// mode differential
#define ENHANCED_MODE  	0x10	  	// mode Enhanced
#define COMPATIBLE_MODE 0x00		// mode compatible
#define IN_10V			0x00		// Entrée de 0 à 10 V ou +-10V
#define IN_5V			0x01		// Entrée de 0 à 5 V ou +- 5V
#define IN_2_5V			0x02		// Entrée de 0 à 2.5 V ou +-2.5V
#define IN_1_25V		0x03		// Entrée de 0 à 1.25 V ou +-1.25V


#include "CarteCIODAS64.h"

//##ModelId=409666CE03A5
CarteCIODAS64::CarteCIODAS64(int adresseDeBase)
{
	_adresseDeBase = adresseDeBase;
	_derniereSortieNumerique = 0;
	
	/******** Configuration du mode d'utilisation de la carte *******/
	// unipolar range, Single ended mode, ENHANCED MODE
	sysOutByte (COMP_CNTRL,ENHANCED_MODE);
	sysOutByte (COMP_CNTRL,(unsigned char)(ENHANCED_MODE|SE_MODE|UNIPOLAR_RANGE|IN_5V));

	/** Reglage de l'horloge interne au plus rapide ****/
	// passage en mode EXTEND
	sysOutByte (STATUS_REG,EXTEND);
	// reglage de la cadence
	sysOutByte (STATUS_REG,(unsigned char)(EXTEND|MHZ_10));
	// fin mode EXTEND
	sysOutByte (STATUS_REG,0x00);

	/******** Configuration du registre d'interruption et de control des Pacers */
	// Configuration sans interruption, sans bruste mode, avec une conversion A/D en sofware
	sysOutByte (PACER_CNTRL,NO_INTERRUPT);
	/******** Configuration du registre de control des triggers *****/
	// Reglage sur du +- 5V sur les deux sorties, Mode pre-trigger OFF
	sysOutByte (TRIG_CNTRL,OUT_5V);

	//printf("Carte Initialisée...!!\n");
}


//##ModelId=409666CE03A6
CarteCIODAS64::~CarteCIODAS64()
{
}

//##ModelId=407CF3490365
bool CarteCIODAS64::ecrireEtatVoie(char voie, char etat)
{
 	if (voie > 7)
  	{
   		printf("Numéro de voie invalide.");
     	return false;
	}

	// La carte CIODAS64 utilise un registre de 8 bits correspondant aux 8
	// sorties disponibles. Nous utilisons des masques afin de ne modifier
	// uniquement le bit de la sortie considérée.
	if (etat==1)
 		_derniereSortieNumerique |= (1<<etat); //Masque pour mise à 1 du bit
	else
 		_derniereSortieNumerique &= (~(1<<etat)); //Masque pour mise à 0 du bit

 	sysOutByte (DIGITAL_REG, _derniereSortieNumerique);
 	
	return true;
}

//##ModelId=407CF352032B
char CarteCIODAS64::lireEtatVoie(char voie) const
{
	if (voie > 7) printf("Numéro de voie invalide.");
	
 	return ((sysInByte (DIGITAL_REG) & (1<<voie)) > 0);
}



/******************************************************************************/
/*                                                                            */
/* 			CONVERSION ANALOGIQUE -> NUMERIQUE                                */
/* 				A PARTIR D'UNE ADRESSE		                                  */
/*                                                                            */
/******************************************************************************/
/*                                                                            */
/* 				ETAPE DE LA CONVERSION                                        */
/*                                                                            */
/* 			1 - Selection de la voie                                          */
/* 			2 - Lancement de la conversion                                    */
/* 			3 - Attente fin de conversion                                     */
/* 			4 - Lecture du resulat de la conversion                           */
/* 			6 - Mise en forme de la valeur                                    */
/*                                                                            */
/******************************************************************************/
//##ModelId=407CF35A01C4
float CarteCIODAS64::adConv(char voie) const
{
	float tension; 		// tension lue en V
	UINT16 mot;

	/* Passage en section critique */
	taskLock();

	/* Selection de la voie sur laquelle la CAN est demandée */
	mot = (voie<<8) | voie;
	sysOutWord (CHAN_LIMITS,mot);
		// Delai d'attente commutation multiplexeur

	/* Lancement de conversion */
	sysOutWord (AD_DATA_REG, 0);

	/* Attente fin de conversion */
	while ((sysInByte (STATUS_REG) & 0x01 )== 0);

	/* Lecture du resulat de la conversion */
	mot = sysInWord (AD_DATA_REG);
	mot = mot>>4;
	
	/* Mise en forme de la valeur */
	tension = (float)((mot)*5)/4095.0;
	/*switch (_analogInputRange)
	{
		case IN_5V :
			if (_analogInputPolarity == UNIPOLAR)
				tension = (float)((mot)*5)/4095.0;
			else
				tension = (float)((mot)*10)/4095.0 - 5.0;
			break;
		case IN_10V :
			if (_analogInputPolarity == UNIPOLAR)
				tension = (float)((mot)*10)/4095.0;
			else
				tension = (float)((mot)*20)/4095.0 - 10.0;
			break;
		case IN_2_5V :
			if (_analogInputPolarity == UNIPOLAR)
				tension = (float)(mot) * 2.5 / 4095.0;
			else
				tension = (float)((mot)*5)/4095.0 - 2.5;
			break;
		case IN_1_25V :
			if (_analogInputPolarity == UNIPOLAR)
				tension = (float)(mot)*1.25/4095.0;
			else
				tension = (float)((mot)*3)/4095.0 - 1.25;
			break;
		default : printf("Rémy tu fais chier merde !!");
	}*/
	
	/* Fin de section critique */
	taskUnlock();

	return tension;
}



//##ModelId=40978D1402F1
bool CarteCIODAS64::daConv(char voie, float valeur)
{
	return false; // En attendant l'implementation de la fonction
}

