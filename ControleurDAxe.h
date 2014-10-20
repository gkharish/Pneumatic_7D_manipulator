#include "BibliothequeTR\HorlogeSynchronisee.h"
#include "BibliothequeTR\Tache.h"


#ifndef ControleurDAxe_H_HEADER_INCLUDED_BF72C705
#define ControleurDAxe_H_HEADER_INCLUDED_BF72C705

#define PRIORITE_CTRL_AXE 80

class Tache;
class Duree;
class Horloge;
class Correcteur;
class Actionneur;

// Un controleur d'axe permet l'asservissement en position d'un axe en faisant
// appel � un correcteur pour calculer la commande � envoyer � l'actionneur
// consid�r�.
// Il contient sa propre t�che d'asservissement dont on peut choisir la p�riode
// d'�chantillonnage.
class ControleurDAxe
{
  public:
    ControleurDAxe(Actionneur* pActionneur, Correcteur* pCorrecteur, float commandeAuRepos, float gainBoucleOuverte = 0.0);

    virtual ~ControleurDAxe();

    //bool envoyerConsigneDePosition(double position);

		bool envoyerConsigneDePosition(float position); //stabilisation
		bool envoyerConsigneDePosition(float position, float vitesse, float acceleration); //suivi de traj.

    bool mettreAJourCommande();

		
    // La m�thode asservirAxe() lance la boucle d'asservissement.
    bool asservirAxe(const Duree& periodeDEchantillonnage);

    // La methode arreterAsservissement interrompt la boucle d'asservissement
    // et plus aucune commande n'est envoy�e � l'actionneur.
    bool arreterAsservissement();
		bool tuerAsservissement();

    // La methode lireConsigne est un accesseur pour la variable
    // consigneDePosition effective du ControleurDAxe.
//double
		float consigne() const;

    // La methode lireCommande est un accesseur pour la variable commande
    // repr�sentant la derni�re valeur envoy�e � l'actionneur.
//double
		float commande() const;
    
    bool initialiser(float commandeRepos);

    bool setPositionMax(float left);

    bool setPositionMin(float left);

//double
		float getPositionMax() const;

//double
		float getPositionMin() const;

		bool	setSaturationMax(float satMax);

		bool	setSaturationMin(float satMin);

		float	getSaturationMax();

		float	getSaturationMin();
		
		float	getCommande();

    bool setBoucleOuverte(bool changer) ;

    bool getBoucleOuverte();
		
		void setCorrecteur(Correcteur* correcteur);
		Correcteur* getCorrecteur();
		
/*double*/ float getPressionRepos();
    
    
  protected:
    Horloge _horlogeDAsservissement;

    Tache _tacheDAsservissement;

    float _consigneDePosition;
		float _consigneDeVitesse;
		float _consigneDeAcceleration;

//double
		float _deltaCommande;

    Actionneur *_pActionneur;

    Correcteur *_pCorrecteur;
	
	
  private:
    bool procedureDAsservissement();

    bool procedureDInitialisation();

    bool _asservissementActif;

    // La periode d'Echantillonnage correspond � la dur�e entre deux calculs de
    // commande successifs. Elle est exprim�e en ms. Par d�faut 10ms.
    Duree _periodeDEchantillonnage;

//double
		float _positionMin;

//double
		float _positionMax;

		float	_saturationMin;

		float	_saturationMax;

    // Commande envoy�e � l'actionneur � l'�tat de repos (position 0).
//double
		float _commandeRepos;
//double
		float	_commandePrecedente;

//double
		float _gainBoucleOuverte;

    // Permet de verifier si le systeme est commande en boucle ouverte
    // ou en boucle ferme (utilistion ou non du correcteur
    bool _boucleOuverte;

		float	_commande;
};


#endif /* ControleurDAxe_H_HEADER_INCLUDED_BF72C705 */