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
// appel à un correcteur pour calculer la commande à envoyer à l'actionneur
// considéré.
// Il contient sa propre tâche d'asservissement dont on peut choisir la période
// d'échantillonnage.
class ControleurDAxe
{
  public:
    ControleurDAxe(Actionneur* pActionneur, Correcteur* pCorrecteur, float commandeAuRepos, float gainBoucleOuverte = 0.0);

    virtual ~ControleurDAxe();

    //bool envoyerConsigneDePosition(double position);

		bool envoyerConsigneDePosition(float position); //stabilisation
		bool envoyerConsigneDePosition(float position, float vitesse, float acceleration); //suivi de traj.

    bool mettreAJourCommande();

		
    // La méthode asservirAxe() lance la boucle d'asservissement.
    bool asservirAxe(const Duree& periodeDEchantillonnage);

    // La methode arreterAsservissement interrompt la boucle d'asservissement
    // et plus aucune commande n'est envoyée à l'actionneur.
    bool arreterAsservissement();
		bool tuerAsservissement();

    // La methode lireConsigne est un accesseur pour la variable
    // consigneDePosition effective du ControleurDAxe.
//double
		float consigne() const;

    // La methode lireCommande est un accesseur pour la variable commande
    // représentant la dernière valeur envoyée à l'actionneur.
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

    // La periode d'Echantillonnage correspond à la durée entre deux calculs de
    // commande successifs. Elle est exprimée en ms. Par défaut 10ms.
    Duree _periodeDEchantillonnage;

//double
		float _positionMin;

//double
		float _positionMax;

		float	_saturationMin;

		float	_saturationMax;

    // Commande envoyée à l'actionneur à l'état de repos (position 0).
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