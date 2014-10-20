#ifndef ACTIONNEUR_H_HEADER_INCLUDED_BF72924D
#define ACTIONNEUR_H_HEADER_INCLUDED_BF72924D

#define PRESSION_MAX (5.0)
#define PRESSION_MIN (0.0)
#define PRESSION_DE_BASE_MAX (3.0)
#define PRESSION_DE_BASE_MIN (2.0)
#define ECHELON_MAX (1.0)				//0.2	//ECHELON_MAX (1.0) limite la vitesse de gonflement

class InterfaceCarteDeSortieAnalogique;

// Cet actionneur est composé de deux muscles antagonistes ainsi que d'une
// liaison par chaine pour permettre le controle de l'angle d'une rotation du
// bras. L'actionneur 2 muscles utilise ici deux servovalves commandées en
// courant 4-20mA correspondant à 0-5 bar de pression.
//##ModelId=4073C0180355
class Actionneur
{
  public:
    //##ModelId=4087CB3A0070
    Actionneur(InterfaceCarteDeSortieAnalogique* pCarteDeSortieAnalogique, char voieMuscle1, char voieMuscle2, float pente = 3.2, float offset = 4.0);


    //##ModelId=4087CA4501B9
    virtual ~Actionneur();

    // La méthode envoyerCommande permet de controler les muscles à l'aide d'un
    // deltaP (cf. pressionDeBase).
    // Paramètre deltaP : différence de pression entre le muscle 1 et le muscle
    // 2 en bar
    // Paramètre pressionDeBase : en bar, valeur par défaut 2.5
    //##ModelId=407CF4280351
    bool envoyerCommande(float deltaP, float pressionDeBase = 2.5);
    
    bool envoyerCommande(float deltaP, float pressionActuelle1, float pressionActuelle2);

    // La méthode envoyerCommmandeDecouplee permet de controler la pression de
    // chaque muscle indépendament.
    // Paramètre pressionMuscle1 : entre 0 et 5 bars
    // Paramètre pressionMuscle2 : entre 0 et 5 bars
    //##ModelId=407CF85C0204
    bool envoyerCommandeDecouplee(float pressionMuscle1, float pressionMuscle2);

    // La methode lireCommande retourne la dernière commande envoyée à
    // l'actionneur.
    // Retour float : 
    //##ModelId=407D1E3F0164
    float pressionMuscle1() const;

	//##ModelId=408E79A101EA
    float pressionMuscle2() const;

    float pressionDeBase() const;
		float deltaPression() const;
    
    // La methode fixerPressionDeBase permet de modifier la valeur par défaut
    // de la pression de base.
    // Parametre pressionDeBase : valeur comprise entre 2 et 3 bars
    //##ModelId=4087659E008A
    bool setPressionDeBase(float pression);
    
    void setPressionRepos(float pression_repos);
    float getPressionRepos(void);
    
  protected:
    // La pression de base est utilisée lors d'une commande symétrique en
    // deltaP pour laquelle un muscle recoit pressionDeBase - deltaP et l'autre
    // pressionDeBase + deltaP. La valeur de ce paramètre peut agir sur la
    // rigidité de l'actionneur par exemple si pressionDeBase est supérieure à
    // la moitié de la pression maximum alors la tension exercée sur l'axe de
    // rotation sera supérieure par contre on réduit la course de l'actionneur.
    //##ModelId=4087657A0381
    float _pressionDeBase;

    // Pression "courante" du muscle1
    //##ModelId=408517AC002F
    float _pressionMuscle1;

    // Pression "courante" du muscle2
    //##ModelId=408517BA02E2
    float _pressionMuscle2;

    //##ModelId=407CF0DF0334
    InterfaceCarteDeSortieAnalogique *_pCarteDeSortieAnalogique;

  private:
    // L'offset indique le décalage à calculer pour mettre en forme la commande
    // de sortie.
    // Le décalage est de 4mA après correction de la pente.
    //##ModelId=407CF43002BD
    float _offset;

    // La pente indique le rapport à calculer pour mettre en forme la commande
    // de sortie. 
    // La pente est de (20-4)/(5-0) =  3.2 pour passer d'une consigne en bar à
    // la commande en mA
    //##ModelId=407CF44602E6
    float _pente;

    // voieMuscle1 indique quel canal de la carte de sortie utiliser pour
    // commander le muscle1.
    //##ModelId=408517C2003B
    char _voieMuscle1;

    // voieMuscle2 indique quel canal de la carte de sortie utiliser pour
    // commander le muscle2.
    //##ModelId=408517CF030B
    char _voieMuscle2;
    
    float _pression_repos;
};



#endif /* ACTIONNEUR_H_HEADER_INCLUDED_BF72924D */

