#ifndef CAPTEURDEPOSITION_H_HEADER_INCLUDED_BF72E6A2
#define CAPTEURDEPOSITION_H_HEADER_INCLUDED_BF72E6A2

#include "BibliothequeTR\Temps.h"

class InterfaceCarteDEntreeAnalogique;

// Le capteur de position permet de convertir la tension lue sur la carte
// d'entrée analogique en information de position. Il nécessite un étalonnage
// afin de régler l'offset et la pente nécessaire au calcul.
//##ModelId=4073C0060179
class CapteurDePosition
{
  public:
    //##ModelId=4087CFEB003B
    CapteurDePosition(InterfaceCarteDEntreeAnalogique* pCarteDEntreeAnalogique, char voie, float pente = 1.0, float offset = 0.0);
    
    
    //##ModelId=4087CFDC02D9
    virtual ~CapteurDePosition();

    // Methode renvoyant la position en degrés de l'axe
    //##ModelId=407CF4070033
    float lirePosition() const;

		float lirePositionRAD() const;

    float lirePositionRAD2(float,float,float,float,float,float) ;  // karim ajout 
    //void setPosition(float xxx); // karim ajout
    
    //##ModelId=4098F6210095
    void setZero();

    //##ModelId=4098F62100BD
    void setPente(float left);
    //##ModelId=40A09D8E0249
    void setOffset(float left);
		
	  float getOffset();
	  float getPente();
    //float getPosition(); // ajout karim


  protected:
    // La validité est un paramètre utilisé pour indiquer la période pendant
    // laquelle l'information de position va être considérée comme vraie. Ceci
    // est utilisé afin de réduire le nombre de lectures sur la carte
    // d'acquisition.
    //##ModelId=407D25A50320
//    mutable Date _validite;

    //##ModelId=407CF0DD03BD
    InterfaceCarteDEntreeAnalogique *_pCarteDEntreeAnalogique;
    //##ModelId=408E7E29021A
//    mutable float _dernierePosition;

  private:
    // L'offset indique le décalage utilisé pour déterminer la position zéro de
    // l'axe.
    //##ModelId=407CF4170054
    float _offset;
		float	_offset2;
    float _positionk;

    float *ad; //ajout karim le 18/07/2008
    
    // La pente indique le rapport à calculer pour mettre en forme la valeur en
    // V lue sur la carte d'entrée en degrés de rotation de l'axe.
    //##ModelId=407CF41C0155
    float _pente;

    // La voie indique sur quelle entrée de la carte le capteur est branché.
    //##ModelId=4084C7B4037C
    char _voie;
};



#endif /* CAPTEURDEPOSITION_H_HEADER_INCLUDED_BF72E6A2 */