#ifndef CONTROLEURDOUTIL_H_HEADER_INCLUDED_BF6F386A
#define CONTROLEURDOUTIL_H_HEADER_INCLUDED_BF6F386A

// On utilise la carte de sortie analogique CIODAC16 pour controler la pince
// mais la sortie est de type binaire NIVEAU HAUT et BAS correspondant aux
// sorties min et max de la carte soient 4 et 20 mA
#define NIVEAU_HAUT 20.0  
#define NIVEAU_BAS 4.0

class InterfaceCarteDeSortieAnalogique;

//##ModelId=407BE3060054
class OutilPince
{
  public:
    //##ModelId=4090B6C800AC
    OutilPince(InterfaceCarteDeSortieAnalogique* pCarteDeSortieAnalogique, char voiePince1, char voiePince2);

    //##ModelId=4088DC0400D5
    virtual ~OutilPince();

    //##ModelId=4088DF4E0252
    bool estOuverte() const;

    //##ModelId=4088DF4C0105
    bool estFermee() const;

    //##ModelId=407D4BFC0369
    bool fermer();

    //##ModelId=407D4C030138
    bool ouvrir();

  protected:
    //##ModelId=40851A5B0113
    bool _pinceOuverte;

    //##ModelId=407D54090128
    InterfaceCarteDeSortieAnalogique *_pCarteDeSortieAnalogique;

  private:
    //##ModelId=40851A020025
    char _voiePince1;

    //##ModelId=40851A0D01EE
    char _voiePince2;

};



#endif /* CONTROLEURDOUTIL_H_HEADER_INCLUDED_BF6F386A */
