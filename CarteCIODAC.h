#ifndef CARTECIODAC16_H_HEADER_INCLUDED_BF69B8A5
#define CARTECIODAC16_H_HEADER_INCLUDED_BF69B8A5

#include "InterfaceCarteDeSortieAnalogique.h"

//##ModelId=407CF10D02D6
class CarteCIODAC16 : public InterfaceCarteDeSortieAnalogique
{
  public:
    //##ModelId=409666C60385
    CarteCIODAC16(int adresseDeBase);

    //##ModelId=409666C60386
    virtual ~CarteCIODAC16();

    //##ModelId=407CF3D00174
    bool daConv(char voie, float valeur);
    
  private:
    // D/A LSB
    //##ModelId=409F2AE9013C
    int _adresseBase0;


    // D/A MSB & channel address
    //##ModelId=409F2AF10238
    int _adresseBase1;

};

#endif /* CARTECIODAC16_H_HEADER_INCLUDED_BF69B8A5 */
