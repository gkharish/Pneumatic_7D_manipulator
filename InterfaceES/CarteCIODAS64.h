/* refCarte : CIO-DAS6402/12 */

#ifndef CARTECIODAS64_H_HEADER_INCLUDED_BF69E024
#define CARTECIODAS64_H_HEADER_INCLUDED_BF69E024

#include "InterfaceCarteDEntreeAnalogique.h"
#include "InterfaceCarteDEntreeNumerique.h"
#include "InterfaceCarteDeSortieNumerique.h"
#include "InterfaceCarteDeSortieAnalogique.h"

/******************************************************************************/
/*        PARAMETRES DE CONFIGURATION                                         */
/******************************************************************************/
#define UNIPOLAR 0


//##ModelId=407CF116027F
class CarteCIODAS64 : public InterfaceCarteDEntreeAnalogique, public InterfaceCarteDEntreeNumerique, public InterfaceCarteDeSortieNumerique, public InterfaceCarteDeSortieAnalogique
{
  public:
      //##ModelId=409666CE03A5
    CarteCIODAS64(int adresseDeBase);

    //##ModelId=409666CE03A6
    virtual ~CarteCIODAS64();
  
    //##ModelId=407CF3490365
    bool ecrireEtatVoie(char voie, char etat);

    //##ModelId=407CF352032B
    char lireEtatVoie(char voie) const;

    //##ModelId=407CF35A01C4
    float adConv(char voie) const;
    
    //##ModelId=40978D1402F1
    bool daConv(char voie, float valeur);

    
  private:
    //##ModelId=409F2CD50342
    int _adresseDeBase;
    
    //##ModelId=40973ECF02B0
    UINT8 _derniereSortieNumerique;
    
    //##ModelId=409766FF023B
    char _analogInputPolarity;

    //##ModelId=409766EA0303
    char _analogInputRange;

};

#endif /* CARTECIODAS64_H_HEADER_INCLUDED_BF69E024 */
