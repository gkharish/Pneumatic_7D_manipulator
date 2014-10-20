#include <semLib.h>

// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#ifndef EVENEMENT_H_HEADER_INCLUDED_BF65DA54
#define EVENEMENT_H_HEADER_INCLUDED_BF65DA54

//#include <semLib.h>

//##ModelId=403B4A2100FD
class Evenement
{
  public:
    //##ModelId=403C62B50065
    Evenement();

    //##ModelId=403C62B5006F
    virtual ~Evenement();

    // Fonction suspendant la tache appelante jusqu'a ce que l'evenement soit
    // signale
    //##ModelId=403B4A7901CB
    bool attendre() const;

    //##ModelId=403B4A8402F4
    bool signaler();
    
    //##ModelId=409B597A018D
    bool signalerATous();


  private:
    //##ModelId=403B4B2D013E
    SEM_ID _id;

};



#endif /* EVENEMENT_H_HEADER_INCLUDED_BF65DA54 */
