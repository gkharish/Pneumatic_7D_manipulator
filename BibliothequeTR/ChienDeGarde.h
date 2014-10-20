#include "TypeProcedure.h"

// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#ifndef CHIENDEGARDE_H_HEADER_INCLUDED_BF65DE29
#define CHIENDEGARDE_H_HEADER_INCLUDED_BF65DE29

#include <wdLib.h>

//##ModelId=4084CD55020B
class ChienDeGarde
{
  public:
    //##ModelId=408D249903BE
    ChienDeGarde();

    //##ModelId=408D249903C8
    virtual ~ChienDeGarde();

    //##ModelId=4084CE3202C7
    bool demarrer(const Duree& temps, TypeProcedure procedure, int parametre = NULL);

    //##ModelId=4084CE3202C8
    bool arreter();

  private:
    //##ModelId=403B6FED031D
    WDOG_ID _id;

};



#endif /* CHIENDEGARDE_H_HEADER_INCLUDED_BF65DE29 */
