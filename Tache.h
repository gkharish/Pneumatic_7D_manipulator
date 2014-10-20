#include "TypeProcedure.h"
// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#ifndef TACHE_H_HEADER_INCLUDED_BF65D9D0
#define TACHE_H_HEADER_INCLUDED_BF65D9D0

#include <taskLib.h>

//##ModelId=403B6814030A
class Tache
{
  public:
    //##ModelId=408D24E40163
    Tache(int tid = 0);

    //##ModelId=408D24E4016D
    virtual ~Tache();

    //##ModelId=4084CB4D039E
    static int creerTache(char *nom, int priorite, int options, int stackSize, FUNCPTR pointDEntree, int arg1 = 0, int arg2 = 0, int arg3 = 0, int arg4 = 0, int arg5 = 0, int arg6 = 0, int arg7 = 0, int arg8 = 0, int arg9 = 0, int arg10 = 0);

    //##ModelId=4084CB4D03A8
    bool suspendre();

    //##ModelId=4084CB4D03A9
    bool reprendre();

    //##ModelId=4084CB4D03B2
    bool relancer();

    //##ModelId=408D270202F3
    bool tuer();

    //##ModelId=4084CB4D03BC
    bool setPriorite(int priorite);

    //##ModelId=4084CB4D03BE
    int priorite() const;
    
    //##ModelId=40A38BF103CA
    bool creer(char *nom, int priorite, int options, int stackSize, FUNCPTR pointDEntree, int arg1 = 0, int arg2 = 0, int arg3 = 0, int arg4 = 0, int arg5 = 0, int arg6 = 0, int arg7 = 0, int arg8 = 0, int arg9 = 0, int arg10 = 0);


  private:
    //##ModelId=403B718F027E
    int _id;
};



#endif /* TACHE_H_HEADER_INCLUDED_BF65D9D0 */
