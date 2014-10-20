// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#ifndef HORLOGESYNCHRONISEE_H_HEADER_INCLUDED_BF4D0F64
#define HORLOGESYNCHRONISEE_H_HEADER_INCLUDED_BF4D0F64

#include "Evenement.h"
//#include <sysLib.h>
#include <slist.h>

// Cette implémentation de l'interface horloge utilise des fonctions
// spécifiques à VxWorks permettant 
//##ModelId=40AB01E40216
class Horloge
{
  public:
    // La structure suivante est utilisée comme contenu de la liste des
    // horloges. 
    //##ModelId=40AB11DD0310
    class ParametresDHorloge
    {
      public:
        //##ModelId=40AB5E890254
        ParametresDHorloge(int id, int periode, Evenement* pEvenement);

        //##ModelId=40AB254A0131
        int _id;

        // Periode en ms de l'horloge.
        //##ModelId=40AB127101F1
        int _periode;

        // Pointeur sur l'evenement à signaler au top d'horloge.
        //##ModelId=40AB1258033F
        Evenement* _pEvenement;

    };

    //##ModelId=40AB1BEC006A
    typedef slist<ParametresDHorloge> ListeDHorloges;

    //##ModelId=40AB02040335
    Horloge();

    //##ModelId=40AB02040336
    virtual ~Horloge();

    //##ModelId=40AB02040339
    bool demarrer(const Duree& periode);

    //##ModelId=40AB0204033B
    bool arreter();

    //##ModelId=40AB0204033C
    bool attendre() const;

  private:
    // Sous-programme d'interruption connecté à l'horloge auxiliaire de VxWorks
    //##ModelId=40AB0204033F
    static void procedureDHorloge();

    //##ModelId=40AB253D018C
    int _id;

    // Evenement utilisé pour signaler et attendre les tops d'horloge.
    //##ModelId=40AB029302FE
    Evenement _finDePeriode;

    //##ModelId=40AB0E360077
    static ListeDHorloges _listeDesHorloges;
    
    //##ModelId=40D1E2160214
    char _etat;


};



#endif /* HORLOGESYNCHRONISEE_H_HEADER_INCLUDED_BF4D0F64 */

