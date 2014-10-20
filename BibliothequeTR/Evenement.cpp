// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#include "Evenement.h"

//##ModelId=403C62B50065
Evenement::Evenement()
{
	_id = semBCreate(SEM_Q_FIFO, SEM_EMPTY);
}


//##ModelId=403C62B5006F
Evenement::~Evenement()
{
	semDelete(_id);
}

// Fonction suspendant la tache appelante jusqu'a ce que l'evenement soit
// signale
//##ModelId=403B4A7901CB
inline bool Evenement::attendre() const
{
	return (semTake(_id, WAIT_FOREVER)==OK);
}

//##ModelId=403B4A8402F4
inline bool Evenement::signaler()
{
	return (semGive(_id)==OK);
}

//##ModelId=409B597A018D
inline bool Evenement::signalerATous()
{
	return (semFlush(_id)==OK);
}

