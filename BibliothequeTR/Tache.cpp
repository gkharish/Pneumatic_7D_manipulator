// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#include "Tache.h"

//##ModelId=408D24E40163
Tache::Tache(int tid)
{
	_id = tid;
}

//##ModelId=408D24E4016D
Tache::~Tache()
{
	taskDelete(_id);
}

//##ModelId=4084CB4D039E
int Tache::creerTache(char *nom, int priorite, int options, int stackSize, FUNCPTR pointDEntree, int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{
	return (taskSpawn(nom, priorite, options, stackSize, pointDEntree, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10));
}

//##ModelId=4084CB4D03A8
bool Tache::suspendre()
{
	return (taskSuspend(_id)==OK);
}

//##ModelId=4084CB4D03A9
bool Tache::reprendre()
{
	return (taskResume(_id)==OK);
}

//##ModelId=4084CB4D03B2
bool Tache::relancer()
{
	return (taskRestart(_id)==OK);
}

//##ModelId=4084CB4D03BC
bool Tache::setPriorite(int priorite)
{
	return (taskPrioritySet(_id, priorite)==OK);
}

//##ModelId=4084CB4D03BE
int Tache::priorite() const
{
	int priorite;
	taskPriorityGet(_id, &priorite);
	
	return (priorite);
}

//##ModelId=408D270202F3
bool Tache::tuer()
{
	return (taskDelete(_id)==OK);
}

//##ModelId=40A38BF103CA
bool Tache::creer(char *nom, int priorite, int options, int stackSize, FUNCPTR pointDEntree, int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{
	return ((_id = taskSpawn(nom, priorite, VX_FP_TASK, stackSize, pointDEntree, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10))!=ERROR);
}

