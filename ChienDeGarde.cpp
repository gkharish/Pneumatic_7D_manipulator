// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#include "ChienDeGarde.h"
#include <stdio.h>

//##ModelId=408D249903BE
ChienDeGarde::ChienDeGarde()
{
	_id = wdCreate();
}


//##ModelId=408D249903C8
ChienDeGarde::~ChienDeGarde()
{
	wdCancel(_id);
	wdDelete(_id);
}

//##ModelId=4084CE3202C8
bool ChienDeGarde::arreter()
{
	return (wdCancel(_id)==OK);
}

//##ModelId=4084CE3202C7
bool ChienDeGarde::demarrer(const Duree& temps, TypeProcedure procedure, int parametre)
{
        int delay;
        delay = (int) ((float) sysClkRateGet() * temps.toSecondes() + 0.5);

				if (delay < 5)
    		{
      		perror("watchdog nb ticks trop faible");
      		return false;
				}
				
        return (wdStart(_id,     /* watchdog ID */
                                delay,    /* delay count, in ticks */
                                procedure, /* routine to call on time-out */
                                parametre /* parameter with which to call routine */
                                )
                ==OK);
}
