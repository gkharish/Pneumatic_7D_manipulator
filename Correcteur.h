#ifndef CORRECTEUR_H_HEADER_INCLUDED_BF72A545
#define CORRECTEUR_H_HEADER_INCLUDED_BF72A545
class Duree;


#define _PID  1
#define _TW   2
#define _STW  3
//#define _TSMC  4         //amar 10-04-2012....


class Correcteur
{
  public:
    virtual float calculerCommande(float, float, float) = 0;
    


		virtual	float	getErreur(void)	=0;
    virtual void initialise(float) = 0;
		virtual	void	setSuivi(bool)	=	0;

    virtual float getUeq(void)    = 0;
    virtual float getUdelta(void) = 0;

    virtual float getSurface(void) = 0;

		virtual float getSurfacePoint(void)    = 0;



    //***********************************karim ajout*************************************************

    virtual float	getuDEUX(void)  = 0; // ajout karim pour etre utilise dans la classe stw mais doivent etre ajout dans les autrs
    

   //*************************************************************************************************




		bool isTW();
		bool isSTW();
    bool isPID();
	//	bool isTSMC();  // amar le 10-04-2012.......
		
	protected:
		int _type = _PID;
};



#endif /* CORRECTEUR_H_HEADER_INCLUDED_BF72A545 */