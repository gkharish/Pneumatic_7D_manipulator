// /******************************************************************************/
// /*                     CONTROLEUR DU ROBOT 7 AXES                             */
// /*                                                                            */
// /* Projet réalisé dans le cadre du DEA de Dave THOMAS de fév-jul 2004         */
// /* L'implémentation est faite en C++ et utilise les primitives du SE VxWorks. */
// /*                                                                            */
// /******************************************************************************/

#ifndef TEMPS_H_HEADER_INCLUDED_BF5F6691
#define TEMPS_H_HEADER_INCLUDED_BF5F6691

#include <sysLib.h>

//##ModelId=4087BDA900E5
enum UniteDeTemps {US, MS, S, MN, H};

//##ModelId=4087BC8700AC
class Duree
{
  public:
    //##ModelId=40A0A2BF03B2
    Duree();

    //##ModelId=40A07118030E
    Duree(float valeur, UniteDeTemps unite = S);

    //##ModelId=40A071180336
    virtual ~Duree();

    //##ModelId=40A089F10011
    double toSecondes() const;

    //##ModelId=40A071180338
    const Duree& operator=(const Duree& obj);

    //##ModelId=40A073DF0049
    Duree operator+(const Duree& obj);

    //##ModelId=40A073D702FA
    Duree operator-(const Duree& obj);

  private:
    //##ModelId=40A070D00093
    double _secondes; //duree en ms = _duree*0.1

};


//##ModelId=409F973B0301
class Date
{
  public:
    //##ModelId=40A0798003E4
    Date();

    //##ModelId=40A0798003E5
    virtual ~Date();

    //##ModelId=409F950E00D0
    void setNow();

    //##ModelId=40A073F30278
    Date operator=(const Date& obj);

    //##ModelId=409F971B017F
    Duree operator-(const Date& obj);
    
    //##ModelId=40A096CF0372
    Date operator+(const Duree& duree);
    
    //##ModelId=40A0981F03BB
    static Date getDateActuelle();
    
    //##ModelId=40A09B9303D0
	bool operator<(const Date& right);

    //##ModelId=40A09B9303DA
	bool operator>(const Date& right);


  private:
    //##ModelId=40A07542005D
    unsigned int _date;

};

#endif /* TEMPS_H_HEADER_INCLUDED_BF5F6691 */
