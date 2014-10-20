#include "CapteurDePosition.h"
#include "InterfaceES\InterfaceCarteDEntreeAnalogique.h"

//##ModelId=4087CFEB003B
CapteurDePosition::CapteurDePosition(InterfaceCarteDEntreeAnalogique* pCarteDEntreeAnalogique, char voie, float pente, float offset)
{
	_pCarteDEntreeAnalogique = pCarteDEntreeAnalogique;
	_voie = voie;
	_pente = pente;
	_offset = offset;
  ad= new float[6]; // rajout karim le 18/07/2008
  for(int i=0;i<6;i++)   // rajout karim le 21/07/2008
{	
	ad[i]=0.0f;
}

  

//	_validite.setNow();
}

//##ModelId=4087CFDC02D9
CapteurDePosition::~CapteurDePosition()
{
}

//##ModelId=407CF4070033
float CapteurDePosition::lirePosition() const	//fournit la position en VOLTS
{
float position;
float posi[6];


for(int i=0;i<4;i++) position = _pCarteDEntreeAnalogique->adConv(_voie);
position = 0.0;

for(int i=0;i<6;i++)
{	
	posi[i] = 0.0f; //initialise position
	posi[i] = _pCarteDEntreeAnalogique->adConv(_voie);
}

position = (9.87e-2*posi[5])+(17.04e-2*posi[4])+(22.93e-2*posi[3])
					+(22.93e-2*posi[2])+(17.04e-2*posi[1])+(9.87e-2*posi[0]);

	
//  position += _pCarteDEntreeAnalogique->adConv(_voie);
//	position /=10.0f; //moyenne de 10 lectures


	return (position);
}

//##ModelId=4098F6210095
void CapteurDePosition::setZero()
{
		for(int i=0;i<10; i++)
		_offset += _pCarteDEntreeAnalogique->adConv(_voie);
		_offset /=10.0f; //moyenne de 10 lectures

}

//##ModelId=4098F62100BD
void CapteurDePosition::setPente(float left)
{
    _pente = left;
}


//##ModelId=40A09D8E0249
void CapteurDePosition::setOffset(float left)
{
	_offset2 = left;		// left / _pente;
}


//=====================================================================
//void CapteurDePosition::setPosition(float xxx)
//{
	//_positionk = xxx;		// position actuelle en radian filtere ajout karim
//}

//---------------------------------------------------------------------
//float CapteurDePosition::getPosition()
//{
//	return _positionk; // retourne position actuelle filtrer en radian ajout karim
//}

//=====================================================================

float CapteurDePosition::getOffset()
{
	return _offset;
}


float CapteurDePosition::getPente()
{
	return _pente;
}

float CapteurDePosition::lirePositionRAD() const	//fournit la position en RADIANS
												//comme une moyenne de 5(10) lectures

{
/*
	float position = 0.0f; //initialise position
	for(int i=0;i<10; i++)
	position += _pCarteDEntreeAnalogique->adConv(_voie);
	position /=10.0f;
	position = (position - _offset2) * _pente; // offset2 est regle par setOffset

//	position = (position - _offset) * _pente;	
	return (position);
*/
float position;
float posi[6];
//int cmp; // ajout karim

for(int i=0;i<4;i++) position = _pCarteDEntreeAnalogique->adConv(_voie);
position = 0.0f;
for(int i=0;i<6;i++)
{	
	posi[i] = 0.0f; //initialise position
	posi[i] = _pCarteDEntreeAnalogique->adConv(_voie);
}
position = (9.87e-2*posi[5])+(17.04e-2*posi[4])+(22.93e-2*posi[3])
					+(22.93e-2*posi[2])+(17.04e-2*posi[1])+(9.87e-2*posi[0]);


//==========================karim ajout le 21/07/2008==============================
//*************** bute de ce code fillter le signale de lecture********************
//=================================================================================

//cmp=5;
//for(int i=0;i<5;i++)
//{	
//	ad[cmp] = ad[cmp-1]; 
//  cmp--;
//}

//ad[0]=position;

//position = (9.87e-2*ad[0])+(17.04e-2*ad[1])+(22.93e-2*ad[2])
//					+(22.93e-2*ad[3])+(17.04e-2*ad[4])+(9.87e-2*ad[5]);

//====================================================================================
	

position = (position - _offset2) * _pente;
//	position += _pCarteDEntreeAnalogique->adConv(_voie);
//	position /=10.0f; //moyenne de 10 lectures


	return (position);
}





float CapteurDePosition::lirePositionRAD2(float data1,float data2,float data3,float data4,float data5,float data6) 	//fournit la position en RADIANS
												//comme une moyenne de 5(10) lectures

{

float x;


x=0;
x= (9.87e-2*data1)+(17.04e-2*data2)+(22.93e-2*data3)
					+(22.93e-2*data4)+(17.04e-2*data5)+(9.87e-2*data6);

return (x);
}

