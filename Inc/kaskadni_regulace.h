/*
 * kaskadni_regulace.h
 *
 *  Created on: 19. 4. 2020
 *      Author: FiveFires
 */

#ifndef KASKADNI_REGULACE_H_
#define KASKADNI_REGULACE_H_

// MACROS
#define  Ts			100e-6 // 100 mikrosekund je perioda volani DMA interruptu..
						   //(128MHz, 6400 dolu a nahoru)

#include "main.h"

// TYPEDEFS

// PI REGULATOR NAPETI typedef
typedef struct{

	float Kp;

	float slozka_p;

	float Ti;

	float slozka_i_memory;

	float slozka_i;

	float mezivypocet_vystupniho_proudu;

	float I_referencni; // output napetoveho regulatoru

	float Horni_limit_proudu;

	float Dolni_limit_proudu;

}Type_PI_Napeti;

// PI REGULATOR PROUDU typedef
typedef struct{

	float Kp;

	float slozka_p;

	float Ti;

	float slozka_i_memory;

	float slozka_i;

	float mezivypocet_vystupniho_napeti;

	float U_ridici; // output proudoveho regulatoru

	float Horni_limit_napeti;

	float Dolni_limit_napeti;

}Type_PI_Proud;

// MENIC typedef
typedef struct{

	float f_pwm;

	float tau_m;

	float c0;

	float c1;

	float Um_memory;

	float Um_output; // output menice

}Type_FiltrMenic;

// RL OBVOD typedef
typedef struct{

	float La;

	float Ra;

	float tau_a;

	float c0;

	float c1;

	float Ia_memory;

	float Ia_output; // output RL obvodu

}Type_RL_Obvod;

// STRUKTURA KONDENZATORU typedef
typedef struct{

	float c;		 // kapacita kondenzatoru

	float Uc_memory; // memory pro integrator

	float Uc_output; // output napeti na kondenzatoru, to co ridime

}Type_Kondenzator;

// HLAVNI STRUKTURA POINTERU A PROMENNYCH typedef
typedef struct{

	uint32_t PotValue;				 // hodnota z AD prevodniku, potenciometr

	float referencni_hodnota_napeti;	 	 // pozadovana hodnota napeti na kondenzatoru

	float error_value_napetiReg; 	 // vstupni hodnota do napetoveho regulatoru

	Type_PI_Napeti *p_PI_napeti;	 // pointer na strukturu napetoveho PI regulatoru

	float error_value_proudReg;		 // vstupni hodnota do proudoveho regulatoru

	Type_PI_Proud *p_PI_proud; 	 	 // pointer na strukturu proudoveho PI regulatoru

	float Ua;						 // vstupni hodnota napeti do RL obvodu

	Type_FiltrMenic *p_menic; 	 	 // pointer na strukturu menice

	Type_RL_Obvod *p_RL_obvod;	 	 // pointer na strukturu RL obvodou

	Type_Kondenzator *p_kondenzator; // pointer na strukturu kondenzatoru

}Type_PointerStructure;

// Deklarace funkci

// Funkce pro tvorbu referencniho signalu
void ReferencniHodnotaPot(Type_PointerStructure *pointer_structure);

// Funkce pro vypocet kaskadni regulace
void KaskadniRegulaceInit(Type_PointerStructure *pointer_structure);
void KaskadniRegulaceMain(Type_PointerStructure *pointer_structure);

// Funkce pro vypocet napetoveho PI regulatoru
void PI_NapetiInit(Type_PI_Napeti*pointer_PI_napeti);
void PI_Napeti(Type_PI_Napeti *pointer_PI_napeti,float error_value_napetiReg);

// Funkce pro vypocet proudoveho PI regulatoru
void PI_ProudInit(Type_PI_Proud *pointer_PI_proud);
void PI_Proud(Type_PI_Proud *pointer_PI_proud,float error_value);

// Funkce pro vypocet menice
void FiltrMenicInit(Type_FiltrMenic *pointer_menic);
void FiltrMenic(Type_FiltrMenic *pointer_menic, float U_ridici);

// Funkce pro vypocet RL obvodu (kotvy)
void RL_ObvodInit(Type_RL_Obvod *pointer_RL_obvod);
void RL_Obvod(Type_RL_Obvod *pointer_RL_obvod, float Um_output);

// Funkce pro vypocet Kondenzatoru s integratorem
void KondenzatorInit(Type_Kondenzator *pointer_kondenzator);
void Kondenzator(Type_Kondenzator *pointer_kondenzator, float Ia_output);

#endif /* KASKADNI_REGULACE_H_ */
