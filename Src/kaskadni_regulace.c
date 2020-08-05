/*
 * kaskadni_regulace.c
 *
 *  Created on: 19. 4. 2020
 *      Author: FiveFires
 */

#include "kaskadni_regulace.h"

// promenne pro generovani referencniho signalu
enum direction{ZERO,
		 AMPLITUDA
};

uint8_t direction = ZERO;

// promenna pro ladeni regulatoru proudove smycky, pokud se rovna 1 tak se vypne vnejsi smycka
uint8_t proudova_smycka_ladeni = 0;

// Hlavni funkce kaskadni regulace provadejici vypocty
void KaskadniRegulaceMain(Type_PointerStructure *pointer_structure){

	if(proudova_smycka_ladeni == 0){

	//rozdil Referencni hodnoty napeti (zadana hodnota potenciometr) a Uc_output (napeti na kondenzatoru)
	pointer_structure->error_value_napetiReg = (pointer_structure->referencni_hodnota_napeti)-(pointer_structure->p_kondenzator->Uc_output);

	// Vypocet Napetoveho PI regulatoru
	PI_Napeti(pointer_structure->p_PI_napeti, pointer_structure->error_value_napetiReg);

	// rozdil I_referencni (vystup z regulatoru napeti) a I_output (vystup z RL obvodu)
	pointer_structure->error_value_proudReg = (pointer_structure->p_PI_napeti->I_referencni)-(pointer_structure->p_RL_obvod->Ia_output);

	// Vypocet Proudoveho PI regulatoru
	PI_Proud(pointer_structure->p_PI_proud, pointer_structure->error_value_proudReg);

	// Vypocet vystupniho napeti menice
	FiltrMenic(pointer_structure->p_menic, pointer_structure->p_PI_proud->U_ridici);

	// rozdil Vystupniho napeti(Um_output) a Napeti na kondenzatoru(Uc_output)
	pointer_structure->Ua = (pointer_structure->p_menic->Um_output)-(pointer_structure->p_kondenzator->Uc_output);

	// Vypocet proudu v RL obvodu
	RL_Obvod(pointer_structure->p_RL_obvod, pointer_structure->Ua);

	// Vypocet napeti na kondenzatoru
	Kondenzator(pointer_structure->p_kondenzator, pointer_structure->p_RL_obvod->Ia_output);

	}

	else if(proudova_smycka_ladeni == 1){

			// zapis pozadovane hodnoty (I_referencni) pro naladeni proudove smycky
			pointer_structure->p_PI_napeti->I_referencni = pointer_structure->referencni_hodnota_napeti;

			// rozdil I_referencni a I_output (vystup z RL obvodu)
			pointer_structure->error_value_proudReg = (pointer_structure->p_PI_napeti->I_referencni)-(pointer_structure->p_RL_obvod->Ia_output);

			// Vypocet Proudoveho PI regulatoru
			PI_Proud(pointer_structure->p_PI_proud, pointer_structure->error_value_proudReg);

			// Vypocet vystupniho napeti menice
			FiltrMenic(pointer_structure->p_menic, pointer_structure->p_PI_proud->U_ridici);

			// Vypocet proudu v RL obvodu
			RL_Obvod(pointer_structure->p_RL_obvod, pointer_structure->p_menic->Um_output);

	}
}

void PI_Napeti(Type_PI_Napeti *pointer_PI_napeti,float error_value_napetiReg){

	// vypocet referencniho proudu pro podrizenou proudovou smycku, vystup napetoveho PI regulatoru
	pointer_PI_napeti->slozka_p = (pointer_PI_napeti->Kp)*error_value_napetiReg;
	pointer_PI_napeti->slozka_i = (pointer_PI_napeti->slozka_i_memory) + (Ts/(pointer_PI_napeti->Ti))*error_value_napetiReg;

	// mezivypocet pro limitaci a vykreslovani spravne finalni hodnoty
	pointer_PI_napeti->mezivypocet_vystupniho_proudu = (pointer_PI_napeti->slozka_i)+(pointer_PI_napeti->slozka_p);

	// LOGICKY BLOK PRO LIMITACI VYSTUPNIHO PROUDU

	// Pri prekroceni limitu se zmrazi pamet integracni slozky
	// pokud je mezivypocet vyssi nez limitni proud, jako vystup se nastavi limitni proud;
	if(pointer_PI_napeti->mezivypocet_vystupniho_proudu >= pointer_PI_napeti->Horni_limit_proudu){
		pointer_PI_napeti->I_referencni = pointer_PI_napeti->Horni_limit_proudu;
	}

	// pokud je mezivypocet nizsi nez dolni limit proudu, nastavi se vystup dolni limit proudu
	else if(pointer_PI_napeti->mezivypocet_vystupniho_proudu <=  pointer_PI_napeti->Dolni_limit_proudu){
		pointer_PI_napeti->I_referencni = pointer_PI_napeti->Dolni_limit_proudu;
	}

	else{
	pointer_PI_napeti->I_referencni = pointer_PI_napeti->mezivypocet_vystupniho_proudu;
	pointer_PI_napeti->slozka_i_memory = pointer_PI_napeti->slozka_i; // ulozeni predesleho kroku
	}

	//KONEC LOGICKEHO BLOKU PRO LIMITACI

}

void PI_Proud(Type_PI_Proud *pointer_PI_proud,float error_value_proudReg){

	// vypocet Ridiciho napeti pro menic, vystup proudoveho PI regulatoru
	pointer_PI_proud->slozka_p = (pointer_PI_proud->Kp)*error_value_proudReg;
	pointer_PI_proud->slozka_i = (pointer_PI_proud->slozka_i_memory) + (Ts/(pointer_PI_proud->Ti))*error_value_proudReg;

	// mezivypocet pro limitaci a vykreslovani spravne finalni hodnoty
	pointer_PI_proud->mezivypocet_vystupniho_napeti = (pointer_PI_proud->slozka_i)+(pointer_PI_proud->slozka_p);

	// LOGICKY BLOK PRO LIMITACI VYSTUPNIHO PROUDU

	// Pri prekroceni limitu se zmrazi pamet integracni slozky
	// pokud je mezivypocet vyssi nez limitni napeti, jako vystup se nastavi limitni proud
	if(pointer_PI_proud->mezivypocet_vystupniho_napeti >= pointer_PI_proud->Horni_limit_napeti){
		pointer_PI_proud->U_ridici = pointer_PI_proud->Horni_limit_napeti;
	}

	// pokud je mezivypocet nizsi nez dolni limit napeti, nastavi se vystup na dolni limit
	else if(pointer_PI_proud->mezivypocet_vystupniho_napeti <= pointer_PI_proud->Dolni_limit_napeti){
		pointer_PI_proud->U_ridici = pointer_PI_proud->Dolni_limit_napeti;
	}

	else{
		pointer_PI_proud->U_ridici = pointer_PI_proud->mezivypocet_vystupniho_napeti;
		pointer_PI_proud->slozka_i_memory = pointer_PI_proud->slozka_i; // ulozeni predesleho kroku
	}

	//KONEC LOGICKEHO BLOKU PRO LIMITACI

}

void FiltrMenic(Type_FiltrMenic *pointer_menic, float U_ridici){

	// vypocet vystupniho napeti z menice ktere vstupuje do RL obvodu
	pointer_menic->Um_output = U_ridici*(pointer_menic->c0) + (pointer_menic->Um_memory)*(pointer_menic->c1);

	pointer_menic->Um_memory = pointer_menic->Um_output; // ulozeni predesleho kroku
}

void RL_Obvod(Type_RL_Obvod *pointer_RL_obvod, float Um_output){

	// vypocet proudu RL obvodu
	pointer_RL_obvod->Ia_output = (Um_output/(pointer_RL_obvod->Ra))*(pointer_RL_obvod->c0) + ((pointer_RL_obvod->Ia_memory)*(pointer_RL_obvod->c1));

	pointer_RL_obvod->Ia_memory = pointer_RL_obvod->Ia_output; // ulozeni predesleho kroku
}

void Kondenzator(Type_Kondenzator *pointer_kondenzator, float Ia_output){

	// vypocet napeti na kondenzatoru
	pointer_kondenzator->Uc_output = (Ia_output/(pointer_kondenzator->c))*Ts + (pointer_kondenzator->Uc_memory);

	pointer_kondenzator->Uc_memory = pointer_kondenzator->Uc_output; // ulozeni predesleho kroku

}

// Inicializacni funkce pro kaskadni regulaci
void KaskadniRegulaceInit(Type_PointerStructure *pointer_structure){

	PI_NapetiInit(pointer_structure->p_PI_napeti);

	PI_ProudInit(pointer_structure->p_PI_proud);

	FiltrMenicInit(pointer_structure->p_menic);

	RL_ObvodInit(pointer_structure->p_RL_obvod);

	KondenzatorInit(pointer_structure->p_kondenzator);

}

void PI_NapetiInit(Type_PI_Napeti*pointer_PI_napeti){

	pointer_PI_napeti->Kp = 0.3; // zesileni P
	pointer_PI_napeti->Ti = 1000; // zesileni I

	pointer_PI_napeti->Horni_limit_proudu = 10; 	// horni limit vystupu regulatoru nastaven na 10A

	pointer_PI_napeti->Dolni_limit_proudu = -10;	// dolni limit vystupu regulatoru nastaven na -10A

	pointer_PI_napeti->slozka_i_memory = 0; //pocatecni hodnota pro pamet I regulatoru
}

void PI_ProudInit(Type_PI_Proud *pointer_PI_proud){

	pointer_PI_proud->Kp = 0.0015; // zesileni P , vynasobeno extra Ts, jinak kmita
	pointer_PI_proud->Ti = 4.166e-5; // zesileni I , vydeleno extra Ts, jinak kmita

	pointer_PI_proud->Horni_limit_napeti = 24; // horni limit vystupu regulatoru nastaven na 24V,
											   // coz je napajeci napeti menice (strida = 1)

	pointer_PI_proud->Dolni_limit_napeti = 0;  // dolni limit vystupu regulatoru nastaven na 0,
											   // coz je min hodnota napeti menice (strida = 0)

	pointer_PI_proud->slozka_i_memory = 0; //pocatecni hodnota pro pamet I regulatoru
}

void FiltrMenicInit(Type_FiltrMenic *pointer_menic){

	pointer_menic->f_pwm = 10000; // pwm "Menice" (TIM1)
	pointer_menic->tau_m = 1/(2*(pointer_menic->f_pwm)); // casova konstanta "menice"
	pointer_menic->c0 = Ts/((pointer_menic->tau_m) + Ts);
	pointer_menic->c1 = (pointer_menic->tau_m)/((pointer_menic->tau_m) + Ts);
	pointer_menic->Um_memory = 0; // pocatecni hodnota pro pamet filtru menice
}

void RL_ObvodInit(Type_RL_Obvod *pointer_RL_obvod){

	pointer_RL_obvod->La = 1.5e-3; // Indukcnost civky La = 100mH
	pointer_RL_obvod->Ra = 2.4;	   // Odpor Ra = 10 Ohm
	pointer_RL_obvod->tau_a = (pointer_RL_obvod->La)/(pointer_RL_obvod->Ra); // casova konstanta kotvy
	pointer_RL_obvod->c0 = Ts/((pointer_RL_obvod->tau_a) + Ts);
	pointer_RL_obvod->c1 = (pointer_RL_obvod->tau_a)/((pointer_RL_obvod->tau_a) + Ts);
	pointer_RL_obvod->Ia_memory = 0; // pocatecni hodnota pro pamet proudu kotvou

}

void KondenzatorInit(Type_Kondenzator *pointer_kondenzator){

	pointer_kondenzator->c = 1e-3; // Kapacita 100mikroFaradu
	pointer_kondenzator->Uc_memory = 0; // pocatecni hodnota pro pamet integratoru
}

void ReferencniHodnotaPot(Type_PointerStructure *pointer_structure){
	{
		if(direction == AMPLITUDA){
			// vypocet referencni hodnoty (max 4096/170 =~ 24 V referencni hodnota)
			pointer_structure->referencni_hodnota_napeti = (pointer_structure->PotValue)/170;
			direction = ZERO;
		}

		else if(direction == ZERO){
			pointer_structure->referencni_hodnota_napeti = 0;
			direction = AMPLITUDA;
		}
	}

}
