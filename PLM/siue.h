/*
 * siue.h
 *
 *  Created on: 13 déc. 2023
 *      Author: ebt
 */

#ifndef APP_SIUE_H_
#define APP_SIUE_H_
#include "pin_config.h"
#include "em_cmu.h"
#include "sl_sleeptimer.h"
#include "em_gpio.h"
#include "i2c.h"
#include "em_timer.h"
//#include "stimulation.h"

typedef union
{
    /*! Valeur 16 bits */
    uint16_t ui16bit;
    /*! Tableau de 2 octets */
    uint8_t u8bit[2];
} Data16bit_t;

typedef enum
{
    eEOFF = 0, /*!< Electrostimulation Off */
    eEPH = 1,  /*!< Electrostimulation Polarit� Haute */
    eEPB = 2,  /*!< Electrostimulation Polarit� Basse */
    eETAPE1,   /*!< Etape 1 de l'�lectrostimulation*/
    eETAPE1_B, /*!< Etape 1B de l'�lectrostimulation*/
    eETAPE2,   /*!< Etape 2 de l'�lectrostimulation*/
    eETAPE3,   /*!< Etape 3 de l'�lectrostimulation*/
    eETAPE4,   /*!< Etape 4 de l'�lectrostimulation*/
    eETAPE6,   /*!< Etape 6 de l'�lectrostimulation*/
    eETAPE7,   /*!< Etape 7 de l'�lectrostimulation*/
    eETAPE8,
    eETAPE9,
    eETAPE10,
    C2,
    C6,
    

} eCdeElectrostimulation_Type;

/*! \def iSERIAL_NUMBER_SIZE
\brief Taille du champ num�ro de s�rie */
#define iSERIAL_NUMBER_SIZE 10U

/*! \def iMODEL_NUMBER_SIZE
\brief Taille du champ model number*/
#define iMODEL_NUMBER_SIZE 12U

/*! \def iINTENSITE_MIN
\brief Bornes inf�rieure pour ISS et ISRP exprim�es en �A */
#define iINTENSITE_MIN 0U

/*! \def iINTENSITE_MAX
\brief Bornes sup�rieure pour ISS et ISRP exprim�es en �A */
#define iINTENSITE_MAX 50000U

/*! \def iTEMPS_ETABLISSEMENT_COURANT_MAX
\brief Bornes sup�rieure pour le temps d'�tablissment courant du SS (Tm) exprim� en ms */
#define iTEMPS_ETABLISSEMENT_COURANT_MAX 10000U

/*! \def iTEMPS_ETABLISSEMENT_COURANT_MIN
\brief Bornes inf�ieure pour le temps d'�tablissment courant du SS (Tm) exprim� en ms */
#define iTEMPS_ETABLISSEMENT_COURANT_MIN 1000U

/*! \def iSTEP_ISRP
\brief Step d'increment/d�crement de l'ISRP exprim�es en �A */
#if (MODE_COMPILATION == 'D')
#define iSTEP_ISRP 3000U
#else
#define iSTEP_ISRP 500U
#endif

/*! \def iSMD_MIN
\brief Bornes inf�rieure pour le SMD */
#define iSMD_MIN 0U

/*! \def iSMD_MAX
\brief Bornes sup�rieure pour le SMD */
#define iSMD_MAX 1260U

/*! \def iPSMD_MIN
\brief Bornes inf�rieure pour le PSMD */
#define iPSMD_MIN 0U

/*! \def iPSMD_MAX
\brief Bornes sup�rieure pour le PSMD */
#define iPSMD_MAX 100U

/*! \def PETIT_DELAI_NOP
\brief Petit d�lai par des cycles horloge */
#define PETIT_DELAI_NOP \
    {                   \
        __asm("NOP");   \
    }

/*! Valeur minimale de courant g�n�r� programmable pour la sonde (uA) */
#define iCOURANT_MINIMUM_PROGRAMMABLE 500U

/*! \def iSS_MOMENT_CALCUL_IAP
\brief Moment en �s d�finit pour calculer la prochaine intensit� � appliquer du SS */
#define iSS_MOMENT_CALCUL_IAP	                                                10000U   // en �s

/*! \def iSS_MOMENT_APPLICATION_AOP
\brief Moment en �s d�finit pour positionner l'AOP avant le d�but de 1ere polarit� du SS */
#define iSS_MOMENT_APPLICATION_AOP	                                        700U     // en �s

/*! \def iSS_MOMENT_RELECTURE_COURANT
\brief Moment en �s d�finit pour relever le courant de relecture durant une polarit� du SS */
#define iSS_MOMENT_RELECTURE_COURANT	                                        80U      // en �s

/*! \def iSS_DUREE_IMPULSION_RESTANTE_PROGRESSIF
\brief Dur�e restante � ex�cuter pour que les polarit�s du SS dans sa phase progressive soit respect�e */
#define iSS_DUREE_IMPULSION_RESTANTE_PROGRESSIF                                 200U     // en �s

/*! \def iSS_DUREE_IMPULSION_RESTANTE_PALIER
\brief Dur�e restante � ex�cuter pour que les polarit�s du SS dans sa phase palier soit respect�e */
#define iSS_DUREE_IMPULSION_RESTANTE_PALIER                                     200U     // en �s

/*! \def iNB_PURGE_ADC_ON_STOP_SS
\brief Nombre de purge de l'ADC r�alis�es � l'arr�t du SS */
#define iNB_PURGE_ADC_ON_STOP_SS                                                10
/*! \def iINTENSITE_PREPARATION_COMPOSANT_DAC
\brief Intensit� pour pr�parer les DAC avant positionnement des MOS */
#define iINTENSITE_PREPARATION_COMPOSANT_DAC 20000U // en �A

//-----------------------------------------------------------------------------
/* D�finition de constantes */
//-----------------------------------------------------------------------------

/*! ----- Constante d�finissant les timming n�cessaires au fonctionnemnt des SS -----*/

/*! \def iSS_DUREE_PALIER
\brief Dur�e du palier du SS apr�s la phase Tm */
#define iSS_DUREE_PALIER                                                        5000U    // en ms

/*! \def iSS_DUREE_TPS_BASE
\brief Dur�e de la partie base du SS */
#define iSS_DUREE_TPS_BASE	                                                10000   // en �s

/*! \def iSS_DUREE_IMPULSION
\brief Dur�e d'une polarit� du SS */
#define iSS_DUREE_IMPULSION	                                                400U     // en �s // en us  variable qui donne le temps de la pulsation Delta "t"

/*! \def iSS_DUREE_IMPULSION_H_CORRIGEE
\brief Dur�e d'e la polarit� haute du SS corrig�e */
#define iSS_DUREE_IMPULSION_H_CORRIGEE                                          284U     // en �s

/*! \def iSS_DUREE_IMPULSION_B_CORRIGEE
\brief Dur�e d'e la polarit� haute du SS corrig�e */
#define iSS_DUREE_IMPULSION_B_CORRIGEE                                          280U     // en �s

/*! \def iSS_TIMMING_COURT
\brief Timming court pour la gestion du SS */
#define iSS_TIMMING_COURT                                                       50U      // en �s

/*! ----- Liste des moments avant �ch�ance du palier du stimuli -----*/

/*! \def iSS_MOMENT_CALCUL_IAP
\brief Moment en �s d�finit pour calculer la prochaine intensit� � appliquer du SS */
#define iSS_MOMENT_CALCUL_IAP	                                                10000U   // en �s


/*! \def iSS_DUREE_IMPULSION_RESTANTE_PALIER
\brief Dur�e restante � ex�cuter pour que les polarit�s du SS dans sa phase palier soit respect�e */
#define iSS_DUREE_IMPULSION_RESTANTE_PALIER                                     200U     // en �s

/*! \def iNB_PURGE_ADC_ON_STOP_SS
\brief Nombre de purge de l'ADC r�alis�es � l'arr�t du SS */
#define iNB_PURGE_ADC_ON_STOP_SS                                                10


#define STIM_GEN_TRM_PERIOD_MAX   32768     // in �s*10^(-1)  //( ((uint32_t)(TMR_RC_COUNTER_MAX + 1)*DEF_TIME_NBR_uS_PER_SEC)/STIM_GEN_TRM_CLK_SOURCE )


int32_t Ad5691r_SetIntensiteStimulation(uint32_t v/*uint32_t ui32Intensite*/);
void Gpio_SetElectrostimulation(eCdeElectrostimulation_Type eCdeElectrostimulation);
void MeSS_GestionCourantBiphasiquePositif(void);
void  MeSS_GestionCourantMonophasiquePositif(void);
void MeSS_GestionCourantBiphasiqueNegatif(void);
void MeSS_GestionCourantMonophasiqueNegatif(void);
void MeSS_GestionCourantBiphasiqueAlterne(void);
void ImpulsMonophas(void);
void ImpulsBiphas(void);
void ImpulsBiphasNeg(void);
void ImpulsBiphasAltern(void);
void Gpio_SetAop();
void Gpio_ClrAop();
void Timer_SetMft1Timming(uint16_t ui16Time);

void VeineuxBiphas(void);
void ImportTrameDAL(void);

#endif /* APP_SIUE_H_ */
