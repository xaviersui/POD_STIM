/*!
* \brief	Fichier de d�finition des constantes et type de la Sonde SIUE-1316
* \file		SIUE1316_ConstType.h
* \version	G�r� en gestion de configuration GITLAB
* \author	MICROTEC (Christophe ALVERNHE)
*
* Gestion des constantes et types de la sonde SIUE-1316
*/
#ifndef _SIUE1316_CONSTTYPE_H       /* Guard against multiple inclusion */
#define _SIUE1316_CONSTTYPE_H 
//-----------------------------------------------------------------------------
/* Inclusions */
//----------------------------------------------------------------------------- 

/* Librairies standards */

/* Librairies FOURNISSEUR */

/* Librairies MICROTEC */

/* Librairies sp�cifiques � l'application */

//-----------------------------------------------------------------------------
/* D�finition de constantes */
//-----------------------------------------------------------------------------
#ifdef SIUE1316_OPTION_TEST_WATCHDOG
  #if (SIUE1316_OPTION_TEST_WATCHDOG=='1')
    #define COMPLEMENT_VERSION_WATCHDOG                                         "W1"    // Test de watchdog #1 
  #else
    #if (SIUE1316_OPTION_TEST_WATCHDOG=='2')
      #define COMPLEMENT_VERSION_WATCHDOG                                       "W2"    // Test de watchdog #2 
    #else
      #if (SIUE1316_OPTION_TEST_WATCHDOG=='3')
        #define COMPLEMENT_VERSION_WATCHDOG                                     "W3"    // Test de watchdog #3
      #else
        #if (SIUE1316_OPTION_TEST_WATCHDOG=='4')
          #define COMPLEMENT_VERSION_WATCHDOG                                   "W4"    // Test de watchdog #4 
        #else
          #error "Valeur SIUE1316_OPTION_TEST_WATCHDOG non impl�ment�e"
        #endif
      #endif
    #endif
  #endif
#else
  #define COMPLEMENT_VERSION_WATCHDOG                                           ""
#endif

#ifdef SIUE1316_SIMULATION_COURANT_DE_RELECTURE
  #if (SIUE1316_SIMULATION_COURANT_DE_RELECTURE=='0')
    #define COMPLEMENT_VERSION_SIMULATION_COURANT_DE_RELECTURE                  "CR0"    // Test de Courant de Relecture #0 
  #else
    #if (SIUE1316_SIMULATION_COURANT_DE_RELECTURE=='1')
      #define COMPLEMENT_VERSION_SIMULATION_COURANT_DE_RELECTURE                "CR1"    // Test de Courant de Relecture #1 
    #else
      #if (SIUE1316_SIMULATION_COURANT_DE_RELECTURE=='2')
        #define COMPLEMENT_VERSION_SIMULATION_COURANT_DE_RELECTURE              "CR2"    // Test de Courant de Relecture #2
      #else
        #if (SIUE1316_SIMULATION_COURANT_DE_RELECTURE=='3')
          #define COMPLEMENT_VERSION_SIMULATION_COURANT_DE_RELECTURE            "CR3"    // Test de Courant de Relecture #3 
        #else
          #error "Valeur SIUE1316_SIMULATION_COURANT_DE_RELECTURE non impl�ment�e"
        #endif
      #endif
    #endif
  #endif
#else
  #define COMPLEMENT_VERSION_SIMULATION_COURANT_DE_RELECTURE                    ""  
#endif

/*! \def sVERSION_LOGICIEL
\brief Version du logiciel */
#define sVERSION_LOGICIEL                                                       "SIUE1316_V2.2.0"COMPLEMENT_VERSION_WATCHDOG""COMPLEMENT_VERSION_SIMULATION_COURANT_DE_RELECTURE

/*! \def iSERIAL_NUMBER_SIZE
\brief Taille du champ num�ro de s�rie */
#define iSERIAL_NUMBER_SIZE                                                     10U

/*! \def iMODEL_NUMBER_SIZE
\brief Taille du champ model number*/
#define iMODEL_NUMBER_SIZE                                                      12U

/*! \def iINTENSITE_MIN
\brief Bornes inf�rieure pour ISS et ISRP exprim�es en �A */
#define iINTENSITE_MIN                                                          0U

/*! \def iINTENSITE_MAX
\brief Bornes sup�rieure pour ISS et ISRP exprim�es en �A */
#define iINTENSITE_MAX                                                          50000U

/*! \def iTEMPS_ETABLISSEMENT_COURANT_MAX
\brief Bornes sup�rieure pour le temps d'�tablissment courant du SS (Tm) exprim� en ms */
#define iTEMPS_ETABLISSEMENT_COURANT_MAX                                        10000U

/*! \def iTEMPS_ETABLISSEMENT_COURANT_MIN
\brief Bornes inf�ieure pour le temps d'�tablissment courant du SS (Tm) exprim� en ms */
#define iTEMPS_ETABLISSEMENT_COURANT_MIN                                        1000U

/*! \def iSTEP_ISRP 
\brief Step d'increment/d�crement de l'ISRP exprim�es en �A */
#if (MODE_COMPILATION =='D')
  #define iSTEP_ISRP                                                            3000U
#else
  #define iSTEP_ISRP                                                            500U
#endif

/*! \def iSMD_MIN
\brief Bornes inf�rieure pour le SMD */
#define iSMD_MIN                                                                0U

/*! \def iSMD_MAX
\brief Bornes sup�rieure pour le SMD */
#define iSMD_MAX                                                                1260U

/*! \def iPSMD_MIN 
\brief Bornes inf�rieure pour le PSMD */
#define iPSMD_MIN                                                               0U

/*! \def iPSMD_MAX
\brief Bornes sup�rieure pour le PSMD */
#define iPSMD_MAX                                                               100U

/*! \def PETIT_DELAI_NOP
\brief Petit d�lai par des cycles horloge */
#define PETIT_DELAI_NOP {                                                       \
  asm("NOP");                                                                   \
}

/*! Valeur minimale de courant g�n�r� programmable pour la sonde (�A) */
#define iCOURANT_MINIMUM_PROGRAMMABLE                                           500U

/*! \def iINTENSITE_PREPARATION_COMPOSANT_DAC
\brief Intensit� pour pr�parer les DAC avant positionnement des MOS */
#define iINTENSITE_PREPARATION_COMPOSANT_DAC                                    20000U  // en �A

//-----------------------------------------------------------------------------
/* D�finition de types et structures */
//-----------------------------------------------------------------------------
/*! \enum ePositionActionDansEtat_Type
*   \brief Liste des positions des actions dans un �tat
*/
typedef enum 
{
  eACTION_EN_ENTREE=0,
  eACTION_EN_BOUCLE=1,
  eACTION_EN_SORTIE=2,
  eNO_ACTION
} ePositionActionDansEtat_Type;

/*! \enum eEtatProgrammeSonde_Type
*   \brief Liste possible des programmes en cours de la Sonde
*/
typedef enum 
{
  eSS_PROGRAMME=0,
  eSRP1_PROGRAMME=1,
  eSRP2_PROGRAMME=2,
  eSRP3_PROGRAMME=3,
  eNO_PROGRAMME_SONDE=255
} eEtatProgrammeSonde_Type;

/*! \enum ePhaseMesureStimuli_Type
*   \brief Liste phases de mesure du courant de relecture sur le stimuli
*/
typedef enum 
{
  eMESURE_STIMULI_HAUT_EN_MONTEE=0,
  eMESURE_STIMULI_BAS_EN_MONTEE=1,
  eMESURE_STIMULI_HAUT_EN_PALIER=2,
  eMESURE_STIMULI_BAS_EN_PALIER=3,
} ePhaseMesureStimuli_Type;

/*! \struct stCouplePressionIap_Type
*   \brief Structure liant la pression avec l'Iap calcul�
*/
//typedef struct {
//  float fPression;      /*!< Pression */
//  uint32_t ui32Iap;     /*! Intensit� IAP */
//}stCouplePressionIap_Type;

//-----------------------------------------------------------------------------
/* D�finition des fonctions et proc�dures export�es */
//-----------------------------------------------------------------------------

#endif /* _SIUE1316_CONSTTYPE_H */
//-----------------------------------------------------------------------------
/*!
 Fin du fichier
*/
