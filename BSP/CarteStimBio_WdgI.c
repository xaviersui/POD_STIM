/*!
 * \file   CarteRf_WdgI.c
 * \brief  Gestion du watchdog interne
 * \author MICROTEC BE
 *
 */

//-----------------------------------------------------------------------------
/* Inclusions */
//-----------------------------------------------------------------------------
/* Librairies standards */
#include <stdbool.h>

/* Librairies FOURNISSEUR */
#if (bOPTION_GESTION_WDG == 1)
#include "em_cmu.h"
#include "em_wdog.h"
#endif

/* Librairies spécifiques à l'application */
#include <CarteStimBio_WdgI.h>

//-----------------------------------------------------------------------------
/* Définition de constantes */
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/* Définition de types et structures */
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/* Définition des fonctions et procédures non exportées */
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/* Variables globales */
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/* Variables et fonctions importées */
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/* Fonctions privées */
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/* Fonctions publiques */
//-----------------------------------------------------------------------------
/*!
 * \brief Initialisation du watchdog interne
 * \return eErr_type status du traitement
 */
eErr_type CarteRf_WdgI_Init(eWdgIPeriode_Type eWdgPeriode)
{
  eErr_type eReturnCode = eERROR_NOK;

#if (bOPTION_GESTION_WDG == 1)
  // Enabling clock to the interface of the low energy modules (including the Watchdog)
  CMU_ClockEnable(cmuClock_WDOG0, true);
  // Watchdog Initialize settings
  WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
  wdogInit.debugRun = false; // A false signifie qu'en mode debug le watchdog est suspendu
  wdogInit.em2Run = false;
  wdogInit.em3Run = false;
  switch (eWdgPeriode)
  {
  case eWDGI_250MS:
    wdogInit.perSel = wdogPeriod_8k;
    break;
  case eWDGI_500MS:
    wdogInit.perSel = wdogPeriod_16k;
    break;
  case eWDGI_2000MS:
    wdogInit.perSel = wdogPeriod_64k;
    break;
  case eWDGI_1000MS:
  default:
    wdogInit.perSel = wdogPeriod_32k;
    break;
  }
  // Initializing watchdog with chosen settings
  WDOGn_Init(WDOG0, &wdogInit);
  // A ce moment là, le WDGI est enable
#else
  // Paramètre non utilisé
  (void)eWdgPeriode;
#endif
  eReturnCode = eERROR_NONE;
  return (eReturnCode);
}

//-----------------------------------------------------------------------------
/*!
 * \brief Activation du watchdog interne
 * \return eErr_type status du traitement
 */
eErr_type CarteRf_WdgI_Enable(void)
{
  eErr_type eReturnCode = eERROR_NOK;

#if (bOPTION_GESTION_WDG == 1)
  WDOGn_Enable(WDOG0, true);
#endif
  eReturnCode = eERROR_NONE;
  return (eReturnCode);
}

//-----------------------------------------------------------------------------
/*!
 * \brief Désactivation du watchdog interne
 * \return eErr_type status du traitement
 */
eErr_type CarteRf_WdgI_Disable(void)
{
  eErr_type eReturnCode = eERROR_NOK;

#if (bOPTION_GESTION_WDG == 1)
  WDOGn_Unlock(DEFAULT_WDOG);
  WDOGn_Enable(DEFAULT_WDOG, false);
#endif
  eReturnCode = eERROR_NONE;
  return (eReturnCode);
}

//-----------------------------------------------------------------------------
/*!
 * \brief Réarmement du watchdog interne
 * \return eErr_type status du traitement
 */
eErr_type CarteRf_WdgI_Rearme(void)
{
  eErr_type eReturnCode = eERROR_NOK;

#if (bOPTION_GESTION_WDG == 1)
  WDOGn_Feed(WDOG0);
#endif
  eReturnCode = eERROR_NONE;
  return (eReturnCode);
}
