/*!
* \file   CarteRf_WdgI.h
* \brief  Gestion du watchdog interne
* \author MICROTEC BE
*
*/
/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef _CARTERF_WDGI_H
#define _CARTERF_WDGI_H

//-----------------------------------------------------------------------------
/* Inclusions */
//-----------------------------------------------------------------------------

/* Librairies standards */
#include <stdbool.h>

/* Librairies FOURNISSEUR */

/* Librairies MICROTEC */
#include "MiCommon.h"

//-----------------------------------------------------------------------------
/* Définition de constantes */
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/* Définition de types et structures */
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/*! \enum eWdgIPeriode_Type
 *   \brief Liste des périodes du watchdog
 */
typedef enum {
    eWDGI_250MS = 0,                      /*!< WDG à 250 ms */
    eWDGI_500MS,                          /*!< WDG à 500 ms */
    eWDGI_1000MS,                         /*!< WDG à 1000 ms */
    eWDGI_2000MS,                         /*!< WDG à 2000 ms */
} eWdgIPeriode_Type;

//-----------------------------------------------------------------------------
/* Définition des fonctions et procédures exportes */
//-----------------------------------------------------------------------------
eErr_type CarteRf_WdgI_Init(eWdgIPeriode_Type eWdgPeriode);
eErr_type CarteRf_WdgI_Enable(void);
eErr_type CarteRf_WdgI_Disable(void);
eErr_type CarteRf_WdgI_Rearme(void);

#endif  // _CARTERF_WDGI_H
