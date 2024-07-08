/*!
 * \file	MiCommon.h
 * \brief	Gestion des éléments commun aux logiciels MICROTEC
 * \author	MICROTEC BE
 *
 */
#ifndef _MICROTEC_COMMON_H       /* Guard against multiple inclusion */
#define _MICROTEC_COMMON_H
//-----------------------------------------------------------------------------
/* Inclusions */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

//-----------------------------------------------------------------------------

/* Librairies standards */

/* Librairies FOURNISSEUR */

/* Librairies MICROTEC */

/* Librairies spécifiques à l'application */

//-----------------------------------------------------------------------------
/* Définition de constantes */
//-----------------------------------------------------------------------------

/*! \def sERR_LIBELLE_NON_IMPLEMENTE
 *	\brief Libellé d'une erreur non implémentée */
#define sERR_LIBELLE_NON_IMPLEMENTE             "Erreur non implémentée"

/*! \def SI_ERREUR_RETURN
 *	\brief Macro pour mémoriser l'erreur locale et retourner l'erreur générique */
#define SI_ERREUR_RETURN(LocalErr, GenErr) {                               \
        if (GenErr != eERROR_NONE) {                                        \
            Gui8LocalError = LocalErr;                                      \
            return(GenErr);                                                 \
        }                                                                   \
}

/*! \def SI_ERREUR_BREAK
 *	\brief Macro pour tester l'erreur et realiser un break */
#define SI_ERREUR_BREAK(GenErr) {                                          \
        if (GenErr != eERROR_NONE) {                                        \
            break;                                                          \
        }                                                                   \
}

/*! \def PACKSTRUCT
 *  \brief Macro pour aligner une structure sur 1 byte */
#ifdef __GNUC__
#define PACKSTRUCT(decl) decl __attribute__((packed, aligned(1)))
#else
#define PACKSTRUCT(a) a
#endif

//-----------------------------------------------------------------------------
/* Définition de types et structures */
//-----------------------------------------------------------------------------

/*! \enum eErr_type
 *   \brief Enuméré listant toutes les erreurs génériques
 */
typedef enum {
    eERROR_NONE          =  0,  /*!< Pas d'erreur */
    eERROR_NOK           = -1,  /*!< Erreur générique */
    eERROR_INVALID_PARAM = -2,  /*!< Erreur de paramètre invalide */
    eERROR_OTHER         = -3,   /*!*< Autre erreur spécifique à la librairie: voir fonction "Lib_GetErrorString(...)" >*/
    eERROR_TIME_OUT      = -4   /*!*< Erreur de tiemout */
}eErr_type;

/*! \enum eActionMe_Type
 *   \brief Liste des actions dans une Machine à états (Me)
 */
typedef enum {
    eACTION_EN_ENTREE = 0,                  /*!< Action en entrée d'un état de la Me */
    eACTION_EN_BOUCLE = 1,                  /*!< Action en boucle d'un état de la Me */
    eACTION_EN_SORTIE = 2,                  /*!< Action en sortie d'un état de la Me */
    eNO_ACTION                              /*!< Pas d'action à exécuter dans un état de la Me */
} eActionMe_Type;

//-----------------------------------------------------------------------------
/* Définition des fonctions et procédures exportées */
//-----------------------------------------------------------------------------


#endif /* _MICROTEC_COMMON_H */
//-----------------------------------------------------------------------------
/*!
   Fin du fichier
 */
