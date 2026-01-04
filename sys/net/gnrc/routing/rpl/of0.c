/*
 * Copyright (C) 2014 Oliver Hahm <oliver. hahm@inria.fr>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_gnrc_rpl
 * @{
 * @file
 * @brief       Objective Function Zero with Battery-Aware Routing Extension.
 *
 * Implementation of Objective Function Zero (OF0) as defined in RFC 6552.
 * This implementation includes a battery-aware extension that modifies rank
 * calculation based on the node's battery state, enabling energy-efficient
 * routing in Low-Power and Lossy Networks (LLNs).
 *
 * MODIFICATION SUMMARY (CSE464 - Task Group 1):
 * ---------------------------------------------
 * The standard OF0 calculates rank as:
 *     rank = parent_rank + min_hop_rank_inc
 *
 * The battery-aware modification calculates rank as:
 *     - Normal state:    rank = parent_rank + min_hop_rank_inc
 *     - Critical state:  rank = parent_rank + (link_metric * min_hop_rank_inc)
 *
 * When a node's battery is critically low, its rank increases significantly,
 * causing other nodes to prefer alternate parents. This preserves battery
 * life on low-energy nodes by reducing their routing responsibilities.
 *
 * REFERENCES:
 * - RFC 6550: RPL Protocol Specification (https://datatracker.ietf.org/doc/html/rfc6550)
 * - RFC 6552: OF0 Specification (https://datatracker.ietf.org/doc/html/rfc6552)
 * - RIOT GNRC RPL:  https://api.riot-os. org/group__net__gnrc__rpl.html
 *
 * @author      Eric Engel <eric.engel@fu-berlin.de>
 * @author      CSE464 Team - Member 2 (The Developer)
 * @}
 */

#include <string.h>
#include "of0.h"
#include "net/gnrc/rpl. h"
#include "net/gnrc/rpl/structs.h"

/* ============================================================================
 * CONSTANTS
 * ============================================================================ */

#define TRUE  (1)
#define FALSE (0)

/* ============================================================================
 * BATTERY-AWARE ROUTING STATE
 * ============================================================================
 *
 * The is_critical flag is the interface between:
 *   - Member 1 (Analyst): Battery monitoring logic that SETS this flag
 *   - Member 2 (Developer): Rank calculation that READS this flag
 *
 * When DEBUGGING is enabled (in of0.h), a local variable is used for testing.
 * When DEBUGGING is disabled, the variable must be defined by the battery
 * monitoring module (Member 1's responsibility).
 *
 * ============================================================================ */

#if DEBUGGING
    /**
     * @brief   Local is_critical variable for testing purposes
     *
     * Set to TRUE to simulate battery-critical state (high rank penalty)
     * Set to FALSE to simulate normal battery state (standard OF0)
     */
    bool is_critical = TRUE;
#endif /* DEBUGGING */

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

static uint16_t calc_rank(gnrc_rpl_dodag_t *, uint16_t);
static int parent_cmp(gnrc_rpl_parent_t *, gnrc_rpl_parent_t *);
static int which_dodag(gnrc_rpl_dodag_t *, gnrc_rpl_dio_t *);
static void reset(gnrc_rpl_dodag_t *);

/* ============================================================================
 * OBJECTIVE FUNCTION STRUCTURE
 * ============================================================================ */

/**
 * @brief   OF0 Objective Function structure
 *
 * Registered with the RPL stack via gnrc_rpl_get_of0().
 * OCP (Objective Code Point) = 0x0 identifies this as OF0 per RFC 6552.
 */
static gnrc_rpl_of_t gnrc_rpl_of0 = {
    .ocp                    = 0x0,          /**< Objective Code Point for OF0 */
    .calc_rank              = calc_rank,    /**< Rank calculation function */
    . parent_cmp             = parent_cmp,   /**< Parent comparison function */
    .which_dodag            = which_dodag,  /**< DODAG comparison function */
    . reset                  = reset,        /**< Reset function */
    . parent_state_callback  = NULL,         /**< Parent state callback (unused) */
    .init                   = NULL,         /**< Initialization function (unused) */
    .process_dio            = NULL          /**< DIO processing callback (unused) */
};

/**
 * @brief   Get pointer to the OF0 objective function
 *
 * @return  Pointer to gnrc_rpl_of0 structure
 */
gnrc_rpl_of_t *gnrc_rpl_get_of0(void)
{
    return &gnrc_rpl_of0;
}

/**
 * @brief   Reset the OF0 state
 *
 * @param[in] dodag     Pointer to the DODAG (unused in OF0)
 *
 * @note    OF0 maintains no internal state, so this function is empty.
 */
void reset(gnrc_rpl_dodag_t *dodag)
{
    /* Nothing to do in OF0 */
    (void) dodag;
}

/* ============================================================================
 * RANK CALCULATION - BATTERY-AWARE MODIFICATION
 * ============================================================================ */

/**
 * @brief   Calculate node rank with battery-aware optimization
 *
 * This function implements the core battery-aware routing logic.  It calculates
 * the node's rank based on the parent's rank and adjusts the rank increase
 * depending on the battery state.
 *
 * ALGORITHM:
 * ----------
 * 1. If base_rank is 0, use the preferred parent's rank
 * 2. Calculate rank addition based on battery state:
 *
 *    NORMAL MODE (is_critical == FALSE):
 *        add = min_hop_rank_inc
 *        (Standard OF0 behavior per RFC 6552)
 *
 *    CRITICAL MODE (is_critical == TRUE):
 *        add = link_metric * min_hop_rank_inc
 *        (Battery-aware extension - higher rank = less preferred)
 *
 * 3. Check for overflow and return final rank
 *
 * EXAMPLE:
 * --------
 * With min_hop_rank_inc = 256 and link_metric = 2. 0:
 *   - Normal mode:    add = 256         → Lower rank, preferred as parent
 *   - Critical mode:  add = 2. 0 * 256 = 512 → Higher rank, avoided as parent
 *
 * @param[in] dodag         Pointer to the DODAG structure
 * @param[in] base_rank     Base rank for calculation (0 = use parent's rank)
 *
 * @return  Calculated rank value
 * @return  GNRC_RPL_INFINITE_RANK if no parents available or overflow detected
 *
 * @see     RFC 6550 Section 3.5.1 (Rank Properties)
 * @see     RFC 6552 Section 4.1 (OF0 Rank Calculation)
 */
uint16_t calc_rank(gnrc_rpl_dodag_t *dodag, uint16_t base_rank)
{
    /*
     * Step 1: Determine base rank
     * If base_rank is 0, use the preferred parent's rank as the base.
     */
    if (base_rank == 0) {
        if (dodag->parents == NULL) {
            /* No parents available - cannot calculate rank */
            return GNRC_RPL_INFINITE_RANK;
        }
        base_rank = dodag->parents->rank;
    }

    /*
     * Step 2: Calculate rank addition based on battery state
     * This is where the battery-aware modification takes effect.
     */
    uint16_t add;

    if (dodag->parents != NULL) {
        if (is_critical) {
            /*
             * BATTERY CRITICAL MODE:
             * ----------------------
             * When the node's battery is critically low, we increase the rank
             * by multiplying link_metric with min_hop_rank_inc.
             *
             * Effect: Higher rank makes this node less attractive as a parent,
             * causing child nodes to select alternate parents with lower ranks.
             * This reduces routing load on the low-battery node.
             *
             * Formula: add = link_metric * min_hop_rank_inc
             */
            add = (uint16_t)(dodag->parents->link_metric * dodag->instance->min_hop_rank_inc);
        }
        else {
            /*
             * NORMAL MODE (Standard OF0):
             * ---------------------------
             * When battery is normal, use the standard OF0 rank calculation
             * as defined in RFC 6552.
             *
             * Formula:  add = min_hop_rank_inc (typically 256)
             */
            add = dodag->instance->min_hop_rank_inc;
        }
    }
    else {
        /*
         * No parent available - use default minimum hop rank increase.
         * This is a fallback for edge cases.
         */
        add = CONFIG_GNRC_RPL_DEFAULT_MIN_HOP_RANK_INCREASE;
    }

    /*
     * Step 3: Overflow check
     * Per RFC 6550 Section 3.5.1, rank must strictly increase downward.
     * If addition would cause overflow, return INFINITE_RANK.
     */
    if ((uint16_t)(base_rank + add) < base_rank) {
        return GNRC_RPL_INFINITE_RANK;
    }

    /* Return final calculated rank */
    return base_rank + add;
}

/* ============================================================================
 * PARENT COMPARISON
 * ============================================================================ */

/**
 * @brief   Compare two parents to determine preference
 *
 * This function is used by the RPL stack to order the parent list.
 * Parents with lower rank are preferred (appear first in the list).
 *
 * @param[in] parent1   First parent to compare
 * @param[in] parent2   Second parent to compare
 *
 * @return  -1 if parent1 is preferred (lower rank)
 * @return   1 if parent2 is preferred (lower rank)
 * @return   0 if both parents have equal rank
 */
int parent_cmp(gnrc_rpl_parent_t *parent1, gnrc_rpl_parent_t *parent2)
{
    if (parent1->rank < parent2->rank) {
        return -1;  /* parent1 preferred */
    }
    else if (parent1->rank > parent2->rank) {
        return 1;   /* parent2 preferred */
    }
    return 0;       /* equal preference */
}

/* ============================================================================
 * DODAG COMPARISON
 * ============================================================================ */

/**
 * @brief   Compare current DODAG with a DODAG advertised in a DIO message
 *
 * This function implements DODAG selection criteria as per RFC 6552 Section 4.2.
 * It determines whether to stay with the current DODAG or switch to the one
 * advertised in the received DIO message.
 *
 * SELECTION CRITERIA (in order of priority):
 * 1. Parent set must not be empty
 * 2. Prefer grounded DODAG
 * 3. Prefer DODAG with higher preference (prf)
 * 4. Prefer DODAG with more recent version
 * 5. Prefer DODAG with lower resulting rank
 * 6. Prefer DODAG with alternate parents available
 *
 * @param[in] d1    Current DODAG
 * @param[in] dio   Received DIO message containing advertised DODAG info
 *
 * @return  -1 if current DODAG (d1) is preferred
 * @return   1 if DIO's DODAG is preferred
 * @return   0 if equal preference
 *
 * @see     RFC 6552 Section 4.2 (DODAG Selection)
 */
int which_dodag(gnrc_rpl_dodag_t *d1, gnrc_rpl_dio_t *dio)
{
    /* RFC 6552, Section 4.2 */

    /* Criterion 1: Parent set must not be empty */
    if ((d1->node_status != GNRC_RPL_ROOT_NODE) && !d1->parents) {
        return 1;   /* Prefer DIO's DODAG - current has no parents */
    }

    /* Criterion 2: Prefer grounded DODAG */
    int dio_grounded = dio->g_mop_prf >> GNRC_RPL_GROUNDED_SHIFT;
    if (d1->grounded > dio_grounded) {
        return -1;  /* Current DODAG is grounded, prefer it */
    }
    else if (dio_grounded > d1->grounded) {
        return 1;   /* DIO's DODAG is grounded, prefer it */
    }

    /* Criterion 3: Prefer DODAG with higher preference */
    int dio_prf = dio->g_mop_prf & GNRC_RPL_PRF_MASK;
    if (d1->prf > dio_prf) {
        return -1;  /* Current DODAG has higher preference */
    }
    else if (dio_prf > d1->prf) {
        return 1;   /* DIO's DODAG has higher preference */
    }

    /* Criterion 4: Prefer DODAG with more recent version */
    if (ipv6_addr_equal(&d1->dodag_id, &dio->dodag_id)) {
        if (GNRC_RPL_COUNTER_GREATER_THAN(d1->version, dio->version_number)) {
            return -1;  /* Current version is newer */
        }
        else if (GNRC_RPL_COUNTER_GREATER_THAN(dio->version_number, d1->version)) {
            return 1;   /* DIO's version is newer */
        }
    }

    /* Criterion 5: Prefer DODAG with lower resulting rank */
    int d1_rank = d1->parents->rank;
    int d2_rank = byteorder_ntohs(dio->rank);
    if (d1_rank < d2_rank) {
        return -1;  /* Current DODAG has lower rank */
    }
    else if (d2_rank < d1_rank) {
        return 1;   /* DIO's DODAG has lower rank */
    }

    /* Criterion 6: Prefer DODAG with alternate parents */
    if (d1->parents->next) {
        return -1;  /* Current DODAG has alternate parents */
    }

    return 0;   /* Equal preference */
}
