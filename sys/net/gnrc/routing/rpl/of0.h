/*
 * Copyright (C) 2014 Oliver Hahm <oliver.hahm@inria.fr>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#pragma once

/**
 * @ingroup     net_gnrc_rpl
 * @{
 * @file
 * @brief       Objective Function Zero.
 *
 * Header-file, which defines all functions for the implementation of Objective Function Zero.
 *
 * @author      Eric Engel <eric.engel@fu-berlin.de>
 */

#include "net/gnrc/rpl/structs.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 * ============================================================================
 * DEBUGGING / INTEGRATION CONFIGURATION
 * ============================================================================
 *
 * Set DEBUGGING to 1 for standalone testing (uses local is_critical variable)
 * Set DEBUGGING to 0 for integration with battery monitoring module
 *
 * When DEBUGGING == 0:
 *   - The battery monitoring team (Member 1) must define is_critical variable
 *   - Include this header in your battery monitoring module
 *   - Update is_critical based on battery level thresholds
 *
 * ============================================================================
 */
#define DEBUGGING (1)

/**
 * @brief   Battery critical state flag for RPL rank calculation
 *
 * This flag controls the behavior of the calc_rank() function in of0.c:
 *
 * - When TRUE:   Node is in battery-critical state
 *               Rank is increased using:  link_metric * min_hop_rank_inc
 *               This makes the node less attractive as a routing parent
 *
 * - When FALSE: Node has normal battery level
 *               Rank uses standard OF0: min_hop_rank_inc
 *
 * OWNERSHIP:
 *   - Member 1 (The Analyst): Controls this variable via battery monitoring logic
 *   - Member 2 (The Developer): Uses this variable in calc_rank() function
 *
 * INTEGRATION:
 *   When battery monitoring code is ready, set DEBUGGING to 0 and define
 *   is_critical in your battery monitoring module.
 */
#if DEBUGGING
    extern bool is_critical;
#else
    extern volatile bool is_critical;
#endif /* DEBUGGING */


/**
 * @brief   Return the address to the of0 objective function
 *
 * @return  Address of the of0 objective function
 */
gnrc_rpl_of_t *gnrc_rpl_get_of0(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
