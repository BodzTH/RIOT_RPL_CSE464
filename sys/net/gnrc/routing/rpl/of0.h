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
 * @brief       Objective Function Zero with Data Priority-Aware Routing Extension.
 *
 * Header file which defines all functions for the implementation of Objective Function Zero.
 * This implementation includes a data priority-aware extension that modifies rank
 * calculation based on sensor data criticality (moisture and temperature readings).
 *
 * @author      Eric Engel <eric.engel@fu-berlin.de>
 * @author      CSE464 Team - Task Group 1 (RPL Objective Function Optimization)
 */

#include "net/gnrc/rpl/structs.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief   Data priority critical state flag for RPL rank calculation
 *
 * This flag controls the behavior of the calc_rank() function in of0.c.
 * It is SET by Team A based on sensor data priority logic:
 *
 * DATA PRIORITY LOGIC (controlled by Team A):
 * -------------------------------------------
 *
 *   MOISTURE LEVEL:
 *     - Critical (<20% or >80%)  -> High Priority Alert     -> is_critical = TRUE
 *     - Normal (20% - 80%)       -> Normal Priority         -> is_critical = FALSE
 *
 *   TEMPERATURE LEVEL:
 *     - Critical (<10°C or >35°C) -> Urgent Priority (ACK needed) -> is_critical = TRUE
 *     - Normal (10°C - 35°C)      -> Normal Priority              -> is_critical = FALSE
 *
 * EFFECT ON ROUTING:
 * ------------------
 * - When TRUE:   Critical sensor data detected, use link_metric * min_hop_rank_inc
 *               for more reliable routing of high-priority data
 * - When FALSE: Normal sensor data, use standard OF0 rank calculation
 *
 * OWNERSHIP:
 *   - Team A (Data Priority Team): Controls this variable based on sensor readings
 *   - Member 2 (The Developer): Uses this variable in calc_rank() function
 */
extern volatile bool is_critical;

/**
 * @brief   Return the address to the OF0 objective function
 *
 * @return  Address of the OF0 objective function
 */
gnrc_rpl_of_t *gnrc_rpl_get_of0(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
