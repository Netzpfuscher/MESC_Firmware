/*
 **
 ******************************************************************************
 * @file           : MESCBLDC.h
 * @brief          : BLDC running code
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 David Molony.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************

 * MESCBLDC.h
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */

// fixme: what do functions in this file do? Need a description and perhaps a
// rename of the file to make it obvious.

// SRM: I moved everything that does not need to be exposed to outside world to
// .c file. This way you know what is accessible and what is not.

#ifndef INC_MESCBLDC_H_
#define INC_MESCBLDC_H_

/* Function prototypes -----------------------------------------------*/
void motorInit();
void motorCommuteHall();
void motorCurrentController();

#endif /* INC_MESCBLDC_H_ */
