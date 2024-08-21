/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: frdm_stbi_pca9957_shield.h
* @brief The frdm_stbi_pca9957_shield.h file declares arduino pin mapping for frdm_stbi_pca9957_shield expansion board.
*/

#ifndef _FRDM_STBI_PCA9957_SHIELD_H_
#define _FRDM_STBI_PCA9957_SHIELD_H_

/* The shield name */
#define SHIELD_NAME "FRDM-HB2002"

/* Enable HB2002 SPI Read */
#define HB2002_ARD

#define HB2002_CS        D10
#define HB2002_MOSI      D11
#define HB2002_MISO      D12
#define HB2002_SCLK      D13

#define HB2002_IN0       DATA0
#define HB2002_IN1       DATA1

#define HB2002_ENBL      D8
#define HB2002_DIS       D9

#define HB2002_FS_B      FS_B

#endif /* _FRDM_STBI_PCA9957_SHIELD_H_ */
