/**
 ****************************************************************************************
 *
 * @file user_custs1_def.h
 *
 * @brief Custom Server 1 (CUSTS1) profile database definitions.
 *
 * Copyright (c) 2016-2019 Dialog Semiconductor. All rights reserved.
 *
 * This software ("Software") is owned by Dialog Semiconductor.
 *
 * By using this Software you agree that Dialog Semiconductor retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Dialog Semiconductor is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Dialog Semiconductor products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * DIALOG SEMICONDUCTOR BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 ****************************************************************************************
 */

#ifndef _USER_CUSTS1_DEF_H_
#define _USER_CUSTS1_DEF_H_

/**
 ****************************************************************************************
 * @defgroup USER_CONFIG
 * @ingroup USER
 * @brief Custom Server 1 (CUSTS1) profile database definitions.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "attm_db_128.h"

/*
 * DEFINES
 ****************************************************************************************
 */

// Service 1 of the custom server 1
#if 1

#define DEF_SVC1_UUID_16         				{0x00,0xFF}//{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x00, 0xf0, 0x00, 0xd0}       
#define DEF_SVC1_CTRL_POINT_UUID_16     {0x01,0xFF}//{0x20, 0xEE, 0x8D, 0x0C, 0xE1, 0xF0, 0x4A, 0x0C, 0xB3, 0x25, 0xDC, 0x53, 0x6A, 0x68, 0x86, 0x2D}
#define DEF_SVC1_CTRL_POINT_CHAR_LEN     15
//#define DEF_SVC1_CONTROL_POINT_USER_DESC     "GET"

#define DEF_SVC1_F002_UUID_16     {0x02,0xFF}//{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x02, 0xf0, 0x00, 0xd0}
#define DEF_SVC1_F002_CHAR_LEN     10
//#define DEF_SVC1_F002_USER_DESC     "SET"

#define DEF_SVC1_F003_UUID_16     {0x03,0xFF}//{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x03, 0xf0, 0x00, 0xd0}
#define DEF_SVC1_F003_CHAR_LEN     15
//#define DEF_SVC1_F003_USER_DESC     "BUTTON"

#define DEF_SVC1_F004_UUID_16     {0x04,0xFF}//{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x04, 0xf0, 0x00, 0xd0}
#define DEF_SVC1_F004_CHAR_LEN     10
//#define DEF_SVC1_F004_USER_DESC     "Passkey"

/// Custom1 Service Data Base Characteristic enum
enum
{
    // Custom Service 1
    SVC1_IDX_SVC = 0,

    SVC1_IDX_CONTROL_POINT_CHAR,
    SVC1_IDX_CONTROL_POINT_VAL,
    SVC1_IDX_CONTROL_POINT_NTF_CFG,
    //SVC1_IDX_CONTROL_POINT_USER_DESC,
	
	  SVC1_IDX_F002_CHAR,
    SVC1_IDX_F002_VAL,
		SVC1_IDX_F002_NTF_CFG,
    //SVC1_IDX_F002_USER_DESC,
	
		SVC1_IDX_F003_CHAR,
    SVC1_IDX_F003_VAL,
		SVC1_IDX_F003_NTF_CFG,
    //SVC1_IDX_F003_USER_DESC,
	#if 0
		SVC1_IDX_F004_CHAR,
    SVC1_IDX_F004_VAL,
		SVC1_IDX_F004_NTF_CFG,
    //SVC1_IDX_F004_USER_DESC,
#endif
    CUSTS1_IDX_NB
};

#else

#if 0
#define DEF_SVC1_UUID_128                {0x59, 0x5a, 0x08, 0xe4, 0x86, 0x2a, 0x9e, 0x8f, 0xe9, 0x11, 0xbc, 0x7c, 0x98, 0x43, 0x42, 0x18}

#define DEF_SVC1_CTRL_POINT_UUID_128     {0x20, 0xEE, 0x8D, 0x0C, 0xE1, 0xF0, 0x4A, 0x0C, 0xB3, 0x25, 0xDC, 0x53, 0x6A, 0x68, 0x86, 0x2D}
#else
#define DEF_SVC1_UUID_128         				{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x00, 0xf0, 0x00, 0xd0}       
#define DEF_SVC1_CTRL_POINT_UUID_128     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x01, 0xf0, 0x00, 0xd0}
#endif

#define DEF_SVC1_CTRL_POINT_CHAR_LEN     15
#define DEF_SVC1_CONTROL_POINT_USER_DESC     "GET"

#define DEF_SVC1_F002_UUID_128     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x02, 0xf0, 0x00, 0xd0}
#define DEF_SVC1_F002_CHAR_LEN     10
#define DEF_SVC1_F002_USER_DESC     "SET"

#define DEF_SVC1_F003_UUID_128     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x03, 0xf0, 0x00, 0xd0}
#define DEF_SVC1_F003_CHAR_LEN     15
#define DEF_SVC1_F003_USER_DESC     "BUTTON"

#define DEF_SVC1_F004_UUID_128     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0xa0, 0x01, 0x00, 0x04, 0xf0, 0x00, 0xd0}
#define DEF_SVC1_F004_CHAR_LEN     10
#define DEF_SVC1_F004_USER_DESC     "Passkey"

/// Custom1 Service Data Base Characteristic enum
enum
{
    // Custom Service 1
    SVC1_IDX_SVC = 0,

    SVC1_IDX_CONTROL_POINT_CHAR,
    SVC1_IDX_CONTROL_POINT_VAL,
    SVC1_IDX_CONTROL_POINT_NTF_CFG,
    SVC1_IDX_CONTROL_POINT_USER_DESC,
	
	  SVC1_IDX_F002_CHAR,
    SVC1_IDX_F002_VAL,
		SVC1_IDX_F002_NTF_CFG,
    SVC1_IDX_F002_USER_DESC,
	
		SVC1_IDX_F003_CHAR,
    SVC1_IDX_F003_VAL,
		SVC1_IDX_F003_NTF_CFG,
    SVC1_IDX_F003_USER_DESC,
	
		SVC1_IDX_F004_CHAR,
    SVC1_IDX_F004_VAL,
		SVC1_IDX_F004_NTF_CFG,
    SVC1_IDX_F004_USER_DESC,

    CUSTS1_IDX_NB
};
#endif
/// @} USER_CONFIG

#endif // _USER_CUSTS1_DEF_H_
