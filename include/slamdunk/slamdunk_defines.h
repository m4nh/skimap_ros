/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAM_DUNK_SLAMDUNK_DEFINES_H
#define SLAM_DUNK_SLAMDUNK_DEFINES_H

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the SLAM_DUNK_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// SEBA_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef _MSC_VER
#ifdef SLAM_DUNK_EXPORTS
#define SLAM_DUNK_API __declspec(dllexport)
#else
#define SLAM_DUNK_API __declspec(dllimport)
#endif
#else
#define SLAM_DUNK_API
#endif

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "windows.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#endif

#if defined(_MSC_VER) && _MSC_VER <= 1600 // 1400 == VC++ 8.0, 1600 == VC++ 10.0
#pragma warning(disable : 4251 4231 4660)
#endif

// Automatically generated part
//#define SLAMDUNK_TIMERS_ENABLED false
#define SLAMDUNK_FLANN_ENABLED
#define SLAMDUNK_ICP_ENABLED

#endif // SLAM_DUNK_SLAMDUNK_DEFINES_H
