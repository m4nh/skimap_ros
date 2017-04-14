/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAM_DUNK_PRETTY_PRINTER_H
#define SLAM_DUNK_PRETTY_PRINTER_H

#ifdef _MSC_VER
#define SLAM_DUNK_FUNCTION_STR __FUNCSIG__
#else
#ifdef __GNUC__
#define SLAM_DUNK_FUNCTION_STR __PRETTY_FUNCTION__
#else
#define SLAM_DUNK_FUNCTION_STR __FUNCTION__
#endif
#endif

// std logging string
#define SLAM_DUNK_DEBUG_STR(s) "\033[1;34m[DEBUG]\033[0m" \
                                   << "\033[1m[" << SLAM_DUNK_FUNCTION_STR << "]\033[0m\n\t" << s
#define SLAM_DUNK_INFO_STR(s) "\033[1;32m[INFO] \033[0m" \
                                  << "\033[1m[" << SLAM_DUNK_FUNCTION_STR << "]\033[0m\n\t" << s
#define SLAM_DUNK_WARNING_STR(s) "\033[1;33m[WARN] \033[0m" \
                                     << "\033[1m[" << SLAM_DUNK_FUNCTION_STR << "]\033[0m\n\t" << s
#define SLAM_DUNK_ERROR_STR(s) "\033[1;31m[ERROR]\033[0m" \
                                   << "\033[1m[" << SLAM_DUNK_FUNCTION_STR << "]\033[0m\n\t" << s
#define SLAM_DUNK_FATAL_STR(s) "\033[1;31m[FATAL]\033[0m" \
                                   << "\033[1m[" << SLAM_DUNK_FUNCTION_STR << "]\033[0m\n\t" << s

#endif // SLAM_DUNK_PRETTY_PRINTER_H
