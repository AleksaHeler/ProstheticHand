/**
 * @file defines.h
 *
 * @author Aleksa Heler (aleksaheler@gmail.com)
 *
 * @brief For defines such as processor type, hardware configuration, mechanical configuration...
 *
 * @todo Aleksa Heler: add comment for this file (what is located here?) and populate this file
 *
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DEFINES_H
#define DEFINES_H

/**************************************************************************
 * Includes
 **************************************************************************/

/**************************************************************************
 * Defines
 **************************************************************************/

/**
 * @brief Define whether to write debug data to Serial port
 * Communication over the serial port can take a _very_ long time, thus slowing down the whole program.
 *
 * @values Comment out the line to disable serial debug
 *
 */
#define SERIAL_DEBUG

#define MILLISEC_TO_MICROSEC 1000

/**************************************************************************
 * Type defines
 **************************************************************************/

/**
 * @brief Floating point type defines
 *
 */
#ifdef __cplusplus
using float32_t = float;
using float64_t = double;
#else
typedef float float32_t;
typedef double float64_t;
#endif // __cplusplus

#endif // DEFINES_H