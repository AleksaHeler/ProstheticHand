/** @file btn.h
 *  @brief For defines such as processor type, hw configuration, mechanical configuration...
 *
 *  @todo Aleksa Heler: add comment for this file (what is located here?) and populate this file
 *
 *  @author Aleksa Heler (aleksaheler@gmail.com)
 *  @bug No known bugs.
 */


/********************************************************************************
 *** Includes
 *******************************************************************************/


/********************************************************************************
 *** Defines
 *******************************************************************************/

/**
 * Define wether to write debug data to Serial port
 *
 * @values Comment out this line to disable serial debug
 */
//#define SERIAL_DEBUG


/********************************************************************************
 *** Type defines
 *******************************************************************************/

/**
 * Floating point type defines
 */
#ifdef __cplusplus
  using float32_t = float;
  using float64_t = double;
#else
  typedef float  float32_t;
  typedef double float64_t;
#endif // __cplusplus
