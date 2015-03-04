/* stdbool.h: ISO/IEC 9899:1999 (C99), section 7.16 */

/* Copyright (C) ARM Ltd., 2002
 * All rights reserved
 * RCS $Revision: 172039 $
 * Checkin $Date: 2011-11-02 12:58:12 +0000 (Wed, 02 Nov 2011) $
 * Revising $Author: drodgman $
 */

#ifndef __bool_true_false_are_defined
#define __bool_true_false_are_defined 1
#define __ARMCLIB_VERSION 410000

  #ifndef __cplusplus /* In C++, 'bool', 'true' and 'false' and keywords */
    #define bool char
    #define true 1
    #define false 0
  #else
    #ifdef __GNUC__
      /* GNU C++ supports direct inclusion of stdbool.h to provide C99
         compatibility by defining _Bool */
      #define BOOL bool
    #endif
  #endif

#endif /* __bool_true_false_are_defined */

