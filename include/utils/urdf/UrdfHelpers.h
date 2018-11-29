
#pragma once

#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <LinearMath/btVector3.h>
#include <LinearMath/btAlignedObjectArray.h>

namespace urdf
{

    /******************************************************************
    *   Casting helper
    ******************************************************************/

    template <typename T>
    T urdfLexicalCast(const char* txt)
    {
        double result = atof(txt);
        return result;
    };

    /******************************************************************
    *   String helpers
    ******************************************************************/

    void urdfStringSplit( btAlignedObjectArray<std::string>& pieces, 
                          const std::string& vector_str, 
                          const btAlignedObjectArray<std::string>& separators );

    void urdfIsAnyOf( const char* seps, 
                      btAlignedObjectArray<std::string>& strArray );


    ///The string split C code is by Lars Wirzenius
    ///See http://stackoverflow.com/questions/2531605/how-to-split-a-string-with-a-delimiter-larger-than-one-single-char

    /* Split a string into substrings. Return dynamic array of dynamically
       allocated substrings, or NULL if there was an error. Caller is
       expected to free the memory, for example with str_array_free. */
    char** urdfStrSplit( const char* input, const char* sep );

    /* Append an item to a dynamically allocated array of strings. On failure,
     return NULL, in which case the original array is intact. The item
     string is dynamically copied. If the array is NULL, allocate a new
     array. Otherwise, extend the array. Make sure the array is always
     NULL-terminated. Input string might not be '\0'-terminated. */
    char** urdfStrArrayAppend( char **array, 
                               size_t nitems, 
                               const char *item,
                               size_t itemlen );

    /* Free a dynamic array of dynamic strings. */
    void urdfStrArrayFree( char** array );

    /* Return length of a NULL-delimited array of strings. */
    size_t urdfStrArrayLen( char** array );

    /******************************************************************
    *   Vector-parsing helpers
    ******************************************************************/

    bool parseVector4( btVector4& vec4, 
                       const std::string& vector_str );

    bool parseVector3( btVector3& vec3, 
                       const std::string& vector_str, 
                       bool lastThree = false );

}
