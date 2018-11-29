
#include <utils/urdf/UrdfHelpers.h>


namespace urdf
{

    /******************************************************************
    *   String helpers
    ******************************************************************/

    void urdfStringSplit( btAlignedObjectArray<std::string> &pieces, 
                          const std::string &vector_str, 
                          const btAlignedObjectArray<std::string> &separators )
    {
        assert(separators.size() == 1);
        if (separators.size() == 1)
        {
            char **strArray = urdfStrSplit(vector_str.c_str(), separators[0].c_str());
            int numSubStr = urdfStrArrayLen(strArray);
            for (int i = 0; i < numSubStr; i++)
                pieces.push_back(std::string(strArray[i]));
            urdfStrArrayFree(strArray);
        }
    }

    void urdfIsAnyOf( const char *seps, 
                      btAlignedObjectArray<std::string> &strArray )
    {
        int numSeps = strlen(seps);
        for (int i = 0; i < numSeps; i++)
        {
            char sep2[2] = {0, 0};

            sep2[0] = seps[i];
            strArray.push_back(sep2);
        }
    }

    /* Append an item to a dynamically allocated array of strings. On failure,
     return NULL, in which case the original array is intact. The item
     string is dynamically copied. If the array is NULL, allocate a new
     array. Otherwise, extend the array. Make sure the array is always
     NULL-terminated. Input string might not be '\0'-terminated. */
    char** urdfStrArrayAppend( char **array, 
                               size_t nitems, 
                               const char *item,
                               size_t itemlen )
    {
        /* Make a dynamic copy of the item. */
        char *copy;
        if (item == NULL)
            copy = NULL;
        else
        {
            copy = (char *)malloc(itemlen + 1);
            if (copy == NULL)
                return NULL;
            memcpy(copy, item, itemlen);
            copy[itemlen] = '\0';
        }

        /* Extend array with one element. Except extend it by two elements,
         in case it did not yet exist. This might mean it is a teeny bit
         too big, but we don't care. */
        array = (char **)realloc(array, (nitems + 2) * sizeof(array[0]));
        if (array == NULL)
        {
            free(copy);
            return NULL;
        }

        /* Add copy of item to array, and return it. */
        array[nitems] = copy;
        array[nitems + 1] = NULL;
        return array;
    }

    /* Free a dynamic array of dynamic strings. */
    void urdfStrArrayFree(char **array)
    {
        if (array == NULL)
            return;
        for (size_t i = 0; array[i] != NULL; ++i)
            free(array[i]);
        free(array);
    }

    /* Split a string into substrings. Return dynamic array of dynamically
     allocated substrings, or NULL if there was an error. Caller is
     expected to free the memory, for example with str_array_free. */
    char** urdfStrSplit( const char *input, const char *sep )
    {
        size_t nitems = 0;
        char **array = NULL;
        const char *start = input;
        const char *next = strstr(start, sep);
        size_t seplen = strlen(sep);
        const char *item;
        size_t itemlen;

        for (;;)
        {
            next = strstr(start, sep);
            if (next == NULL)
            {
                /* Add the remaining string (or empty string, if input ends with
                 separator. */
                char **newstr = urdfStrArrayAppend(array, nitems, start, strlen(start));
                if (newstr == NULL)
                {
                    urdfStrArrayFree(array);
                    return NULL;
                }
                array = newstr;
                ++nitems;
                break;
            }
            else if (next == input)
            {
                /* Input starts with separator. */
                item = "";
                itemlen = 0;
            }
            else
            {
                item = start;
                itemlen = next - item;
            }
            char **newstr = urdfStrArrayAppend(array, nitems, item, itemlen);
            if (newstr == NULL)
            {
                urdfStrArrayFree(array);
                return NULL;
            }
            array = newstr;
            ++nitems;
            start = next + seplen;
        }

        if (nitems == 0)
        {
            /* Input does not contain separator at all. */
            assert(array == NULL);
            array = urdfStrArrayAppend(array, nitems, input, strlen(input));
        }

        return array;
    }

    /* Return length of a NULL-delimited array of strings. */
    size_t urdfStrArrayLen(char **array)
    {
        size_t len;

        for ( len = 0; array[len] != NULL; ++len )
            continue;
        return len;
    }

    /******************************************************************
    *   Vector-parsing helpers
    ******************************************************************/

    bool parseVector4( btVector4& vec4, const std::string& vector_str )
    {
        vec4.setZero();
        btAlignedObjectArray<std::string> pieces;
        btAlignedObjectArray<float> rgba;
        btAlignedObjectArray<std::string> strArray;
        urdfIsAnyOf(" ", strArray);
        urdfStringSplit(pieces, vector_str, strArray);
        for (int i = 0; i < pieces.size(); ++i)
        {
            if (!pieces[i].empty())
            {
                rgba.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
            }
        }
        if (rgba.size() != 4)
        {
            return false;
        }
        vec4.setValue(rgba[0], rgba[1], rgba[2], rgba[3]);
        return true;
    }

    bool parseVector3( btVector3& vec3, 
                       const std::string& vector_str,
                       bool lastThree )
    {
        vec3.setZero();
        btAlignedObjectArray<std::string> pieces;
        btAlignedObjectArray<float> rgba;
        btAlignedObjectArray<std::string> strArray;
        urdfIsAnyOf(" ", strArray);
        urdfStringSplit(pieces, vector_str, strArray);
        for (int i = 0; i < pieces.size(); ++i)
        {
            if (!pieces[i].empty())
            {
                rgba.push_back(urdfLexicalCast<double>(pieces[i].c_str()));
            }
        }
        if (rgba.size() < 3)
        {
            std::cout << "ERROR> Couldn't parse vector3" << std::endl;
            return false;
        }
        if (lastThree)
        {
            vec3.setValue(rgba[rgba.size() - 3], rgba[rgba.size() - 2], rgba[rgba.size() - 1]);
        }
        else
        {
            vec3.setValue(rgba[0], rgba[1], rgba[2]);
        }
        return true;
    }

}