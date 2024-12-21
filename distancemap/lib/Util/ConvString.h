#ifndef UTIL_CONV_STRING_H
#define UTIL_CONV_STRING_H

#include <exception>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <climits>
#include <algorithm>
#include <iterator>
#include <cctype>

// NOTE: If include Debug.h it uses splitString so would need forward dec

    //
    // YES, this is a mess. Some of the functions are useful, but no one else can find them :P
    //
namespace Util {

    // List of whitespace characters
    const char* const WS_CHARS=" \t\n\r";

///////////////////////////////////////////////////////////////////////

inline
char lastchar(const std::string& s)
{
    return (s.empty() ? '\0' : s[s.length()-1]);
}

///////////////////////////////////////////////////////////////////////

inline
bool startswith(const std::string& s, const std::string& e)
{
    return (s.size() >= e.size()) and (e == s.substr(0,e.size()));
}

///////////////////////////////////////////////////////////////////////

inline
bool endswith(const std::string& s, const std::string& e)
{
    return (s.size() >= e.size()) and (e == s.substr(s.size()-e.size()));
}

///////////////////////////////////////////////////////////////////////

inline
std::string shorten(const std::string& s, unsigned int len)
{
    return (s.size() >= len) ? s.substr(0,s.size()-len) : "";
}

///////////////////////////////////////////////////////////////////////

inline
std::string shortenFront(const std::string& s, unsigned int len)
{
    return (s.size() >= len) ? s.substr(len) : "";
}

///////////////////////////////////////////////////////////////////////

inline
std::string upper(const std::string& s)
{
    std::string l_retStr(s.length(),' ');
    std::transform(s.begin(), s.end(), std::back_inserter(l_retStr), ::toupper);
    return l_retStr;
}

inline
std::string lower(const std::string& s)
{
    std::string l_retStr(s.length(),' ');
    std::transform(s.begin(), s.end(), std::back_inserter(l_retStr), ::tolower);
    return l_retStr;
}

///////////////////////////////////////////////////////////////////////

// NOTE: This will work for "123XXXX" and return "123"
// Could fail if l_iss.eof() is false (should probably rtrim then as well)
template<typename NumType>
NumType strToNum(const std::string& rS, int base)
{
    NumType value = 0;
    // Allow for empty so can have optional CFG numbers
    // (well actually coz of TIMER1_SAMPLES_SIZE in InitLogic)
    if (! rS.empty())
    {
        std::istringstream l_iss(rS);
        l_iss >> std::setbase(base) >> value;
        if (l_iss.fail())
        {
            throw (std::domain_error("Cant convert String to Num:"));
        }
    }
    return value;
}
template<typename NumType>
NumType strToNum(const std::string& rS)
{
    return strToNum<NumType>(rS, (Util::startswith(rS,"0x") ? 16 : 10));
}


inline long decStrToLong(const std::string& rS)           { return strToNum<long>(rS, 10); }
inline long hexStrToLong(const std::string& rS)           { return strToNum<long>(rS, 16); }

inline unsigned long decStrToULong(const std::string& rS) { return strToNum<unsigned long>(rS, 10); }
inline unsigned long hexStrToULong(const std::string& rS) { return strToNum<unsigned long>(rS, 16); }

inline int decStrToInt(const std::string& rS)             { return strToNum<int>(rS, 10); }
inline int hexStrToInt(const std::string& rS)             { return strToNum<int>(rS, 16); }

inline int decStrToShort(const std::string& rS)           { return strToNum<short>(rS, 10); }
inline int hexStrToShort(const std::string& rS)           { return strToNum<short>(rS, 16); }

inline unsigned int decStrToUInt(const std::string& rS)   { return strToNum<unsigned int>(rS, 10); }
inline unsigned int hexStrToUInt(const std::string& rS)   { return strToNum<unsigned int>(rS, 16); }

inline float decStrToFloat(const std::string& rS)         { return strToNum<float>(rS, 10); }
inline float decStrToDouble(const std::string& rS)        { return strToNum<double>(rS, 10); }

///////////////////////////////////////////////////////////////////////

inline
std::string rtrim(const std::string& rS, const char* pTrimChars=WS_CHARS)
{
    std::string::size_type l_nonWS = rS.find_last_not_of(pTrimChars);
    if (l_nonWS == std::string::npos)
    {
        return std::string();
    }
    else
    if ((l_nonWS+1) < rS.size())
    {
        return std::string(rS, 0, l_nonWS+1);
    }
    return rS;
}

inline
std::string ltrim(const std::string& rS, const char* pTrimChars=WS_CHARS)
{
    std::string::size_type l_nonWS = rS.find_first_not_of(pTrimChars);
    if (l_nonWS == std::string::npos)
    {
        return std::string();
    }
    return std::string(rS, l_nonWS);
}

inline
std::string trim(const std::string& rS, const char* pTrimChars=WS_CHARS)
{
    return rtrim(ltrim(rS, pTrimChars), pTrimChars);
}

///////////////////////////////////////////////////////////////////////

//
// Return the input string left/right padded to the length (or unchanged if longer)
//
inline
std::string lpad(unsigned int padLen, const std::string& s, char padChr=' ')
{
    return (s.size() < padLen) ? std::string(padLen-s.size(),padChr)+s : s;
}
inline
std::string rpad(unsigned int padLen, const std::string& s, char padChr=' ')
{
    return (s.size() < padLen) ? s+std::string(padLen-s.size(),padChr) : s;
}
// Pads the input to the new length with padding at start/end
inline
std::string cpad(unsigned int padLen, const std::string& s, char padChr=' ')
{
    const int rpadLen =(int(padLen)-int(s.size()))/2 + (int(padLen)-int(s.size()))%2;
    const int lpadLen =(int(padLen) - int(s.size()))/2;
    return (s.size() < padLen)
        ? std::string(lpadLen,padChr)+s+std::string(rpadLen,padChr)
        : s;
}

///////////////////////////////////////////////////////////////////////

inline
std::string removeUpToLast(const std::string& rS, char c)
{
    std::string::size_type l_cutPos = rS.rfind(c);
    return (l_cutPos != std::string::npos)
        ? std::string(rS).erase(0, l_cutPos+1)
        : rS;
}

inline
std::string removeFromLast(const std::string& rS, char c)
{
    std::string::size_type l_cutPos = rS.rfind(c);
    return (l_cutPos != std::string::npos)
        ? std::string(rS).erase(l_cutPos)
        : rS;
}


inline
std::string removeUpToFirst(const std::string& rS, char c)
{
    std::string::size_type l_cutPos = rS.find(c);
    return (l_cutPos != std::string::npos)
        ? std::string(rS).erase(0, l_cutPos+1)
        : rS;
}

inline
std::string removeFromFirst(const std::string& rS, char c)
{
    std::string::size_type l_cutPos = rS.find(c);
    return (l_cutPos != std::string::npos)
        ? std::string(rS).erase(l_cutPos)
        : rS;
}

///////////////////////////////////////////////////////////////////////

// Could have char versions etc, but never going to use this if need speed
//
inline
std::string replaceAll(const std::string& str, const std::string& findStr, const std::string& withStr)
{
    std::string l_retStr;
    std::string::size_type l_searchPos = 0;
    std::string::size_type l_foundPos = 0;
    const std::string::size_type l_findLen = findStr.length();
    while ( std::string::npos != (l_foundPos = str.find(findStr, l_searchPos) ) )
    {
        // Not sure what STL says about using 0 for substr length. So do a check
        if (l_foundPos > l_searchPos)
        {
            l_retStr += str.substr(l_searchPos,l_foundPos-l_searchPos);
        }
        l_retStr += withStr;
        l_searchPos = l_foundPos+l_findLen;
    }
    l_retStr += str.substr(l_searchPos);
    return l_retStr;
}

// Helper just to make it obvious what it's doing
//
inline
std::string makeSingleLine(const std::string& str)
{
    return replaceAll(str,"\n"," ");
}

///////////////////////////////////////////////////////////////////////

inline
int splitString(const std::string& rString,
                const std::string& rDelim,
                std::vector<std::string>& rSubStrs,
                bool allowBlank=false,
                bool doTrim=true)
{
    std::string::size_type l_maxLen = rString.size();
    std::string::size_type l_delimLen = rDelim.size();
    std::string::size_type l_curPos = 0;
    std::string::size_type l_delimPos;

    do {
        l_delimPos = rString.find(rDelim, l_curPos);
        if (l_delimPos == std::string::npos)
        {
            // No end delimiter, put the remainder of the string on
            // but rString could have been completely empty. If so
            // then only store the blank string if allowed
            if (l_maxLen or allowBlank)
            {
                if (doTrim)
                {
                    rSubStrs.push_back(trim(rString.substr(l_curPos)));
                    if (!allowBlank and rSubStrs.back().empty())
                    {
                        rSubStrs.erase(rSubStrs.end());
                    }
                }
                else
                {
                    rSubStrs.push_back(rString.substr(l_curPos));
                }
            }
        }
        else
        {
            std::string::size_type l_subLen = l_delimPos-l_curPos;
            if (l_subLen or allowBlank)
            {
                if (doTrim)
                {
                    rSubStrs.push_back(trim(rString.substr(l_curPos, l_subLen)));
                    if (!allowBlank and rSubStrs.back().empty())
                    {
                        rSubStrs.erase(rSubStrs.end());
                    }
                }
                else
                {
                    rSubStrs.push_back(rString.substr(l_curPos, l_subLen));
                }
            }
            l_curPos = l_delimPos + l_delimLen;
        }
    } while ((l_curPos < l_maxLen) and (l_delimPos != std::string::npos));

    // If ended with the delimiter then add an empty string
    // (if allowed)
    if (allowBlank and (l_delimPos != std::string::npos))
    {
        rSubStrs.push_back("");
    }
    return rSubStrs.size();
}

///////////////////////////////////////////////////////////////////////

inline
void tokenizeStr(const std::string& str, std::vector<std::string>& rTokens, const std::string& delimiters=WS_CHARS)
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);

    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        std::string token(str.substr(lastPos, pos - lastPos));
        rTokens.push_back(trim(token));

        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);

        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

///////////////////////////////////////////////////////////////////////

// Convert the number to a string in the given base
// padded with zeros to the given width.
// NOTE: Base 16 string will have a "0x" prefix
// NOTE: Only base 8, 10 and 16 supported
// NOTE: unsigned char/char will not work use numToStr<[unsigned] int>
// (Reason is ostringstream << aValue will put a char)
//
template<typename NumType>
std::string numToStr(NumType aValue, int aBase=10, int aWidth=0, int aPrecision=-1)
{
    std::ostringstream oss;
    // Don't want to use fixed if not given precision
    if (aPrecision != -1)
    {
        oss << std::fixed
            << std::setprecision(aPrecision);
    }
    // NOTE: std::setiosflags(std::ios_base::showbase) is useless
    // with setfill e.g. it generates "00000x1" instead of "0x00001"
    // Therefore add the "0x" ourselves
    if (16 == aBase) oss << "0x";

    oss << std::setfill('0') << std::setw(aWidth)
        << std::setbase(aBase)
        << aValue;

    return oss.str();
}

inline std::string ulongToDecStr(unsigned long value, int w=0)   { return numToStr<unsigned long>(value, 10, w); }
inline std::string ushortToDecStr(unsigned short value, int w=0) { return numToStr<unsigned short>(value, 10, w); }

inline std::string longToDecStr(long value, int w=0)             { return numToStr<long>(value, 10, w); }
inline std::string shortToDecStr(short value, int w=0)           { return numToStr<short>(value, 10, w); }

inline std::string ulongToHexStr(unsigned long value, int w=0)   { return numToStr<unsigned long>(value, 16, w); }
inline std::string ushortToHexStr(unsigned short value, int w=0) { return numToStr<unsigned short>(value, 16, w); }

inline std::string longToHexStr(long value, int w=0)             { return numToStr<long>(value, 16, w); }
inline std::string shortToHexStr(short value, int w=0)           { return numToStr<float>(value, 16, w); }

inline std::string intToDecStr(int value, int w=0)               { return numToStr<int>(value, 10, w); }
inline std::string uintToDecStr(unsigned int value, int w=0)     { return numToStr<unsigned int>(value, 10, w); }

inline std::string intToHexStr(int value, int w=0)               { return numToStr<int>(value, 16, w); }
inline std::string uintToHexStr(unsigned int value, int w=0)     { return numToStr<unsigned int>(value, 16, w); }

inline std::string floatToDecStr(float value, int w=0)           { return numToStr<float>(value, 10, w); }

// Call numToStr using short to print char as number
inline std::string ucharToDecStr(unsigned char value, int w=0)   { return numToStr<unsigned short>(value, 10, w); }
inline std::string charToDecStr(char value, int w=0)             { return numToStr<short>(value, 10, w); }

inline std::string ucharToHexStr(unsigned char value, int w=0)   { return numToStr<unsigned short>(value, 16, w); }
inline std::string charToHexStr(char value, int w=0)             { return numToStr<short>(value, 16, w); }

///////////////////////////////////////////////////////////////////////

template<typename NumType>
int getNumbers(const std::vector<std::string>& valueStrs, std::vector<NumType>& values)
{
    for (std::vector<std::string>::const_iterator l_itr = valueStrs.begin();
            l_itr != valueStrs.end();
            ++l_itr)
    {
        values.push_back(Util::strToNum<NumType>(*l_itr));
    }
    return values.size();
}

template<typename NumType>
int getNumbers(const std::string& rString,
                const std::string& rDelim,
                std::vector<NumType>& rValues)
{
    std::vector<std::string> l_valueStrs;
    int l_numValues = Util::splitString(rString, rDelim, l_valueStrs,false,false);
    Util::getNumbers(l_valueStrs, rValues);
    return l_numValues;
}

} // namespace

#endif
