#ifndef UTIL_DEBUG_H
#define UTIL_DEBUG_H

#ifdef HAVE_BUILD_CONFIG_H
#include "BuildConfig.h"
#endif

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>

// Need for setDebugging via string
#include <vector>
#include <iterator>
#include "ConvString.h"
#include "Singleton.h"

namespace Util {

#ifdef RELEASE_BUILD

#   define SET_DEBUG(f)        do { } while(0)
#   define SET_DEBUGFILE(f)    do { } while(0)
#   define SET_ERRORFILE(f)    do { } while(0)

#   define LOG_DEBUG(p1)       do { } while(0)
#   define LOG_INFO(p1)        do { } while(0)
#   define LOG_WARNING(p1)     do { } while(0)
#   define LOG_ERROR(p1)       do { } while(0)
#   define LOG_ASSERT(cnd, p1) do { } while(0)

#undef USE_LOG_ENTRY

#else

#define SET_DEBUG(f) \
    Util::Debug::instance()->setDebugging(f)

#define SET_DEBUGFILE(f) \
    Util::Debug::instance()->setDebugFile(f)

#define SET_ERRORFILE(f) \
    Util::Debug::instance()->setErrorFile(f)

#define LOG_DEBUG(p1) \
   do { \
       if (Util::Debug::instance()->debugOn()) \
           Util::Debug::instance()->debugOut() << p1 << std::endl; \
   } while(0)

#define LOG_INFO(p1) \
   do { \
       if (Util::Debug::instance()->infoOn()) \
          Util::Debug::instance()->errorOut() << "INFO: " << p1 << std::endl; \
   } while(0)

#define LOG_WARNING(p1) \
   do { \
       if (Util::Debug::instance()->warningOn()) \
          Util::Debug::instance()->errorOut() << "WARNING: " << p1 << std::endl; \
   } while(0)

#define LOG_ERROR(p1) \
   do { \
       if (Util::Debug::instance()->errorOn()) \
       { \
          Util::Debug::instance()->errorOut() << "ERROR: Func:" << __PRETTY_FUNCTION__ << " File:" << __FILE__ << " Line: " << __LINE__ << std::endl; \
          Util::Debug::instance()->errorOut() << "ERROR: " << p1 << std::endl; \
       } \
   } while(0)

#define LOG_ASSERT(cnd, p1) \
   do { \
       if (! (cnd) ) \
       { \
          Util::Debug::instance()->errorOut() << "ASSERT: Func:" << __PRETTY_FUNCTION__ << " File:" << __FILE__ << " Line: " << __LINE__ << std::endl; \
          Util::Debug::instance()->errorOut() << "ASSERT: " << #cnd << std::endl; \
          Util::Debug::instance()->errorOut() << "ASSERT: " << p1 << std::endl; \
           abort(); \
       } \
   } while(0)

#endif

#define LOG_ALWAYS(p1) \
   do { \
           Util::Debug::instance()->debugOut() << "LOG: " << p1 << std::endl; \
   } while(0)

#define LOG_ABORT(p1) \
   do { \
           Util::Debug::instance()->errorOut() << "ABORT: Func:" << __PRETTY_FUNCTION__ << " File:" << __FILE__ << " Line: " << __LINE__ << std::endl; \
           Util::Debug::instance()->errorOut() << "ABORT: " << p1 << std::endl; \
           abort(); \
   } while(0)

//////////////////////////////////////////////////////

#ifdef USE_LOG_ENTRY

#define LOG_ENTRY() \
        Util::Debug::EntryLogger l_entryLogger(__FILE__, __func__); \
        if (Util::Debug::instance()->logEntryOn()) \
            Util::Debug::instance()->debugOut() << std::endl


#define LOG_ENTRY_INFO(p1) \
        Util::Debug::EntryLogger l_entryLogger(__FILE__, __func__); \
        if (Util::Debug::instance()->logEntryOn()) \
            Util::Debug::instance()->debugOut() << "> " << p1 << std::endl

#else

#define LOG_ENTRY() do { } while(0)
#define LOG_ENTRY_INFO(p1) do { } while(0)

#endif

//////////////////////////////////////////////////////

class Debug : public Util::Singleton<Debug>
{
public:
    static const int DEBUG_ON      = 1 << 0;
    static const int INFO_ON       = 1 << 1;
    static const int WARNING_ON    = 1 << 2;
    static const int ERROR_ON      = 1 << 3;
    static const int LOGENTRY_ON   = 1 << 4;
    static const int ALL_ON        = 0xffff;
    static const int WARN_ERR_ON          = WARNING_ON + ERROR_ON;
    static const int INFO_WARN_ERR_ON     = INFO_ON + WARNING_ON + ERROR_ON;
public:
    Debug() : m_debugFlags(0)
        , m_level(0)
        , m_pDebugOut(&std::cout)
        , m_pErrorOut(&std::cerr)
    {
        setDebug(false);
        setInfo(true);
        setWarning(true);
        setError(true);
        setLogEntry(false);
    }
private:
    int m_debugFlags;
    int m_level;
    std::ostream* m_pDebugOut;
    std::ostream* m_pErrorOut;
    std::ofstream m_debugFile;
    std::ofstream m_errorFile;
    void setFlag(bool on, int flag)
    {
        if (on) m_debugFlags |= flag;
        else    m_debugFlags &= ~flag;
    }
    // Close the current file and open a new open, use default if error
    void changeFile(const std::string& f, std::ofstream& fs, std::ostream*& pOut, std::ostream* pDef) {
        if (not f.empty())
        {
            if (fs.is_open())
            {
                fs.close();
            }
            fs.open(f.c_str(), std::ios::out|std::ios::trunc);
            pOut = fs.good() ? &fs : pDef;
        }
    }
public:
    bool debugOn() const    { return (m_debugFlags&DEBUG_ON); }
    bool infoOn() const     { return (m_debugFlags&INFO_ON); }
    bool warningOn() const  { return (m_debugFlags&WARNING_ON); }
    bool errorOn() const    { return (m_debugFlags&ERROR_ON); }
    bool logEntryOn() const { return (m_debugFlags&LOGENTRY_ON); }

    void setDebugging(int val)  { m_debugFlags = val; }
    void setDebug(bool val)     { setFlag(val, DEBUG_ON); }
    void setInfo(bool val)      { setFlag(val, INFO_ON); }
    void setWarning(bool val)   { setFlag(val, WARNING_ON); }
    void setError(bool val)     { setFlag(val, ERROR_ON); }
    void setLogEntry(bool val)  { setFlag(val, LOGENTRY_ON); }

    void setDebugging(const std::string& s) { m_debugFlags = debugFlagsFromString(s); }

    void setDebugFile(const std::string& f) { changeFile(f, m_debugFile, m_pDebugOut, &std::cout); }
    void setErrorFile(const std::string& f) { changeFile(f, m_errorFile, m_pErrorOut, &std::cerr); }

    std::ostream& debugOut() { return *m_pDebugOut; }
    std::ostream& errorOut() { return *m_pErrorOut; }
protected:
    // Get the debugging flags from a string like
    // WARN+ERROR+INFO
    // Probably shouldn't be here, but hey
    int debugFlagsFromString(const std::string& s)
    {
        int l_debugFlags = 0;
        std::vector<std::string> l_flagStrs;
        Util::splitString(s,"+",l_flagStrs);
        for (std::vector<std::string>::const_iterator l_itr = l_flagStrs.begin();
             l_itr != l_flagStrs.end();
             ++l_itr)
        {
            if (*l_itr == "ERROR") l_debugFlags |= Util::Debug::ERROR_ON;
            else
            if (*l_itr == "WARN") l_debugFlags |= Util::Debug::WARNING_ON;
            else
            if (*l_itr == "INFO") l_debugFlags |= Util::Debug::INFO_ON;
            else
            if (*l_itr == "DEBUG") l_debugFlags |= Util::Debug::DEBUG_ON;
            else
            if (*l_itr == "STACK") l_debugFlags |= Util::Debug::LOGENTRY_ON;
            else
            if (*l_itr == "ALL")   l_debugFlags |= Util::Debug::ALL_ON;
        }
        return l_debugFlags;
    }

    // For EntryLogger
    void incDepth() { ++m_level; }
    void decDepth() { --m_level; }
    int  getDepth() const { return m_level; }

public:
    struct EntryLogger
    {
        const char* m_pFile;
        const char* m_pFunc;
        EntryLogger(const char* pFile, const char* pFunc) :
            m_pFile(pFile), m_pFunc(pFunc)
        {
            if (Util::Debug::instance()->logEntryOn())
            {
                std::string l_indent(Debug::instance()->getDepth()*3,' ');
                std::cout << l_indent << "{-> "
                    << m_pFile << ':' << m_pFunc;
            }
            Debug::instance()->incDepth();
        }
        ~EntryLogger()
        {
            Debug::instance()->decDepth();
            if (Util::Debug::instance()->logEntryOn())
            {
                std::string l_indent(Debug::instance()->getDepth()*3,' ');
                std::cout << l_indent << "<-} "
                    << m_pFile << ':' << m_pFunc << std::endl;
            }
        }
    };
};


} // namespace
#endif

