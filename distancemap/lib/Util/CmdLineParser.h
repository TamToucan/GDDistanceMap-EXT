#ifndef UTIL_CMD_LINE_PARSER_H
#define UTIL_CMD_LINE_PARSER_H

#include <list>
#include <vector>
#include <sstream>

namespace Util {

template<class MainClass>
class CmdLineParser
{
public:
    CmdLineParser() { }
    virtual ~CmdLineParser() { }

    struct CmdOpt {
        const char* pOptStr;
        int (MainClass::*pParserFn)(CmdLineParser<MainClass>* pParser);
    };

    // Parse and leave unparsed opts on argc, argv
    int parse(MainClass* pTheObj, int* pArgc, char** argv,
              CmdOpt* pOpts)
    {
        std::list<char*> l_remainingOpts;
        int l_retVal =  parse(pTheObj, *pArgc, argv, pOpts, &l_remainingOpts);
        if (0 == l_retVal)
        {
            // Copy the remaingingOpts back to argc, argv
            *pArgc = 0;
            std::list<char*>::iterator l_itr = l_remainingOpts.begin();
            while (l_itr != l_remainingOpts.end())
            {
                argv[*pArgc] = *l_itr;
                ++(*pArgc);
                ++l_itr;
            }
            argv[*pArgc] = 0;
        }
        return l_retVal;
    }

    int parse(MainClass* pTheObj, int argc, char** argv,
              CmdOpt* pOpts, std::list<char*>* pRemainingOpts)
    {
        m_pRemainingOpts = pRemainingOpts;
        for (int i=0; i < argc; ++i)
        {
            m_pRemainingOpts->push_back(argv[i]);
        }
        int l_retval = 0;
        while ((pOpts->pOptStr) and (0 == l_retval))
        {
            m_itr = m_pRemainingOpts->begin();
            while (m_itr != m_pRemainingOpts->end())
            {
                if (0 == strcmp(pOpts->pOptStr, *m_itr))
                {
                    std::list<char*>::iterator l_erasePos = m_itr;
                    ++m_itr;
                    m_pRemainingOpts->erase(l_erasePos);
                    l_retval = ((*pTheObj).*(pOpts->pParserFn))(this);
                    break;
                }
                else
                {
                    ++m_itr;
                }
            }
            ++pOpts;
        }
        return l_retval;
    }

    template<typename ParamType>
    bool getNext(std::vector<ParamType>& rValues, int numParams)
    {
        while ((0 != numParams) and (m_itr != m_pRemainingOpts->end()))
        {
            ParamType l_val;
            std::stringstream l_ss(std::stringstream::in | std::stringstream::out);
            l_ss << *m_itr;
            l_ss >> l_val;
            rValues.push_back(l_val);

            std::list<char*>::iterator l_erasePos = m_itr;
            ++m_itr;
            m_pRemainingOpts->erase(l_erasePos);
            --numParams;
        }
        return (0 == numParams);
    }
    bool getBool(bool* o_pFlag)
    {
        std::vector<std::string> l_params;
        if (getNext(l_params, 1))
        {
            if (l_params[0] == "on")
            {
                *o_pFlag = true;
            }
            else
            if (l_params[0] == "off")
            {
                *o_pFlag = false;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
        return true;
    }

private:
    std::list<char*>* m_pRemainingOpts;
    std::list<char*>::iterator m_itr;

};

} // namespace

#endif

