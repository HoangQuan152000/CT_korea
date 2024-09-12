#pragma once
#include "xvtCV/xvtDefine.h"
#include <map>
#include <string>

namespace xvt {

template<class T>
std::string ToString(T const& t, std::map<T, std::string> const& mapData)
{
    std::string tmp;
    for (auto& grid : mapData)
    {
        if (grid.first == t)
        {
            tmp = grid.second;
            break;
        }
    }

    if(tmp.empty()) tmp = "Enum converted string was not defined in enum map data";
    return tmp;
}

template<class T>
T ToEnum(std::string const& str, T const& defaultValue, std::map<T, std::string> const& mapData)
{
    for (auto& grid : mapData)
    {
        if (grid.second == str)
        {
            return grid.first;
        }
    }

    return defaultValue;
}

template<class T>
bool ParseEnum(std::string const& str, T& rOutEnum, std::map<T, std::string> const& mapData)
{
    bool rvl = false;
    for (auto& grid : mapData)
    {
        if (grid.second == str)
        {
            rOutEnum = grid.first;
            rvl = true;
            break;
        }
    }
    return rvl;
}
}

#ifndef ENUM_MAP_DATA_NAME
#define ENUM_MAP_DATA_NAME(EnumName) MapData##EnumName
#endif // !ENUM_MAP_DATA_NAME(EnumName)

#ifndef DEFINE_ENUM_MAP_DATA
#define DEFINE_ENUM_MAP_DATA(EnumName) std::map<EnumName, std::string> ENUM_MAP_DATA_NAME(EnumName)
#endif // DEFINE_ENUM_MAP_DATA

#ifndef TEMP_DEFINE_TOSTRING_FUNC
/*Try to parse a string to Enum type. \
* if can not parse return the defualtValue*/
#define TEMP_DEFINE_TOSTRING_FUNC(EnumName) XVT_EXPORTS std::string ToString(EnumName const& str)
#endif // !TEMP_TOSTRING_FUNC

#ifndef TEMP_DECLARE_TOSTRING_FUNC
#define TEMP_DECLARE_TOSTRING_FUNC(EnumName) TEMP_DEFINE_TOSTRING_FUNC(EnumName) \
    { \
        return xvt::ToString(str, ENUM_MAP_DATA_NAME(EnumName)); \
    }
#endif // !TEMP_TOSTRING_DECLARE_FUNC

#ifndef TEMP_DEFINE_TOENUM_FUNC
/*Try to parse a string to Enum type. \
* if can not parse return the defualtValue*/
#define TEMP_DEFINE_TOENUM_FUNC(EnumName) XVT_EXPORTS EnumName ToEnum(std::string const& str, EnumName const& defaultValue)
#endif // !TEMP_TOENUM_DEFINE_FUNC

#ifndef TEMP_DECLARE_TOENUM_FUNC
#define TEMP_DECLARE_TOENUM_FUNC(EnumName) TEMP_DEFINE_TOENUM_FUNC(EnumName) \
    { \
        return xvt::ToEnum(str, defaultValue, ENUM_MAP_DATA_NAME(EnumName)); \
    }
#endif // !TEMP_TOENUM_DECLARE_FUNC

#ifndef TEMP_DEFINE_PARSEENUM_FUNC
/*Try to parse a string to Enum type. \
* Return false if can not parse, true if parses successfully */
#define TEMP_DEFINE_PARSEENUM_FUNC(EnumName) XVT_EXPORTS bool ParseEnum(std::string const& str, EnumName & rOutEnum)
#endif // !TEMP_PARSEENUM_DEFINE_FUNC

#ifndef TEMP_DECLARE_PARSEENUM_FUNC
#define TEMP_DECLARE_PARSEENUM_FUNC(EnumName) TEMP_DEFINE_PARSEENUM_FUNC(EnumName) \
    { \
        return xvt::ParseEnum(str, rOutEnum, ENUM_MAP_DATA_NAME(EnumName)); \
    }
#endif // !TEMP_PARSEENUM_DEFINE_FUNC

#ifndef DEFINE_CONVERT_ENUM_FUNCS
//When call this function make sure that you create the ENUM MAP DATA by call the DECLARE_ENUM_MAP_DATA(EnumName)
//and define the enum key-value for that data map in the .cpp file
#define DEFINE_CONVERT_ENUM_FUNCS(EnumName) \
    /*Convert Enum to string*/ \
    TEMP_DEFINE_TOSTRING_FUNC(EnumName);\
    \
    /*Try to parse a string to Enum type. \
    * if can not parse return the defualtValue*/ \
    TEMP_DEFINE_TOENUM_FUNC(EnumName);\
    \
    /*Try to parse a string to Enum type. \
    * Return false if can not parse, true if parses successfully */ \
    TEMP_DEFINE_PARSEENUM_FUNC(EnumName);\
    \
    // /*Write xml*/ \
    // inline void Write(cv::FileStorage& fs, std::string const& name, EnumName const& value) \
    // { \
    //     cv::write(fs, name, ToString(value)); \
    // } \
    // \
    // /*Read xml*/ \
    // inline bool Read(cv::FileNode const& node, EnumName& rOutEnum, EnumName defaultValue) \
    // {\
    //     bool rtnVl = false; \
    //     if (!node.empty()) \
    //     { \
    //         std::string tmp; \
    //         cv::read(node, tmp, ""); \
    //         rtnVl = ParseEnum(tmp, defaultValue); \
    //     } \
    //     else\
    //     { \
    //         rtnVl = false; \
    //     } \
    //     rOutEnum = std::move(defaultValue); \
    //     return rtnVl; \
    // } \

#endif // !DEFINE_CONVERT_ENUM_FUNCS(EnumName)

#ifndef DECLARE_ENUM_MAP_DATA
#define DECLARE_ENUM_MAP_DATA(EnumName) XVT_EXPORTS DEFINE_ENUM_MAP_DATA(EnumName)
#endif // !DECLARE_ENUM_DATA(EnumName)

#ifndef DECLARE_CONVERT_ENUM_FUNCS
#define DECLARE_CONVERT_ENUM_FUNCS(EnumName) \
    TEMP_DECLARE_TOSTRING_FUNC(EnumName)\
    TEMP_DECLARE_TOENUM_FUNC(EnumName)\
    TEMP_DECLARE_PARSEENUM_FUNC(EnumName)
#endif // DECLARE_CONVERT_ENUM_FUNCS