#pragma once
#include "xvtCV/xvtConvert.h"
#include <map>
#include <string>

namespace xvt {

class XVT_EXPORTS ParseArgs
{
public:
    ParseArgs() = default;

    ParseArgs(int argc, wchar_t* argv[]);

    template<typename T>
    auto GetArg(std::wstring const& arg, T defaultValue)->T;

    static
    auto Parse(int argc, wchar_t* argv[])->std::map<std::wstring, std::wstring>;

private:
    std::map<std::wstring, std::wstring> mArgList;

};

inline
xvt::ParseArgs::ParseArgs(int argc, wchar_t* argv[])
{
    mArgList = xvt::ParseArgs::Parse(argc, argv);
}

template<>
inline
auto ParseArgs::GetArg<std::wstring>(std::wstring const& arg, std::wstring defaultValue)->std::wstring
{
    return mArgList[arg];
}

template<typename T>
inline
auto ParseArgs::GetArg(std::wstring const& arg, T defaultValue) -> T
{
    try {
        defaultValue = xvt::To_<T>(mArgList[arg]);
    }
    catch (std::exception const& ex) {

    }

    return defaultValue;
}

}

