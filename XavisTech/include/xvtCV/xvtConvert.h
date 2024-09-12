#pragma once
#include "xvtCV/xvtDefine.h"
#include <string>
#include <vector>
#include <sstream>

namespace xvt
{

XVT_EXPORTS
auto ToWString(std::string value)->std::wstring;

XVT_EXPORTS
auto ToString(std::wstring value)->std::string;

inline
auto ToString(int a)->std::string
{
    return std::to_string(a);
}

inline
auto ToString(float value)->std::string
{
    return std::to_string(value);
}

inline
auto ToString(double value)->std::string
{
    return std::to_string(value);
}

inline
auto ToString(std::string const& value)->std::string const&
{
    return value;
}

inline
auto ToString(std::string& value)->std::string&
{
    return value;
}

inline
auto ToString(std::string&& value)->std::string
{
    return value;
}

inline
std::string ToString(const double a, int precision)
{
    std::ostringstream oss;
    if (precision <= 0)
    {
        oss << (int)std::round(a);
    }
    else
    {
        if (precision > 6) precision = 6;
        int power = std::pow(10, precision); // Calculate the power of 10
        double value = std::round(a * power) / power;
        oss.precision(precision);
        oss << std::fixed << value;
    }
    return std::move(oss).str();
}

XVT_EXPORTS
auto ToLowerCase(std::string s)->std::string;

XVT_EXPORTS
auto ToLowerCase(std::wstring s)->std::wstring;

XVT_EXPORTS
auto TrimSpace (const std::string& s)->std::string;

XVT_EXPORTS
auto TrimSpace(const std::wstring& s)->std::wstring;

XVT_EXPORTS
auto TrimLeadingSpace(std::string const& str)->std::string;

XVT_EXPORTS
auto TrimLeadingSpace(std::wstring const& str)->std::wstring;

XVT_EXPORTS
auto TrimTrailingSpace(std::string const& str)->std::string;

XVT_EXPORTS
auto TrimTrailingSpace(std::wstring const& str)->std::wstring;

XVT_EXPORTS
auto Split(std::string const& str, char separator = ',')->std::vector<std::string>;

XVT_EXPORTS
auto Split(std::wstring const& str, char separator = ',')->std::vector<std::wstring>;

template<typename T>
inline
auto To_(const std::wstring& str) -> T = delete;

template<>
inline
auto To_<std::wstring>(const std::wstring& str)->std::wstring
{
    return str;
}

template<>
inline
auto To_<const wchar_t*>(const std::wstring& str)->const wchar_t*
{
    return str.c_str();
}

template<>
inline
auto To_<std::string>(const std::wstring& str)->std::string
{
    return xvt::ToString(str);
}

template<>
inline
auto To_<int>(const std::wstring& str)->int
{
    return std::stoi(str);
}

template<>
inline
auto To_<long>(const std::wstring& str)->long
{
    return std::stol(str);
}

template<>
inline
auto To_<float>(const std::wstring& str)->float
{
    return std::stof(str);
}

template<>
inline
auto To_<double>(const std::wstring& str)->double
{
    return std::stod(str);
}

template<>
inline
auto To_<long double>(const std::wstring& str)->long double
{
    return std::stold(str);
}

template<>
inline
auto To_<bool>(const std::wstring& str)->bool
{
    return (bool)std::stoi(str);
}

template<typename T>
auto To_(std::vector<T> const& data)->std::vector<std::string>
{
    using namespace xvt;
    std::vector<std::string> vec;
    for (auto&& d : data)
        vec.emplace_back(ToString(d));
    return vec;
}

template<typename T>
auto To_(std::vector<std::pair<std::string, T>> const& data, std::vector<std::pair<std::string, std::string>>& out, std::string prefix="")->void
{
    using namespace xvt;
    size_t targetCapacity = out.size() + data.size() * 2;
    if (out.capacity() < targetCapacity)
        out.reserve(targetCapacity);

    if(prefix.empty())
    {
        for (auto&& d : data)
            out.emplace_back(d.first, ToString(d.second));
    }
    else
    {
        for (auto&& d : data)
            out.emplace_back(prefix + d.first, ToString(d.second));
    }
}

}