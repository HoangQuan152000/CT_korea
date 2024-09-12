#pragma once
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtConvert.h"
#include <fstream>
#include <string>
#include <vector>
namespace xvt {
using CSVOutput = VecKeyValueStr;

//Interface support save data to .csv file.
class XVT_EXPORTS CSV
{
public:
    virtual auto GetTitleStr()const->std::string;

    //Get the header-data csv string format
    virtual auto GetCSVData(std::string prefix = "", bool isRecursive = true)const->CSVOutput;

    //Get the header-data csv string format, isRecursive=true -> get the children data
    virtual auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void;

    //Save data to csv file
    virtual bool Save(std::wstring const& path, std::wstring const& imgName = L"", bool isNewFile = true)const;

    //Add the header-data csv string
    static
    auto AddData(std::pair<std::string, std::string> const& p1, std::pair<std::string, std::string> const& p2)->std::pair<std::string, std::string>;

    static
    auto AddData(CSVOutput& p1, CSVOutput const& p2)->void;

    //Get the CSV string format
    template<typename T>
    static
    auto GetStr(std::vector<T>const& data, bool isAddEndline=true)->std::string;

    //Get the header-data csv string format
    template<typename T>
    static
    auto GetData(std::vector<std::pair<std::string, T>>const& data, std::string prefix = "")->CSVOutput;

    //Get the header-data csv string format
    template<typename T>
    static
    auto GetData(std::vector<std::pair<std::string, T>>const& data, CSVOutput& out, std::string prefix = "")->void;

    //Get the header csv string format
    template<typename T>
    static
    auto GetHeaderStr(std::vector<std::pair<std::string, T>>const& data, bool isAddEndline = true, std::string prefix = "")->std::string;

    //Get the data csv string format
    template<typename T>
    static
    auto GetDataStr(std::vector<std::pair<std::string, T>>const& data, bool isAddEndline = true)->std::string;

    //Get the key-value string format
    template<typename T>
    static
    auto GetKeyValueStr(std::vector<std::pair<std::string, T>>const& data)->std::string;

    //Save data to csv file
    template<typename T>
    static
    bool Save(std::wstring const& path, std::vector<std::string> const& header, std::vector<T> const& data, bool isNewFile = true);

    //Save data to csv file
    template<typename T>
    static
    bool Save(std::wstring const& path, std::vector<std::pair<std::string, T>> const& data, bool isNewFile = true);

};

inline
auto CSV::GetTitleStr() const -> std::string
{
    return std::string();
}

inline
auto CSV::GetCSVData(std::string prefix, bool isRecursive)const ->CSVOutput
{
    CSVOutput out;
    GetCSVData(out, prefix);
    return out;
}

inline auto CSV::GetCSVData(CSVOutput& out, std::string prefix, bool isRecursive) const -> void
{
}

template<typename T>
auto CSV::GetStr(std::vector<T>const& data, bool isAddEndline) -> std::string
{
    using namespace xvt;
    std::string str;
    if (!data.empty())
    {
        str = ToString(data[0]);
        for (std::size_t i = 1, size = data.size(); i < size; i++)
        {
            str += "," + ToString(data[i]);
        }
        if (isAddEndline) str += "\n";
    }
    return str;
}

template<typename T>
inline
auto CSV::GetData(std::vector<std::pair<std::string, T>> const& data, std::string prefix) -> CSVOutput
{
    CSVOutput out;
    To_(data, out, prefix);
    return out;
}

template<typename T>
inline auto CSV::GetData(std::vector<std::pair<std::string, T>> const& data, CSVOutput& out, std::string prefix) -> void
{
    To_(data, out, prefix);
}

inline
auto CSV::AddData(std::pair<std::string, std::string> const& p1, std::pair<std::string, std::string> const& p2) -> std::pair<std::string, std::string>
{
    return std::pair<std::string, std::string>(p1.first + "," + p2.first, p1.second + "," + p2.second);
}

inline auto CSV::AddData(CSVOutput& p1, CSVOutput const& p2) -> void
{
    p1.insert(p1.end(), p2.begin(), p2.end());
}

template<typename T>
auto CSV::GetHeaderStr(std::vector<std::pair<std::string, T>> const& data, bool isAddEndline, std::string prefix) -> std::string
{
    using namespace xvt;
    std::string str;
    if (!data.empty())
    {
        str = prefix + data[0].first;
        for (std::size_t i = 1, size = data.size(); i < size; i++)
        {
            str += "," + prefix + data[i].first;
        }
        if (isAddEndline) str += "\n";
    }
    return str;
}

template<typename T>
auto CSV::GetDataStr(std::vector<std::pair<std::string, T>> const& data, bool isAddEndline) -> std::string
{
    using namespace xvt;
    std::string str;
    if (!data.empty())
    {
        str = ToString(data[0].second);
        for (std::size_t i = 1, size = data.size(); i < size; i++)
        {
            str += "," + ToString(data[i].second);
        }
        if (isAddEndline) str += "\n";
    }
    return str;
}

template<typename T>
inline auto CSV::GetKeyValueStr(std::vector<std::pair<std::string, T>> const& data) -> std::string
{
    auto tmp = std::string();
    if (!data.empty())
    {
        tmp = data[0].first + ":" + xvt::ToString(data[0].second);
        for (size_t i = 1, size = data.size(); i < size; i++)
        {
            tmp += ", " + data[i].first + ":" + xvt::ToString(data[i].second);
        }
    }
    return tmp;
}

template<typename T>
bool CSV::Save(std::wstring const& path, std::vector<std::string> const& header, std::vector<T> const& data, bool isNewFile)
{
    if (data.empty() && header.empty()) return true;

    int fileMode = isNewFile ? std::iostream::trunc : std::iostream::app;
    std::wfstream f(path, std::iostream::out | fileMode);
    auto isOK = f.is_open();
    if (isOK)
    {
        if (isNewFile && !header.empty())
        {
            f << GetStr(header).c_str();
        }

        if (!data.empty())
        {
            f << GetStr(data).c_str();
        }
    }
    f.close();

    return isOK;
}

template<typename T>
bool CSV::Save(std::wstring const& path, std::vector<std::pair<std::string, T>> const& data, bool isNewFile)
{
    if (data.empty()) return true;

    int fileMode = isNewFile ? std::iostream::trunc : std::iostream::app;
    std::wfstream f(path, std::iostream::out | fileMode);
    auto isOK = f.is_open();
    if (isOK)
    {
        if (isNewFile)
        {
            f << GetHeaderStr(data).c_str();
        }

        f << GetDataStr(data).c_str();
    }
    f.close();

    return isOK;
}

}


