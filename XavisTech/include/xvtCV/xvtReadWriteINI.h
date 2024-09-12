#pragma once
#include "xvtCV/xvtDefine.h"
#include <string>

namespace xvt
{
namespace ini
{

XVT_EXPORTS
void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey,  bool& outValue, bool defaultValue=false);

XVT_EXPORTS
void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, int& outValue, int defaultValue=0);

XVT_EXPORTS
void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, float& outValue, float defaultValue=0.0f);

XVT_EXPORTS
void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, double& outValue, double defaultValue=0.0);

XVT_EXPORTS
void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::string & outValue, std::string defaultValue = "");

XVT_EXPORTS
void Read(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::wstring & outValue, std::wstring defaultValue = L"");

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, bool value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, int value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, float value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, double value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, const char* value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::string const& value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::string && value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, const wchar_t* value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::wstring const& value);

XVT_EXPORTS
void Write(const wchar_t* lpszIniFile, const wchar_t* lpszSection, const wchar_t* lpszKey, std::wstring && value);

}
}