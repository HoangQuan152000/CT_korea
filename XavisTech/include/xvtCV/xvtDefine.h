#pragma once

#if (defined _WINDLL)

#ifndef XVT_EXPORTS
#define XVT_EXPORTS             __declspec(dllexport)
#endif // !XVT_EXPORTS

#elif (defined _XVTCV_USE_SRC_)

#ifndef XVT_EXPORTS
#define XVT_EXPORTS
#endif // !XVT_EXPORTS

#else 

#ifndef XVT_EXPORTS
#define XVT_EXPORTS             __declspec(dllimport)
#endif // !XVT_EXPORTS

#endif

#if (defined _WINDLL) && (defined __XVTCV)

#ifndef XVTCV_EXPORTS
#define XVTCV_EXPORTS              __declspec(dllexport)
#endif

#else 

#ifndef XVTCV_EXPORTS
#define XVTCV_EXPORTS             __declspec(dllimport)
#endif // !XVT_EXPORTS

#endif

#define XVT_WSTR_HELPER(__A) L ## __A
//Convert from string to wstring
#define XVT_WSTR(__A)        XVT_WSTR_HELPER(__A)
//Get the string expression of macro
#define XVT_STR_EXP(__A)    #__A
//Get the wstring expression of macro
#define XVT_WSTR_EXP(__A)   L#__A
//Convert macro value to string
#define XVT_CVT_STR(__A)    XVT_STR_EXP(__A)
#define XVT_CVT_WSTR(__A)   XVT_WSTR_EXP(__A)

//Get the Function name, line and info
#define XVT_FUNC_DETAIL(info) __FUNCTION__ ", " XVT_CVT_STR(__LINE__) ": " XVT_STR_EXP(info)
//if condition is failed then throw invalid_argument.
#define CHECK_INPUT(condition) {if (!(condition)) throw std::invalid_argument(XVT_FUNC_DETAIL(condition));}

#ifdef _DEBUG
#define XVT_BUILD_CONFIGURATION_STR   "Debug"
#else
#ifdef  __PERFORMANCE_ANALYSIS__
#define XVT_BUILD_CONFIGURATION_STR   "ReleasePerf"
#else
#define XVT_BUILD_CONFIGURATION_STR   "Release"
#endif
#endif // _DEBUG


#ifdef _WIN64
#define XVT_BUILD_PLATFORM_STR   "x64"
#else
#define XVT_BUILD_PLATFORM_STR   "win32"
#endif

#define XVT_BUILD_PLATFORM_WSTR         XVT_WSTR(XVT_BUILD_PLATFORM_STR)
#define XVT_BUILD_CONFIGURATION_WSTR    XVT_WSTR(XVT_BUILD_CONFIGURATION_STR)

#define XVT_PLATFORM_CONFIG_PATH  XVT_BUILD_PLATFORM_STR "/" XVT_BUILD_CONFIGURATION_STR "/"
#define XVT_PLATFORM_CONFIG_WPATH XVT_WSTR(XVT_PLATFORM_CONFIG_PATH)