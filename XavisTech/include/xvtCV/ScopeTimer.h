#pragma once
#include "xvtCV/xvtDefine.h"
#include <chrono>
#include <string>
#include <iostream>
#include <iomanip>

namespace xvt {
class ScopeTimer
{
public:
    using Miliseconds = std::chrono::duration<double, std::milli>;
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    ScopeTimer(std::string const& str)
    {
        mName = std::wstring(str.begin(), str.end());
        Start();
    }

    ScopeTimer(std::wstring const& str)
    {
        mName = str;
        Start();
    }

    ~ScopeTimer()
    {
        if (!mIsStopped) Stop();
    }

    inline TimePoint Start()
    {
        mStart = Clock::now();
        mStopPoint = mStart;
        mIsStopped = false;
        return mStart;
    }

    inline Miliseconds GetElapsedTime(std::wstring subName = L"", bool isDisplay=true)
    {
        if (!mIsStopped)
        {
            mEnd = Clock::now();
            mStopCount++;
        }
        auto res = std::chrono::duration_cast<Miliseconds>(mEnd - mStopPoint);
        mStopPoint = mEnd;

        PrintTime(res, subName, isDisplay);

        return res;
    }

    inline Miliseconds GetTotalElapsedTime(std::wstring subName = L"", bool isDisplay=true)
    {
        if (!mIsStopped)
        {
            mEnd = Clock::now();
        }

        auto res = std::chrono::duration_cast<Miliseconds>(mEnd - mStart);
        PrintTime(res, subName, isDisplay);
        return res;
    }

    inline Miliseconds Stop(bool isDisplay = true)
    {
        auto res = GetTotalElapsedTime(L"", isDisplay);
        mIsStopped = true;
        return   res;
    }

    inline void PrintTime(Miliseconds res, std::wstring subName = L"", bool isDisplay=true)
    {
        if(isDisplay)
        {
            subName = subName.empty() ? L":" : L"-" + subName + L":";
            std::wstring str = mStopCount > 0 ? mName + std::to_wstring(mStopCount) + subName : mName + subName;
            std::wstring str2 = (str + std::to_wstring(res.count()) + L"ms\n");

        #ifndef _AFX
            //Don't deletet those comment line.
            std::wcout << str2;
        #else
            //TRACE(str2.c_str());
            ::OutputDebugString(CString(str2.c_str()));
        #endif // !MFC
        }
    }

    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer(ScopeTimer&&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;
    ScopeTimer& operator=(ScopeTimer&&) = delete;

private:
    bool mIsStopped = true;
    int  mStopCount = 0;
    TimePoint mStart;
    TimePoint mEnd;
    TimePoint mStopPoint;
    std::wstring mName;
};

XVT_EXPORTS
//%F -> 2000-11-1 (%y%m%d)
//%T -> 14:10:43  (%H:%M:%S)
auto GetCurrentTime(std::string format="%F %T")->std::string;

void XVT_EXPORTS WritePerformance(std::wstring const& srcFileName, int line, std::wstring const& funcName, float time);

template<typename T, typename... Types>
void TestPerformance(std::wstring srcFileName, int line, std::wstring funcName, T funcT, Types... args)
{

    xvt::ScopeTimer timer(funcName);
    timer.Start();
    funcT(args...);
    auto t = timer.Stop();
    WritePerformance(srcFileName, line, funcName, t.count());
    return;
}

class XVT_EXPORTS PerfomanceTest: ScopeTimer
{
public:
    PerfomanceTest(std::wstring srcFileName, int line, std::wstring funcName) : ScopeTimer(funcName)
        , mSrcFileName{ srcFileName }
        , mLine{ line }
        , mFuncName{ funcName }
    {
        Start();
    }

    ~PerfomanceTest()
    {
        auto t = Stop();
        WritePerformance(mSrcFileName, mLine, mFuncName, t.count());
    }

private:
    std::wstring mSrcFileName;
    int mLine;
    std::wstring mFuncName;
};

#define XVT_PERF_FUNCNAME_FUNC( name ) L#name, name
#define XVT_PERF_TEMPNAME1(x) __xvt_Perf_temp##x
#define XVT_PERF_TEMPNAME(x) XVT_PERF_TEMPNAME1(x)
#ifdef __PERFORMANCE_ANALYSIS__
#define XVT_MESUARE_PERFORMANCE_F(func, ...) xvt::TestPerformance(std::wstring(__FILEW__), __LINE__, XVT_PERF_FUNCNAME_FUNC(func), __VA_ARGS__)
#define XVT_MESUARE_PERFORMANCE() xvt::PerfomanceTest XVT_PERF_TEMPNAME(__LINE__)(std::wstring(__FILEW__), __LINE__, __FUNCTIONW__)
#else
#define XVT_MESUARE_PERFORMANCE_F(func, ...) 
#define XVT_MESUARE_PERFORMANCE()
#endif // __PERFORMANCE_ANALYSIS__

}
