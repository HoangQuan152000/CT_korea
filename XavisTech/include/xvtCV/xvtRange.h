#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/persistence.hpp>
#include <stdexcept>

namespace xvt {

template <class T>
struct Range
{
private:
    T mLower;
    T mUpper;
public:
    bool mIsEnable;

    Range()
    {
        Set(0, 0);
        mIsEnable = true;
    }

    Range(T const& l, T const& u, bool enable = true)
    {
        Set(l, u);
        mIsEnable = enable;
    }
    // while upper < lower => mUpper = l; mLower = u;
    
    void Set(T const& l, T const& u)
    {
        if (u < l)
        {
            //throw std::invalid_argument("Invalid Range: mUpper < mLower");

            mUpper = l;
            mLower = u;
        }
        else
        {
            mUpper = u;
            mLower = l;
        }
    }

    T GetUpper() const
    {
        return mUpper;
    }

    T GetLower() const
    {
        return mLower;
    }

    bool operator==(const Range& other) const
    {
        return (mUpper == other.mUpper) && (mLower == other.mLower) && (mIsEnable == other.mIsEnable);
    }

    template<class T2>
    bool operator()(T2 const& value) const
    {
        return mIsEnable ? (value >= mLower && value <= mUpper) : true;
    }

    template<class T2>
    inline bool IsInRange(T2 const& value) const
    {
        return  operator()(value);
    }

    inline void Write(cv::FileStorage& rFs) const&
    {
        if (rFs.isOpened())
        {
            rFs << "mLower" << mLower << "mUpper" << mUpper << "mIsEnable" << (int)mIsEnable;
        }
        else
        {
            //Do nothing
        }
    }

    inline void Read(cv::FileNode const& node)&
    {
        mLower = (T)node["mLower"];
        mUpper = (T)node["mUpper"];
        mIsEnable = (int)node["mIsEnable"];
    }

    inline
    std::string ToString()
    {
        return  std::to_string(mIsEnable) + "[" + std::to_string(mLower) + " " + std::to_string(mUpper) + "]";
    }
};

struct XVT_EXPORTS RotatedAngleRange
{
private:
    // mRefAngle: Reference Angle. Here we calculate how far from rotated angle to reference angle by using IsRotatedAngleInRange
    double mRefAngle;
    // mStdAngle: variable to determine shape of minAreaRect of an object (45 if square, 90 if not square).
    double mStdAngle;

    double mUpperOffsetAngle;
    double mLowerOffsetAngle;
public:
    bool mIsEnable;
    RotatedAngleRange()
    {
        Set(0, 0, 0, 90);
        this->mIsEnable = true;
    }

    RotatedAngleRange(double const& l, double const& u, double const& refAngle = 0, double const& stdAngle = 90, bool enable = true)
    {
        Set(l, u, refAngle, stdAngle);
        this->mIsEnable = enable;
    }

    void Set(double const& l, double const& u, double const& refAngle, double const& stdAngle)
    {
        if (refAngle < -stdAngle || refAngle > stdAngle || u < 0 || l < 0 ||
            u > stdAngle || l > stdAngle)
        {
            throw std::invalid_argument("Invalid Parameter");
        }
        else
        {
            this->mRefAngle = refAngle;
            this->mStdAngle = stdAngle;
            this->mUpperOffsetAngle = u;
            this->mLowerOffsetAngle = l;
        }
    }

    double GetRefAngle()
    {
        return this->mRefAngle;
    }

    double GetStdAngle()
    {
        return this->mStdAngle;
    }

    double GetUpperOffsetAngle()
    {
        return this->mUpperOffsetAngle;
    }

    double GetLowerOffsetAngle()
    {
        return this->mLowerOffsetAngle;
    }
    // Calculate angle distance between rotated angle and reference angle. 
    // Returns true if distance is within upper and lower threshold. Otherwise returns false.
    inline bool IsRotatedAngleInRange(double const& rotatedAngle) const
    {
        bool retVal = true;

        if (this->mIsEnable)
        {
            Range<double> angleRange = Range<double>(this->mRefAngle - this->mLowerOffsetAngle, this->mRefAngle + this->mUpperOffsetAngle, true);
            retVal = angleRange.IsInRange(rotatedAngle) ||
                ((rotatedAngle < 0) ? angleRange.IsInRange(rotatedAngle + mStdAngle * 2) : angleRange.IsInRange(rotatedAngle - mStdAngle * 2));
        }
        else
        {
            //Do nothing
        }

        return retVal;
    }

    inline void Write(cv::FileStorage& rFs) const&
    {
        if (rFs.isOpened())
        {
            rFs << "mLowerOffsetAngle" << mLowerOffsetAngle << "mUpperOffsetAngle" << mUpperOffsetAngle << "mIsEnable" << (int)mIsEnable;
        }
        else
        {
            //Do nothing
        }
    }

    inline void Read(cv::FileNode const& node)&
    {
        mLowerOffsetAngle = (double)node["mLowerOffsetAngle"];
        mUpperOffsetAngle = (double)node["mUpperOffsetAngle"];
        mIsEnable = (int)node["mIsEnable"];
    }
};

using Rangei = Range<int>;
using Rangef = Range<float>;
using Ranged = Range<double>;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// for convenience
#ifndef XPU_RANGE_INT
#define XPU_RANGE_INT       ::xvt::Range<int>
#endif // !XPU_RANGE_INT

#ifndef XPU_RANGE_FLOAT
#define XPU_RANGE_FLOAT     ::xvt::Range<float>
#endif // !XPU_RANGE_FLOAT

#ifndef XPU_RANGE_DOUBLE
#define XPU_RANGE_DOUBLE    ::xvt::Range<double>
#endif // !XPU_RANGE_DOUBLE


//class XVT_EXPORTS Ranged : public Range<double>
//{
//
//};

}//xvt