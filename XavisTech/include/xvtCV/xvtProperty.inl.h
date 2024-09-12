#pragma once
#include "xvtProperty.h"
#include "xvtReadWriteINI.h"

namespace xvt {

template< typename T > inline
bool TypedProperty<T>::SaveINI(const std::wstring& iniFile, const std::wstring& section) const
{
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mName.c_str(), mData.get());
    return true;
}

template< typename T > inline
bool TypedProperty<T>::LoadINI(const std::wstring& iniFile, const std::wstring& section)
{
    auto& tmpValue = mData.get();
    ::xvt::ini::Read(iniFile.c_str(), section.c_str(), mName.c_str(), tmpValue, mDefaultValue);
    tmpValue = ::xvt::SaturateCast(tmpValue, mMinValue, mMaxValue);
    return true;
}

template< typename T > inline
bool TypedProperty<std::basic_string<T, std::char_traits<T>, std::allocator<T>>>::SaveINI(const std::wstring& iniFile, const std::wstring& section) const
{
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mName.c_str(), mData.get());
    return true;
}

template< typename T > inline
bool TypedProperty<std::basic_string<T, std::char_traits<T>, std::allocator<T>>>::LoadINI(const std::wstring& iniFile, const std::wstring& section)
{
    auto& tmpValue = mData.get();
    ::xvt::ini::Read(iniFile.c_str(), section.c_str(), mName.c_str(), tmpValue, mDefaultValue);
    return true;
}

#pragma region cv::Point_ property
template< typename T >
TypedProperty<cv::Point_<T>>::TypedProperty(std::wstring const& name
                                            , cv::Point_<T>& data
                                            , cv::Point_<T> defaultValue
                                            , cv::Point_<T> minValue
                                            , cv::Point_<T> maxValue
) : mName{ name }, mData{ std::ref(data) }, mDefaultValue{ defaultValue }, mMinValue{ minValue }, mMaxValue{ maxValue }
{
    mNameX = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_X);
    mNameY = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_Y);
};

template< typename T >
bool TypedProperty<cv::Point_<T>>::SaveINI(const std::wstring& iniFile, const std::wstring& section) const
{
    auto& p = mData.get();
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameX.c_str(), p.x);
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameY.c_str(), p.y);
    return true;
}

template< typename T >
bool TypedProperty<cv::Point_<T>>::LoadINI(const std::wstring& iniFile, const std::wstring& section)
{
    auto& p = mData.get();
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameX.c_str(), p.x, mDefaultValue.x);
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameY.c_str(), p.y, mDefaultValue.y);
    //p = ::xvt::SaturateCast(p, mMinValue, mMaxValue);
    return true;
}

#pragma endregion

#pragma region cv::Rect_ property
template< typename T >
TypedProperty<cv::Rect_<T>>::TypedProperty(std::wstring const& name
                                           , cv::Rect_<T>& data
                                           , cv::Rect_<T> defaultValue
) : mName{ name }, mData{ std::ref(data) }, mDefaultValue{ defaultValue }
{
    mNameX = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_X);
    mNameY = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_Y);
    mNameW = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_W);
    mNameH = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_H);
};

template< typename T >
bool TypedProperty<cv::Rect_<T>>::SaveINI(const std::wstring& iniFile, const std::wstring& section) const
{
    auto& p = mData.get();
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameX.c_str(), p.x);
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameY.c_str(), p.y);
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameW.c_str(), p.width);
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameH.c_str(), p.height);
    return true;
}

template< typename T >
bool TypedProperty<cv::Rect_<T>>::LoadINI(const std::wstring& iniFile, const std::wstring& section)
{
    auto& p = mData.get();
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameX.c_str(), p.x, mDefaultValue.x);
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameY.c_str(), p.y, mDefaultValue.y);
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameW.c_str(), p.width, mDefaultValue.width);
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameH.c_str(), p.height, mDefaultValue.height);
    //p = ::xvt::SaturateCast(p, mMinValue, mMaxValue);
    return true;
}
#pragma endregion

#pragma region cv::Size_ property
template< typename T >
TypedProperty<cv::Size_<T>>::TypedProperty(std::wstring const& name
                                           , cv::Size_<T>& data
                                           , cv::Size_<T> defaultValue
) : mName{ name }, mData{ std::ref(data) }, mDefaultValue{ defaultValue }
{
    mNameW = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_W);
    mNameH = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_H);
};

template< typename T >
bool TypedProperty<cv::Size_<T>>::SaveINI(const std::wstring& iniFile, const std::wstring& section) const
{
    auto& p = mData.get();
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameW.c_str(), p.width);
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameH.c_str(), p.height);
    return true;
}

template< typename T >
bool TypedProperty<cv::Size_<T>>::LoadINI(const std::wstring& iniFile, const std::wstring& section)
{
    auto& p = mData.get();
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameW.c_str(), p.width, mDefaultValue.width);
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameH.c_str(), p.height, mDefaultValue.height);
    //p = ::xvt::SaturateCast(p, mMinValue, mMaxValue);
    return true;
}
#pragma endregion

#pragma region xvt::Range property
template< typename T >
TypedProperty<xvt::Range<T>>::TypedProperty(std::wstring const& name
                                            , xvt::Range<T>& data
                                            , xvt::Range<T> defaultValue
) : mName{ name }, mData{ std::ref(data) }, mDefaultValue{ defaultValue }
{
    mNameEnable = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_ENABLE);
    mNameFrom = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_FROM);
    mNameTo = _XVT_WSTR_INI_JOIN(mName, _XVT_WSTR_TO);
};

template< typename T >
bool TypedProperty<xvt::Range<T>>::SaveINI(const std::wstring& iniFile, const std::wstring& section) const
{
    auto& p = mData.get();
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameEnable.c_str(), p.mIsEnable);
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameFrom.c_str(), p.GetLower());
    xvt::ini::Write(iniFile.c_str(), section.c_str(), mNameTo.c_str(), p.GetUpper());
    return true;
}

template< typename T >
bool TypedProperty<xvt::Range<T>>::LoadINI(const std::wstring& iniFile, const std::wstring& section)
{
    auto& p = mData.get();
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameEnable.c_str(), p.mIsEnable, mDefaultValue.mIsEnable);
    T l = 0.0;
    T u = 0.0;
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameFrom.c_str(), l, mDefaultValue.GetLower());
    xvt::ini::Read(iniFile.c_str(), section.c_str(), mNameTo.c_str(), u, mDefaultValue.GetUpper());
    p.Set(l, u);
    //p = ::xvt::SaturateCast(p, mMinValue, mMaxValue);
    return true;
}
#pragma endregion

}//namespace xvt