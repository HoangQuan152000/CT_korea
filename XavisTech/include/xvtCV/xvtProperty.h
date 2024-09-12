#pragma once
#include "xvtCV/xvtRange.h"
#include "xvtCV/xvtTypes.h"
#include <string>
#include <vector>
#include <memory>
#include <limits.h>

#define _XVT_WSTR_INI_JOIN_CHAR(ext) L"_" ext
#define _XVT_WSTR_INI_JOIN(name, ext) name + (_XVT_WSTR_INI_JOIN_CHAR(ext))
#define _XVT_WSTR_INI_JOIN2(name, ext) name _XVT_WSTR_INI_JOIN_CHAR(ext)

#define _XVT_WSTR_ENABLE L"enable"

#define _XVT_WSTR_FROM   L"from"
#define _XVT_WSTR_TO     L"to"
#define _XVT_WSTR_LOWER  L"lower"
#define _XVT_WSTR_UPPER  L"upper"

#define _XVT_WSTR_X      L"x"
#define _XVT_WSTR_Y      L"y"
#define _XVT_WSTR_W      L"width"
#define _XVT_WSTR_H      L"height"

//ROI
#define _XVT_WSTR_INI_ROIX(name) name _XVT_WSTR_INI_JOIN_CHAR(_XVT_WSTR_X)
#define _XVT_WSTR_INI_ROIY(name) name _XVT_WSTR_INI_JOIN_CHAR(_XVT_WSTR_Y)
#define _XVT_WSTR_INI_ROIW(name) name _XVT_WSTR_INI_JOIN_CHAR(_XVT_WSTR_W)
#define _XVT_WSTR_INI_ROIH(name) name _XVT_WSTR_INI_JOIN_CHAR(_XVT_WSTR_H)
//Range
#define _XVT_WSTR_INI_RANGE_FROM(name) name _XVT_WSTR_INI_JOIN_CHAR(_XVT_WSTR_FROM)
#define _XVT_WSTR_INI_RANGE_TO(name) name _XVT_WSTR_INI_JOIN_CHAR(_XVT_WSTR_TO)

namespace xvt {

class IProperty
{
public:
    virtual ~IProperty() {}

    virtual bool SaveINI(const std::wstring& iniFile, const std::wstring& section) const = 0;

    virtual bool LoadINI(const std::wstring& iniFile, const std::wstring& section) = 0;

    virtual std::wstring GetName() const = 0;
};

template< typename T >
class TypedProperty : public IProperty
{
public:
    explicit TypedProperty(std::wstring const& name
                           , T& data
                           , T defaultValue
                           , T minValue = (std::numeric_limits<T>::min)()
                           , T maxValue = (std::numeric_limits<T>::max)()
    )
        : mName{ name }, mData{ std::ref(data) }, mDefaultValue{ defaultValue }, mMinValue{ minValue }, mMaxValue{ maxValue }
    {};

    explicit TypedProperty(std::wstring const& name
                           , T&& data
                           , T defaultValue
                           , T minValue = (std::numeric_limits<T>::min)()
                           , T maxValue = (std::numeric_limits<T>::max)()
    ) = delete;

    bool SaveINI(const std::wstring& iniFile, const std::wstring& section) const override;
    bool LoadINI(const std::wstring& iniFile, const std::wstring& section) override;
    std::wstring GetName() const override
    {
        return mName;
    }

public:
    T mDefaultValue;
    T mMinValue;
    T mMaxValue;
    std::wstring mName;
    std::reference_wrapper<T> mData;
};

using TypedProperty_i = TypedProperty<int>;
using TypedProperty_f = TypedProperty<float>;
using TypedProperty_d = TypedProperty<double>;

using TypedProperty_Pointi = TypedProperty<cv::Point_<int>>;
using TypedProperty_Pointf = TypedProperty<cv::Point_<float>>;
using TypedProperty_Pointd = TypedProperty<cv::Point_<double>>;

using TypedProperty_Recti = TypedProperty<cv::Rect_<int>>;
using TypedProperty_Rectf = TypedProperty<cv::Rect_<float>>;
using TypedProperty_Rectd = TypedProperty<cv::Rect_<double>>;

using TypedProperty_Sizei = TypedProperty<cv::Size_<int>>;
using TypedProperty_Sizef = TypedProperty<cv::Size_<float>>;
using TypedProperty_Sized = TypedProperty<cv::Size_<double>>;

using TypedProperty_Rangei = TypedProperty<xvt::Range<int>>;
using TypedProperty_Rangef = TypedProperty<xvt::Range<float>>;
using TypedProperty_Ranged = TypedProperty<xvt::Range<double>>;

template<typename T>
class TypedProperty< std::basic_string<T, std::char_traits<T>, std::allocator<T>> > : public IProperty
{
public:
    using type_trait = std::basic_string<T, std::char_traits<T>, std::allocator<T>>;
    explicit TypedProperty(std::wstring const& name
                           , type_trait& data
                           , type_trait defaultValue
    )
        : mName{ name }, mData{ std::ref(data) }, mDefaultValue{ defaultValue }
    {};

    explicit TypedProperty(std::wstring const& name
                           , type_trait&& data
                           , type_trait defaultValue
    ) = delete;

    bool SaveINI(const std::wstring& iniFile, const std::wstring& section) const override;
    bool LoadINI(const std::wstring& iniFile, const std::wstring& section) override;
    std::wstring GetName() const override
    {
        return mName;
    }

public:
    type_trait mDefaultValue;
    std::wstring mName;
    std::reference_wrapper<type_trait> mData;
};

//Function to create the shared pointer of TypedProperty
template<typename T> inline
auto CreateShareProperty(std::wstring const& name, T& value, T const& defaultValue)->std::shared_ptr<TypedProperty<T>>
{
    return std::make_shared<TypedProperty<T>>(name, std::ref(value), defaultValue);
}

class PropertyList : public std::vector<std::shared_ptr<IProperty>>, public IProperty
{
using VecSharedProperty = std::vector<std::shared_ptr<IProperty>>;
using InitialSharedProperty = std::initializer_list<std::shared_ptr<IProperty>>;
public:
    PropertyList() : VecSharedProperty() {};
    PropertyList(VecSharedProperty const& vec) : VecSharedProperty(vec) {};
    PropertyList(VecSharedProperty&& vec) : VecSharedProperty(vec) {};

    explicit
    PropertyList(int n) : VecSharedProperty(n) {};
    PropertyList(std::initializer_list<std::shared_ptr<IProperty>> const& initList) : VecSharedProperty(initList) {}
    PropertyList(std::initializer_list<std::shared_ptr<IProperty>>&& initList) : VecSharedProperty(initList) {}

    PropertyList(PropertyList const& other) : VecSharedProperty(other) {};
    PropertyList(PropertyList&& other) noexcept : VecSharedProperty(other) {};

    ~PropertyList() { VecSharedProperty::~vector(); }
    
    PropertyList& operator=(PropertyList const& other) 
    {
        VecSharedProperty::operator=(other); 
        return *this;
    };

    PropertyList& operator=(PropertyList && other) noexcept 
    { 
        VecSharedProperty::operator=(other);
        return *this;
    };

    virtual bool SaveINI(const std::wstring& iniFile) const
    {
        return SaveINI(iniFile, mName);
    }

    virtual bool SaveINI(const std::wstring& iniFile, const std::wstring& section) const
    {
        for (auto& p : *this)
        {
            p->SaveINI(iniFile, section);
        }
        return true;
    }

    virtual bool LoadINI(const std::wstring& iniFile)
    {
        return LoadINI(iniFile, mName);
    }

    virtual bool LoadINI(const std::wstring& iniFile, const std::wstring& section)
    {
        for (auto& p : *this)
        {
            p->LoadINI(iniFile, section);
        }
        return true;
    }

    std::wstring GetName() const override
    {
        return mName;
    }

public:
    std::wstring mName = L"Default";
};

template<typename T>
class TypedProperty<cv::Point_<T>> : public IProperty
{
public:
    explicit TypedProperty(std::wstring const& name
                           , cv::Point_<T>& data
                           , cv::Point_<T> defaultValue
                           , cv::Point_<T> minValue = cv::Point_<T>()
                           , cv::Point_<T> maxValue = cv::Point_<T>((std::numeric_limits<T>::max)(), (std::numeric_limits<T>::max)())
    );

    bool SaveINI(const std::wstring& iniFile, const std::wstring& section) const override;
    bool LoadINI(const std::wstring& iniFile, const std::wstring& section) override;
    std::wstring GetName() const override
    {
        return mName;
    }

public:
    cv::Point_<T> mDefaultValue;
    cv::Point_<T> mMinValue;
    cv::Point_<T> mMaxValue;
    std::wstring mName;
    std::wstring mNameX;
    std::wstring mNameY;
    std::reference_wrapper<cv::Point_<T>> mData;
};

template<typename T>
class TypedProperty<cv::Rect_<T>> : public IProperty
{
public:
    explicit TypedProperty(std::wstring const& name
                           , cv::Rect_<T>& data
                           , cv::Rect_<T> defaultValue
    );

    explicit TypedProperty(std::wstring const& name
                           , cv::Rect_<T>&& data
                           , cv::Rect_<T> defaultValue
    )=delete;

    bool SaveINI(const std::wstring& iniFile, const std::wstring& section) const override;
    bool LoadINI(const std::wstring& iniFile, const std::wstring& section) override;
    std::wstring GetName() const override
    {
        return mName;
    }

public:
    cv::Rect_<T> mDefaultValue;
    cv::Rect_<T> mMinValue;
    cv::Rect_<T> mMaxValue;
    std::wstring mName;
    std::wstring mNameX;
    std::wstring mNameY;
    std::wstring mNameW;
    std::wstring mNameH;
    std::reference_wrapper<cv::Rect_<T>> mData;
};

template<typename T>
class TypedProperty<cv::Size_<T>> : public IProperty
{
public:
    explicit TypedProperty(std::wstring const& name
                           , cv::Size_<T>& data
                           , cv::Size_<T> defaultValue
    );

    explicit TypedProperty(std::wstring const& name
                           , cv::Size_<T>&& data
                           , cv::Size_<T> defaultValue
    )=delete;

    bool SaveINI(const std::wstring& iniFile, const std::wstring& section) const override;
    bool LoadINI(const std::wstring& iniFile, const std::wstring& section) override;
    std::wstring GetName() const override
    {
        return mName;
    }

public:
    cv::Size_<T> mDefaultValue;
    cv::Size_<T> mMinValue;
    cv::Size_<T> mMaxValue;
    std::wstring mName;
    std::wstring mNameW;
    std::wstring mNameH;
    std::reference_wrapper<cv::Size_<T>> mData;
};

template<typename T>
class TypedProperty<xvt::Range<T>> : public IProperty
{
public:
    explicit TypedProperty(std::wstring const& name
                           , xvt::Range<T>& data
                           , xvt::Range<T> defaultValue
    );

    explicit TypedProperty(std::wstring const& name
                           , xvt::Range<T>&& data
                           , xvt::Range<T> defaultValue
    )=delete;

    bool SaveINI(const std::wstring& iniFile, const std::wstring& section) const override;
    bool LoadINI(const std::wstring& iniFile, const std::wstring& section) override;
    std::wstring GetName() const override
    {
        return mName;
    }

public:
    xvt::Range<T> mDefaultValue;
    std::wstring mName;
    std::wstring mNameFrom;
    std::wstring mNameTo;
    std::wstring mNameEnable;
    std::reference_wrapper<xvt::Range<T>> mData;
};

}//namspace xvt

#include "xvtProperty.inl.h"