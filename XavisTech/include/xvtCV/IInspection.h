#pragma once
#include "xvtCV/xvtEnumTemplate.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtCSV.h"
#include <opencv2/core/types.hpp>
#include <string>

namespace xvt {

enum class EResult
{
    UC = -1, //Un check/ not inspect yet
    OK =  0, //OK result
    NG,      //NG result
    ER       //Some Errors existed
};
DEFINE_CONVERT_ENUM_FUNCS(EResult);

/* Update the mResult follow inputResult.
    |       |result1 |-1      |0    |1    |2       |
    |result2|        |UC      |OK   |NG   |ERROR   |
    |-----  |--------|--------|--   |--   |--------|
    |-1     |UC      |UC      |OK   |NG   |ERROR   |
    |0      |OK      |OK      |OK   |NG   |ERROR   |
    |1      |NG      |NG      |NG   |NG   |ERROR   |
    |2      |ERROR   |ERROR   |ERROR|ERROR|ERROR   |
*/
inline
EResult CombineResult(EResult const& result1, EResult const& result2)
{
    //auto res = (result2 >= EResult::ER)
    //    || ((result2 == EResult::NG) && (EResult::ER != result1))
    //    || ((result2 == EResult::OK) && (EResult::UC == result1))
    //    ? result2 : result1;

    /*auto res = (result2 == EResult::ER || result1 == EResult::ER) ? EResult::ER :
        ((result2 == EResult::NG || result1 == EResult::NG) ? EResult::NG :
         (result2 == EResult::OK || result1 == EResult::OK) ? EResult::OK : EResult::UC
         );*/

    return result2 > result1 ? result2 : result1;
}

inline
EResult operator&(EResult const& result1, EResult const& result2)
{
    return CombineResult(result1, result2);
}

inline
EResult operator&=(EResult& result1, EResult const& result2)
{
    result1 = CombineResult(result1, result2);
    return result1;
}

XVT_EXPORTS
inline
cv::Scalar ChoseColor(EResult result)
{
    cv::Scalar color = cv::Scalar(125, 125, 125);;
    switch (result)
    {
    case xvt::EResult::OK:
        color = cv::Scalar(0, 255, 0);
        break;
    case xvt::EResult::UC:
        color = cv::Scalar(125, 125, 125);
        break;
    case xvt::EResult::NG:
    case xvt::EResult::ER:
    default:
        color = cv::Scalar(0, 0, 255);
        break;
    }

    return color;
}

//Interface for inspection result
class XVT_EXPORTS IInspectionResult
{
public:
    //Return 0 if OK otherwise it is NG or ERROR
    virtual auto GetResult() const -> EResult = 0;
    //Set the result
    virtual void SetResult(EResult const& result) = 0;
    //Set the result, true->OK, false->NG
    virtual void SetResult(bool const& result) = 0;

    //Get the message when ERROR happen, help to cactch ERROR
    virtual auto GetMsg() const& -> std::string const& = 0;
    virtual auto GetMsg() && -> std::string = 0;
    virtual void SetMsg(std::string const& msg = "") = 0;
    virtual void SetMsg(std::string && msg) = 0;
    //Draw the object results into img
    virtual void DrawResult(cv::Mat& img, cv::Point offSetPoint=cv::Point(), CVPen pen= CVPen()) const = 0;
    //Draw the message results into img
    virtual auto DrawMsg(cv::Mat& image, CVPen const& pen, cv::Point const& offset) const->cv::Point = 0;

    virtual void SetROI(cv::Rect const& roi) = 0;
    virtual auto GetROI() const->cv::Rect = 0;

    virtual ~IInspectionResult() = 0 {};

    //Methode for clone the object
    virtual auto Clone() const -> IInspectionResult * = 0;

    virtual auto GetResultStr() const->std::string
    {
        return xvt::ToString(GetResult());
    }

    virtual auto ToString() const -> std::string
    {
        auto msg = GetMsg();
        auto r = GetResultStr();
        auto str = msg.empty() ? r : r + " " + msg;

        return str;
    }

    bool IsOK() const { return GetResult() == EResult::OK; }
    bool IsNG() const { return GetResult() == EResult::NG; }
    bool IsER() const { return GetResult() == EResult::ER; }
    bool IsUC() const { return GetResult() == EResult::UC; }

    void operator()(EResult const& result, std::string const& msg)
    {
        SetResult(result);
        SetMsg(msg);
    }

    void CombineResult(EResult const& result)
    {
        SetResult(xvt::CombineResult(GetResult(), result));
    }

    //Combine the result when the input is a boolean, true->OK, false->NG
    void CombineResult(bool const& result)
    {
        SetResult(xvt::CombineResult(GetResult(), result ? EResult::OK : EResult::NG));
    }

    void CombineMsg(std::string const& msg)
    {
        auto str = GetMsg();
        auto str3 = str.empty() || msg.empty() ? "" : "\n";
        SetMsg(str + str3 + msg);
    }

    void AddMsg(std::string const& msg)
    {
        CombineMsg(msg);
    }

    void CombineResult(IInspectionResult const* result)
    {
        CombineResult(result->GetResult());
        CombineMsg(result->GetMsg());
    }

    void GetResult(EResult& result, std::string& msg)
    {
        result = GetResult();
        msg = GetMsg();
    }

    auto GetResultColor() const -> cv::Scalar { return ChoseColor(GetResult()); }

    //Combine result when input is boolean, true->OK, false->NG
    IInspectionResult& operator&=(bool const& other)
    {
        CombineResult(other);
        return *this;
    }

    IInspectionResult& operator&=(EResult const& other)
    {
        CombineResult(other);
        return *this;
    }

    IInspectionResult& operator&=(IInspectionResult const& other)
    {
        CombineResult(&other);
        return *this;
    }
};

class XVT_EXPORTS InspectionResult :
     public IInspectionResult
    ,public CSV
{
public:
    explicit InspectionResult(xvt::EResult result=xvt::EResult::UC, std::string const& msg = "");
    // Inherited via IInspectionResult
    virtual void SetROI(cv::Rect const& roi) override { mROI = roi; }
    virtual auto GetROI() const -> cv::Rect override { return mROI; }

    //Set the result
    virtual void SetResult(EResult const& result) override
    {
        mResult = result;
    }
    //Set the result, true->OK, false->NG
    virtual void SetResult(bool const& result) override
    {
        mResult = result ? EResult::OK : EResult::NG;
    }

    virtual auto GetResult() const -> xvt::EResult override
    {
        return mResult;
    }
    virtual auto GetResultStr()const->std::string override;
    virtual auto GetMsg() const& -> std::string const& override
    {
        return mMsg;
    }
    virtual auto GetMsg() && -> std::string override
    {
        return std::move(mMsg);
    }
    virtual void SetMsg(std::string const& msg = "")
    {
        mMsg = msg;
    }
    virtual void SetMsg(std::string && msg)
    {
        std::swap(mMsg, msg);
    }
    virtual auto Clone() const -> xvt::IInspectionResult* override
    {
        return new InspectionResult(*this);
    }
    virtual void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;
    virtual auto DrawResultStr(cv::Mat& image
                               , std::string const& name=""
                               , CVPen const& pen = CVPen()
                               , cv::Point const& pos = cv::Point()
                               , bool isDrawOKResult = false
    ) const->cv::Point;
    virtual auto DrawMsg(cv::Mat& image, CVPen const& pen, cv::Point const& pos) const->cv::Point override;

    //CSV support functions
    virtual auto GetTitleStr()const->std::string override
    {
        return std::string("Inspection Result\n");
    }

    virtual
    //Transform the line base on the tranform matrix m
    auto Transform(cv::Mat const& m)&->void;

    //Return tl, tr, br, bl
    auto GetCornerPoint() const->std::array<cv::Point, 4>;

    //Return vector index of 4 corner point
    auto GetCornerPointIdx() const->std::vector<int>;
public:
    bool mEnable = true;

#ifdef _DEBUG
    bool mShowRoi = true;
#else
    bool mShowRoi = false;
#endif // _DEBUG
    //Inspection Points
    std::vector<cv::Point> mPoints;
    //Procesing time in ms
    double mProTime=0.0;
    std::string mDateTime="";
private:
    cv::Rect    mROI;
    EResult     mResult=EResult::UC;
    std::string mMsg;
};

class XVT_EXPORTS IInspection
{
public:
    virtual ~IInspection() = 0 {};
    virtual auto Inspect(cv::Mat const& src) const->IInspectionResult * =0;
    //Method for clone the object
    //virtual IInspection* Clone() const = 0;
};

}

