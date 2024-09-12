#pragma once
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtRansac.h"
#include "xvtCV/IInspection.h"
#include <opencv2/core/types.hpp>

namespace xvt {

class XVT_EXPORTS Polynomial
{
public:
    Polynomial() = default;
    Polynomial(int n) { SetOrder(n); };

    Polynomial(VecDouble const& vec) : mCoeff{ vec } {};
    Polynomial(VecDouble&& vec) : mCoeff{ vec } {};

    Polynomial(std::initializer_list<double> const& initList) : mCoeff{ initList } {}
    Polynomial(std::initializer_list<double>&& initList) : mCoeff{ initList } {}

    auto GetOrder()const -> int { return (std::max)((int)mCoeff.size() - 1, 0); }
    auto SetOrder(int n) -> void { mCoeff = VecDouble((std::max)(n, 0), std::numeric_limits<double>::infinity()); }

    auto GetCoeff() const& ->VecDouble const& { return mCoeff; }
    auto GetCoeff() & ->VecDouble & { return mCoeff; }
    auto GetCoeff() && -> VecDouble const { return std::move(mCoeff); }


    auto f(double x) const ->double { return f(mCoeff, x); }
    auto f(VecDouble const& x) const->VecDouble { VecDouble y; f(mCoeff, x, y); return y; }
    void f(VecDouble const& x, VecDouble& y) const { f(mCoeff, x, y); }

    auto GetRMSE(double x, double y)const->double;
    auto GetRMSE(cv::Point2d p)const->double;
    auto GetRMSE(VecPoint p)const->double;
    auto GetRMSE(VecDouble const& x,VecDouble const& y)const->double;

    //Return the value of f(x)
    static auto f(VecDouble const& coeff, double x)->double;
    static void f(VecDouble const& coeff, VecDouble const& x, VecDouble& y);

    static auto MakeDataMatrix(VecDouble const& xdata, int order)->cv::Mat;

    static auto GetRMSE(cv::Mat const& A, cv::Mat const& coeff, cv::Mat const& ydata)->double;

private:
    //Co-efficient vector
    VecDouble mCoeff;
};

class XVT_EXPORTS PolynomialResult :
      public Polynomial
    , public InspectionResult
{
public:
    PolynomialResult() = default;
    PolynomialResult(int n) :Polynomial(n), InspectionResult(){};

    PolynomialResult(VecDouble const& vec) : Polynomial(vec), InspectionResult() {};
    PolynomialResult(VecDouble && vec) : Polynomial(vec), InspectionResult() {};

    PolynomialResult(std::initializer_list<double> const& initList) : Polynomial(initList), InspectionResult() {};
    PolynomialResult(std::initializer_list<double> && initList) : Polynomial(initList), InspectionResult() {};

    void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

};

class XVT_EXPORTS PolynomialDetector : public Ransac<PolynomialResult>
{
public:
    PolynomialDetector() : RansacType(2, 1.0, 5) {};
    PolynomialDetector(int order) : RansacType(order + 1, 1.0, 5), mOrder{ order }{};

    auto FitLSQE(xvt::VecPoint const& points) const->ModelResultType override;
    auto IsValidModel(ModelResultType const& res) const->bool override;

    static     
    auto FitLSQE(std::vector<double>const& xdata, std::vector<double>const& ydata, int order)->ModelResultType;

    int mOrder=0;
};

class XVT_EXPORTS Polyfit : public Polynomial
{
public:
    Polyfit() = default;

    Polyfit(std::vector<double>const& xdata, std::vector<double>const& ydata, int order);

    Polyfit(const std::vector<cv::Point>& points, int order);

    double GetRMS() const
    {
        return mRMS;
    }

    //This fitting used leastsquare method. This function only fits to the order bigger than 0
    //	Input: xdata, ydata, size of the data and the order of the output fitting polinomial. 
    //	Output: coeff array with the smallest order first: Coeff[0] +  Coeff[1]*x^1 +...+ Coeff[Order]*x^Order 
    static auto LSQFit(std::vector<double>const& xdata, std::vector<double>const& ydata, int order, std::vector<double>& coeff)->double;
    static auto LSQFit(std::vector<double>const& xdata, std::vector<double>const& ydata, int order, std::vector<double>& coeff, double& error)->bool;

    static auto LSQFit(const std::vector<cv::Point>& points, int order, std::vector<double>& coeff)->double;
    static bool LSQFit(const std::vector<cv::Point>& points, int order, std::vector<double>& coeff, double& error);

#pragma region not verify functions
public:
    //Find the maximum of f(xi): f(xi) | xi=[xStart:xEnd:step]. It will time out when run 1000 iteration even not find the maximum
    // Input: 
    //	 min_max <= 0 find minimum other find the maximum
    //   coeff: array with the smallest order first: Coeff[0] +  Coeff[1]*x^1 +...+ Coeff[Order]*x^Order.
    //   order: order of the polynomial.
    //   xStart, xEnd: range to find the maximum.
    //   e: acuracy. step: number of step in range.
    // Output:
    //   xMax: x position.
    //   maxValue: y value.
    static auto GetMinMax(VecDouble const& coeff
        , int min_max
        , float xStart, float xEnd
        , double& xMax, double& maxValue
        , double e = 1.e-3, int step = 1000
    )->double;

    //xStart is the start position
    //Change "alpha" to change the momemtum
    //min_max <= 0 find minimum other find the maximum
    static auto GetMaxMinDescent(VecDouble const& coeff
        , int min_max, float xStart
        , double& xMax, double& maxValue
        , float alpha = 0.09, int maxIteritor = 90000
        , double e = 1.e-7
    )->double;

private:
    //This function only works with order bigger than 0
    static VecDouble GetFirstDerivativeCoeff(VecDouble const& coeff);
#pragma endregion

private:
    static double GetRMS(cv::Mat const& A, cv::Mat const& coeff, cv::Mat const& ydata);

private:
    //Root-Mean-Square error
    double mRMS = std::numeric_limits<double>::infinity();
};

}
