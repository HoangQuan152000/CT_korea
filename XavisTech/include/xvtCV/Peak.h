#pragma once
#include "xvtCV/Drawing.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtDefine.h"
#include <opencv2/core.hpp>

namespace xvt {

struct Peak
{
    int index=0;
    float value=0.0f;
    float prominence=0.0f;

    bool operator==(Peak const& other) const
    {
        return (index == other.index && value == other.value && prominence == other.prominence);
    }
};

enum class PeakType
{
    Valley = -1, //Prominence is negative value
    None = 0,    //Not peak or valley.
    Peak = 1,    //Prominence is positive value.
    Both = 2     //It is peak or valley.
};

enum class PeakFindingMethod
{
    Prominence
    , RelativeHeight
};

using VecPeak = std::vector<Peak>; //Vector of peak

class XVT_EXPORTS FindPeaks
{
public:
    /// <summary>
    /// Default constructor
    /// </summary>
    FindPeaks();

    /// <summary>
    /// Initialize the FindPeaks object
    /// </summary>
    /// <param name="peakType">Type of turning point which you want to find: Valley or Peak</param>
    /// <param name="findPeaksMethod">Using MATLAB Prominence method, or relative height based on period of signal</param>
    /// <param name="period">Estimated period of signal to calculate the relative height</param>
    /// <param name="includeEndPoints">Detect two endpoints as peaks or not</param>
    FindPeaks(PeakType peakType, PeakFindingMethod findPeaksMethod = PeakFindingMethod::Prominence, int period = 0, bool includeEndPoints = false);


    /// <summary>
    /// Run the Find Peak algorithm
    /// </summary>
    /// <param name="signal">Input signal</param>
    void Process(const std::vector<float>& signal);

    /// <summary>
    /// Set the Peak Type
    /// </summary>
    /// <param name="peakType">Type of turning point which you want to find: Valley or Peak</param>
    void SetPeakType(PeakType peakType);

    /// <summary>
    /// Set the Peak Finding Method
    /// </summary>
    /// <param name="peakFindingMethod">Using MATLAB Prominence method, or relative height based on period of signal</param>
    void SetPeakFindingMethod(PeakFindingMethod peakFindingMethod);

    /// <summary>
    /// Set the period
    /// </summary>
    /// <param name="period">Estimated period of signal to calculate the relative height</param>
    void SetPeriod(int period);

    /// <summary>
    /// Set the including endpoints
    /// </summary>
    /// <param name="includeEndPoints">Detect two endpoints as peaks or not</param>
    void SetIncludeEndPoints(bool includeEndPoints);

    /// <summary>
    /// Get the Peak Type
    /// </summary>
    /// <returns>Peak type</returns>
    PeakType GetPeakType() const;

    /// <summary>
    /// Get the Peak Finding Method
    /// </summary>
    /// <returns>Peak finding method</returns>
    PeakFindingMethod GetPeakFindingMethod() const;

    /// <summary>
    /// Get the period
    /// </summary>
    /// <returns>Period</returns>
    int GetPeriod() const;

    /// <summary>
    /// Get the Find Peak Result
    /// </summary>
    /// <param name="minProminence">Minimum Prominence of peaks</param>
    /// <param name="minDistance">Minimum Distance between peaks</param>
    /// <param name="peakRefinement">Using Peak Refinement</param>
    /// <returns>Filtered Peaks list</returns>
    std::vector<Peak> GetPeakResult(float minProminence = 0, float minDistance = 0, float peakRefinement = 0) const;

    /// <summary>
    /// Get Include Endpoints status
    /// </summary>
    /// <returns>Include Endpoints</returns>
    bool GetIncludeEndPoints() const;

    /// <summary>
    /// Return all the peak candidates
    /// </summary>
    /// <returns>List of peaks</returns>
    auto GetPeaks() & ->std::vector<Peak> const&
    {
        return mPeakList;
    }

    auto GetPeaks() && ->std::vector<Peak>
    {
        return std::move(mPeakList);
    }
private:
    std::vector<float> mSignal;
    PeakType mPeakType;
    PeakFindingMethod mPeakFindingMethod;
    std::vector<Peak> mPeakList;
    int mPeriod;
    bool mIncludeEndPoints;

    /// <summary>
    /// Find the peak and valley candidates, base on the left and right difference. If diffLeft*diffright > 0 it is a peak.
    /// </summary>
    /// <param name="peakCandidatesList">output the list of peak candidate</param>
    /// <param name="valleyCandidatesList">output the list of valley candidate</param>
    void FindPeaksCandidate(std::vector<Peak>& peakCandidatesList, std::vector<Peak>& valleyCandidatesList) const;

    /// <summary>
    /// Find the left and right index where we can find the reference point around the peak/valley
    /// </summary>
    /// <param name="peakPoint">: peak/valley that we want to find the reference point</param>
    /// <param name="leftClosestSubPeak"></param>
    /// <param name="rightClosestSubPeak"></param>
    /// <returns>two index: left idx, right idx</returns>
    std::pair<int, int> FindTwoIntervals(const Peak& peakPoint, const Peak& leftClosestSubPeak, const Peak& rightClosestSubPeak) const;

    /// <summary>
    /// Find the left and right index where we can find the reference point around the peak/valley
    /// </summary>
    /// <param name="signal">: Input signal</param>
    /// <param name="isPeakFinder">: true for max value(peak), false for min value (valley)</param>
    /// <param name="peakPoint">: peak/valley that we want to find the reference point</param>
    /// <param name="leftClosestSubPeak"></param>
    /// <param name="rightClosestSubPeak"></param>
    ///	<param name="period"></param>
    /// <returns>two index&lt;left dix, right idx&gt;</returns>
    std::pair<int, int> findTwoIntervalsRelativeHeight(const std::vector<float>& signal, PeakType peaktype, Peak peakPoint, Peak leftClosestSubPeak, Peak rightClosestSubPeak, int period);

    /// <summary>
    /// Find the reference point of a peak to calculate the relative height by searching the left and right side of its.
    /// </summary>
    /// <param name="peakPoint">peak/valley that we want to find the reference point</param>
    /// <param name="twoIntervals">two index range. twoIntervals.first for left side searching minimum/maximum range. twoIntervals.second for right side searching minimum/maximum range</param>
    /// <param name="leftClosestSubPeak"></param>
    /// <param name="rightClosestSubPeak"></param>
    /// <returns>A reference point to calculate prominence/relative height</returns>
    std::pair<int, float> FindKeyCol(const Peak& peakPoint, std::pair<int, int> twoIntervals, const Peak& leftClosestSubPeak, const Peak& rightClosestSubPeak) const;

    /// <summary>
    /// Find the reference point of a peak to calculate the relative height by searching the left and right side of its.
    /// </summary>
    /// <param name="signal">: Input signal</param>
    /// <param name="isPeakFinder">: true for max value(peak), false for min value (valley)</param>
    /// <param name="peakPoint">: peak/valley that we want to find the reference point</param>
    /// <param name="twoIntervals">: two index range. twoIntervals.first for left side searching minimum/maximum range. twoIntervals.second for right side searching minimum/maximum range</param>
    /// <param name="leftClosestSubPeak"></param>
    /// <param name="rightClosestSubPeak"></param>
    /// <returns>reference point&lt;index, value&gt;</returns>
    std::pair<int, float> findKeyColRelativeHeight(const std::vector<float>& signal, PeakType peaktype, Peak peakPoint, std::pair<int, int> twoIntervals, Peak leftClosestSubPeak, Peak rightClosestSubPeak);
    /// <summary>
    /// Remove peak if it too close to others
    /// </summary>
    /// <param name="filteredPeakList">Peak list</param>
    /// <param name="minDistance">Minimum distance between peaks</param>
    void RemoveClosePeaks(std::vector<Peak>& filteredPeakList, float minDistance) const;

    /// <summary>
    /// Remove low prominence/relative height peaks
    /// </summary>
    /// <param name="filteredPeakList">Peak list</param>
    /// <param name="minProminence">Minimum prominence/relative height</param>
    void RemoveLowProminencePeaks(std::vector<Peak>& filteredPeakList, float minProminence) const;

    /// <summary>
    /// Find the peak base on the relative height. This is a modified version of findPeakByProminence
    /// </summary>
    /// <param name="signal">: Input signal</param>
    /// <param name="peaktype"></param>
    /// <param name="minProminence">: threshold for consider one as a peak/valley</param>
    /// <param name="minDistance">: distance so that the peaks in this range will be removed</param>
    /// <param name="period">: assumed period of signal</param>
    /// <param name="p">: percent from the peak/valley value for refinding the peak postion. if p==0 it will not perform refind</param>
    /// <param name="includeEndpoints: true as the start and end point will be consider as a peak/valleyparam>
    /// <returns>peaklist&lt;index, value&gt;</returns>
    std::vector<Peak> findPeakByRelativeHeight(const std::vector<float>& signal, PeakType peaktype, float minProminence, int minDistance, int period, float p, bool includeEndpoints = false);

    /// <summary>
    /// Refine peaks list
    /// </summary>
    /// <param name="filteredPeakList">Peak list</param>
    /// <param name="peakRefinement">Peak Refinement parameter</param>
    void RefinePeaks(std::vector<Peak>& filteredPeakList, float peakRefinement) const;

    /// <summary>
    /// Draw prominence graph for debugging
    /// </summary>
    /// <param name="filteredPeakList">Peak list</param>
    void DrawGraph(std::vector<Peak>& filteredPeakList) const;
};

/// <summary>
/// Draw the poles position, signal and prominence information
/// </summary>
/// <param name="src">Input Image</param>
/// <param name="signal">Input signal</param>
/// <param name="lstPeak"></param>
/// <param name="drawTool"></param>
/// <param name="centerNeglectionWidth"></param>
XVT_EXPORTS
cv::Mat DrawPeakInfo(const cv::Mat& src, std::vector<float> const& signal, std::vector<Peak> const& lstPeak, Drawing & drawTool, int centerNeglectionWidth = 0);

XVT_EXPORTS
Drawing DrawPeaks(cv::Mat& img, VecFloat const& sig, VecPeak const& lstPeak, CVPen pen);

inline
bool SmallerValue(Peak const& rhs, Peak const& lsh)
{
    return rhs.value < lsh.value;
}

inline
bool GreaterValue(Peak const& rhs, Peak const& lsh)
{
    return rhs.value > lsh.value;
}

inline
bool SmallerProminence(Peak const& rhs, Peak const& lsh)
{
    return rhs.prominence < lsh.prominence;
}

inline
bool GreaterProminence(Peak const& rhs, Peak const& lsh)
{
    return rhs.prominence > lsh.prominence;
}

}//namespace xvt