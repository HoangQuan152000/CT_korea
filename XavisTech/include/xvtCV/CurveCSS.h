#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

const float EPS = 2.2204e-16f;

namespace xvt {
namespace shape {
//#pragma mark Curves Utilities
template<typename T, typename V>

XVT_EXPORTS
void PolyLineSplit(const std::vector<cv::Point_<T> >& pl, std::vector<V>& contourx, std::vector<V>& contoury)
{
	contourx.resize(pl.size());
	contoury.resize(pl.size());

	for (int j = 0; j < pl.size(); j++)
	{
		contourx[j] = (V)(pl[j].x);
		contoury[j] = (V)(pl[j].y);
	}
}

template<typename T, typename V>

XVT_EXPORTS
void PolyLineMerge(std::vector<cv::Point_<T> >& pl, const std::vector<V>& contourx, const std::vector<V>& contoury)
{
	assert(contourx.size() == contoury.size());
	pl.resize(contourx.size());
	for (int j = 0; j < contourx.size(); j++)
	{
		pl[j].x = (T)(contourx[j]);
		pl[j].y = (T)(contoury[j]);
	}
}

template<typename T, typename V>

XVT_EXPORTS
void ConvertCurve(const std::vector<cv::Point_<T> >& curve, std::vector<cv::Point_<V> >& output)
{
	output.clear();
	for (int j = 0; j < curve.size(); j++)
	{
		output.push_back(cv::Point_<V>(curve[j].x, curve[j].y));
	}
}

XVT_EXPORTS
void ResampleCurve(const std::vector<double>& curvex, const std::vector<double>& curvey,
				   std::vector<double>& resampleX, std::vector<double>& resampleY,
				   int N, bool isOpen = false
);

template<typename T>

XVT_EXPORTS
void drawOpenCurve(cv::Mat& img, const std::vector<cv::Point_<T> >& curve, cv::Scalar color, int thickness)
{
	std::vector<cv::Point> curve2i;
	ConvertCurve(curve, curve2i);
	for (int i = 0; i < curve2i.size() - 1; i++)
	{
		line(img, curve2i[i], curve2i[i + 1], color, thickness);
	}
}

//#pragma mark CSS Image

XVT_EXPORTS
void ComputeCurveCSS(const std::vector<double>& curvex,
					 const std::vector<double>& curvey,
					 std::vector<double>& kappa,
					 std::vector<double>& smoothX, std::vector<double>& smoothY,
					 double sigma = 1.0,
					 bool isOpen = false);

XVT_EXPORTS
std::vector<int> FindCSSInterestPoints(const std::vector<double>& kappa);

XVT_EXPORTS
std::vector<int> ComputeCSSImageMaximas(const std::vector<double>& contourx_, const std::vector<double>& contoury_,
										std::vector<double>& contourx, std::vector<double>& contoury, bool isClosedCurve = true);

template<typename T>

XVT_EXPORTS
void ComputeCurveCSS(const std::vector<cv::Point_<T> >& curve,
					 std::vector<double>& kappa,
					 std::vector<cv::Point_<T> >& smooth,
					 double sigma,
					 bool isOpen = false
)
{
	std::vector<double> contourx(curve.size()), contoury(curve.size());
	PolyLineSplit(curve, contourx, contoury);

	std::vector<double> smoothx, smoothy;
	ComputeCurveCSS(contourx, contoury, kappa, smoothx, smoothy, sigma, isOpen);

	PolyLineMerge(smooth, smoothx, smoothy);
}

//#pragma mark Curve Segments


template<typename T, typename V>

XVT_EXPORTS
void GetCurveSegments(const std::vector<cv::Point_<T> >& curve, const std::vector<int>& interestPoints, std::vector<std::vector<cv::Point_<V> > >& segments, bool closedCurve = true)
{
	if (closedCurve)
	{
		segments.resize(interestPoints.size());
	}
	else
	{
		segments.resize(interestPoints.size() + 1);
	}

	for (int i = (closedCurve) ? 0 : 1; i < segments.size() - 1; i++)
	{
		int intpt_idx = (closedCurve) ? i : i - 1;
		segments[i].clear();
		for (int j = interestPoints[intpt_idx]; j < interestPoints[intpt_idx + 1]; j++)
		{
			segments[i].push_back(cv::Point_<V>(curve[j].x, curve[j].y));
		}
	}
	if (closedCurve)
	{
		//put in the segment that passes the 0th point
		segments.back().clear();
		for (int j = interestPoints.back(); j < curve.size(); j++)
		{
			segments.back().push_back(cv::Point_<V>(curve[j].x, curve[j].y));
		}
		for (int j = 0; j < interestPoints[0]; j++)
		{
			segments.back().push_back(cv::Point_<V>(curve[j].x, curve[j].y));
		}
	}
	else
	{
		//put in the segment after the last point
		segments.back().clear();
		for (int j = interestPoints.back(); j < curve.size(); j++)
		{
			segments.back().push_back(cv::Point_<V>(curve[j].x, curve[j].y));
		}
		//put in the segment before the 1st point
		segments.front().clear();
		for (int j = 0; j < interestPoints[0]; j++)
		{
			segments.front().push_back(cv::Point_<V>(curve[j].x, curve[j].y));
		}
	}
	for (int i = 0; i < segments.size(); i++)
	{
		std::vector<double> x, y;
#ifdef _SHOW_CURVECSS_INFO_
		std::cout << "segments[i].size() " << segments[i].size() << std::endl;
#endif
		PolyLineSplit(segments[i], x, y); ResampleCurve(x, y, x, y, 50, true); PolyLineMerge(segments[i], x, y);
	}
}
template<typename T, typename V>

XVT_EXPORTS
void GetCurveSegmentsWithCSSImage(std::vector<cv::Point_<T> >& curve, std::vector<int>& interestPoints, std::vector<std::vector<cv::Point_<V> > >& segments, bool closedCurve = true)
{
	std::vector<double> contourx(curve.size()), contoury(curve.size());
	PolyLineSplit(curve, contourx, contoury);

	std::vector<double> smoothx, smoothy;
	interestPoints = ComputeCSSImageMaximas(contourx, contoury, smoothx, smoothy);

	PolyLineMerge(curve, smoothx, smoothy);

	double minx, maxx; cv::minMaxLoc(smoothx, &minx, &maxx);
	double miny, maxy; cv::minMaxLoc(smoothy, &miny, &maxy);
	cv::Mat drawing(maxy, maxx, CV_8UC3, cv::Scalar(0));
	cv::RNG rng(time(NULL));
	cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	//	vector<vector<cv::Point_<T> > > contours(1,curve);
	//	drawContours( drawing, contours, 0, color, 2, 8);
	drawOpenCurve(drawing, curve, color, 2);

	for (int m = 0; m < interestPoints.size(); m++)
	{
		circle(drawing, curve[interestPoints[m]], 5, cv::Scalar(0, 255), cv::FILLED);
	}
	//cv::imshow("curve interests", drawing);
	//cv::waitKey();

	GetCurveSegments(curve, interestPoints, segments, closedCurve);
}

//#pragma mark Matching

XVT_EXPORTS
double MatchTwoSegments(const std::vector<cv::Point2d>& a, const std::vector<cv::Point2d>& b);

XVT_EXPORTS
double MatchCurvesSmithWaterman(const std::vector<std::vector<cv::Point2d> >& a, const std::vector<std::vector<cv::Point2d> >& b, std::vector<cv::Point>& traceback);

XVT_EXPORTS
double AdaptedMatchCurvesSmithWaterman(const std::vector<std::vector<cv::Point2d> >& a, const std::vector<std::vector<cv::Point2d> >& b, std::vector<cv::Point>& traceback);

XVT_EXPORTS
void checkCircleFitting(std::vector<int> crossings, std::vector<cv::Point2f> vecContourPoints, double minRadius, double maxRadius, cv::Mat ImgRes, std::vector<cv::Point2f>& vtCenter, std::vector<double>& vtRadius);

XVT_EXPORTS
std::vector<int> FindCSSInterestPoints0(const std::vector<double>& kappa);

XVT_EXPORTS
std::vector<double> computeCurvature(std::vector<cv::Point2f> vecContourPoints);

XVT_EXPORTS
void plotHist(cv::Mat img, std::string tilte);

XVT_EXPORTS
float getAccHistAtThreshold(cv::Mat image, int threshold);

XVT_EXPORTS
void fit_circle(std::vector<cv::Point2f>& pnts, cv::Point2d& centre, double& radius);

XVT_EXPORTS
bool isBoundaryContour(std::vector<cv::Point2f> vec, int width, int height);

/**
 * Get derivative in vector.
 *
 * \param in
 * \param out
 */

XVT_EXPORTS
void diff(std::vector<float> in, std::vector<float>& out);
/// <summary>
/// Multiply two vector
/// </summary>
/// <param name="a">Vector input 1</param>
/// <param name="b">Vector input 2</param>
/// <param name="out">Returned vector after multiplying vector 1 and 2</param>

XVT_EXPORTS
void vectorProduct(std::vector<float> a, std::vector<float> b, std::vector<float>& out);
/// <summary>
/// Finding index of elements which have value smaller than threshold
/// </summary>
/// <param name="in">vector</param>
/// <param name="threshold">compared value</param>
/// <param name="indices">index vector</param>

XVT_EXPORTS
void findIndicesLessThan(std::vector<float> in, float threshold, std::vector<int>& indices);
/// <summary>
/// Selecting elements in vector based on selected index
/// </summary>
/// <param name="in">vector input</param>
/// <param name="indices">selected index</param>
/// <param name="out">returned vector based on selected index</param>
template <class T>

XVT_EXPORTS
void selectElements(std::vector<T> in, std::vector<T> indices, std::vector<T>& out);
/// <summary>
/// Getting Sign of vector
/// </summary>
/// <param name="in">input vector</param>
/// <param name="out">Signed vector</param>

XVT_EXPORTS
void signVector(std::vector<float> in, std::vector<int>& out);
/**
 * %PEAKFINDER Noise tolerant fast peak finding algorithm
 * https://www.mathworks.com/matlabcentral/fileexchange/25500-peakfinder-x0-sel-thresh-extrema-includeendpoints-interpolate
 *
 * TODO:
 *  [] includeEndpoints
 *
%   INPUTS:
%       x0 - A real std::vector from the maxima will be found (required)
%       sel - The amount above surrounding data for a peak to be,
%           identified (default = (max(x0)-min(x0))/4). Larger values mean
%           the algorithm is more selective in finding peaks.
%       thresh - A threshold value which peaks must be larger than to be
%           maxima or smaller than to be minima.
%       extrema 1 if maxima are desired, -1 if minima are desired
%           (default = maxima, 1)
%       includeEndpoints - If true the endpoints will be included as
%           possible extrema otherwise they will not be included
%           (default = true)
%       interpolate - If true quadratic interpolation will be performed
%           around each extrema to estimate the magnitude and the
%           position of the peak in terms of fractional indicies. Note that
%           unlike the rest of this function interpolation assumes the
%           input is equally spaced. To recover the x_values of the input
%           rather than the fractional indicies you can do:
%           peakX = x0 + (peakLoc - 1) * dx
%           where x0 is the first x value and dx is the spacing of the
%           std::vector. Output peakMag to recover interpolated magnitudes.
%           See example 2 for more information.
%           (default = false)
%
%   OUTPUTS:
%       peakLoc - The indicies of the identified peaks in x0
%       peakMag - The magnitude of the identified peaks
%
%   [peakLoc] = peakfinder(x0) returns the indicies of local maxima that
%       are at least 1/4 the range of the data above surrounding data.
%
%   [peakLoc] = peakfinder(x0,sel) returns the indicies of local maxima
%       that are at least sel above surrounding data.
%
%   [peakLoc] = peakfinder(x0,sel,thresh) returns the indicies of local
%       maxima that are at least sel above surrounding data and larger
%       (smaller) than thresh if you are finding maxima (minima).
%
%   [peakLoc] = peakfinder(x0,sel,thresh,extrema) returns the maxima of the
%       data if extrema > 0 and the minima of the data if extrema < 0
%
%   [peakLoc] = peakfinder(x0,sel,thresh,extrema, includeEndpoints)
%       returns the endpoints as possible extrema if includeEndpoints is
%       considered true in a boolean sense
%
%   [peakLoc, peakMag] = peakfinder(x0,sel,thresh,extrema,interpolate)
%       returns the results of results of quadratic interpolate around each
%       extrema if interpolate is considered to be true in a boolean sense
%
%   [peakLoc, peakMag] = peakfinder(x0,...) returns the indicies of the
%       local maxima as well as the magnitudes of those maxima
%
%   If called with no output the identified maxima will be plotted along
%       with the input data.
%
%   Note: If repeated values are found the first is identified as the peak
%
% Example 1:
% t = 0:.0001:10;
% x = 12*sin(10*2*pi*t)-3*sin(.1*2*pi*t)+randn(1,numel(t));
% x(1250:1255) = max(x);
% peakfinder(x)
%
% Example 2:
% ds = 100;  % Downsample factor
% dt = .001; % Time step
% ds_dt = ds*dt; % Time delta after downsampling
% t0 = 1;
% t = t0:dt:5 + t0;
% x = 0.2-sin(0.01*2*pi*t)+3*cos(7/13*2*pi*t+.1)-2*cos((1+pi/10)*2*pi*t+0.2)-0.2*t;
% x(end) = min(x);
% x_ds = x(1:ds:end); % Downsample to test interpolation
% [minLoc, minMag] = peakfinder(x_ds, .8, 0, -1, false, true);
% minT = t0 + (minLoc - 1) * ds_dt; % Take into account 1 based indexing
% p = plot(t,x,'-',t(1:ds:end),x_ds,'o',minT,minMag,'rv');
% set(p(2:end), 'linewidth', 2); % Show the markers more clearly
% legend('Actual Data', 'Input Data', 'Estimated Peaks');
% Copyright Nathanael C. Yoder 2015 (nyoder@gmail.com).
 *
 */

XVT_EXPORTS
void findPeaks(std::vector<float> x0, std::vector<int>& peakInds, float selection, float thresh, int extrema, bool includeEndpoints, bool interpolate);
//Create Circle Mask

XVT_EXPORTS
cv::Mat makeCirKernel(int radius, int& nPoints, int value);
/**
 * @brief makeCanvas Makes composite image from the given images
 * @param vecMat Vector of Images.
 * @param windowHeight The height of the new composite image to be formed.
 * @param nRows Number of rows of images. (Number of columns will be calculated
 *              depending on the value of total number of images).
 * @return new composite image.
 */

XVT_EXPORTS
cv::Mat makeCanvas(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows);

//Finding Largest contour based on the contour area

XVT_EXPORTS
void findinglargestcontour(cv::Mat image, std::vector<std::vector<cv::Point>>& vtr_contours, std::vector<cv::Point>& largest_contour, double& area_largest_contour, int& index_of_largest_area);

}
}