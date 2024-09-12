#include "ImageProcessorBase.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
//#define new DEBUG_NEW
#endif

#define GAUSS_SIZE	(7)
#define GAUSS_SIGMA	(1.0)
#define SOBEL_ORDER	(1)
#define SOBEL_SIZE	(3)

using namespace std;

namespace xf
{
	namespace ImageProcessor
	{
		CImageProcessorBase::CImageProcessorBase()
		{
		}

		CImageProcessorBase::~CImageProcessorBase()
		{
		}

		std::string CImageProcessorBase::to_string_with_precision(double a_value, int n)
		{
			std::ostringstream out;
			out.precision(n);
			out << std::fixed << a_value;
			return out.str();
		}

		void CImageProcessorBase::plotHist(cv::Mat img, std::string tilte) {
			cv::Mat hist;
			int histSize = 64;
			float range[] = { 0, 255 }; //the upper boundary is exclusive
			const float* histRange = { range };
			calcHist(&img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, true);
			hist = hist / (img.rows * img.cols);
			// Compute the CDF of histogram
			cv::Mat cdfHist = hist.clone();
			int cdfIdx = 1;
			cdfHist.at<float>(0) = hist.at<float>(histSize - 1);
			for (int i = histSize; i > 1; i--) {
				cdfHist.at<float>(cdfIdx) += cdfHist.at<float>(cdfIdx - 1);
				cdfIdx++;
			}

			int xAx = 30, yAx = 30;
			int hist_w = 768 + yAx, hist_h = 450;
			cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
			hist_h -= xAx;
			hist_w -= yAx;
			int bin_w = cvRound((double)hist_w / histSize);
			// nomalized the graph the fit the size of windows
			normalize(hist, hist, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());
			for (int i = 1; i < histSize; i++)
			{
				rectangle(histImage, cv::Point(yAx + bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
					cv::Point(yAx + bin_w * (i), hist_h),
					cv::Scalar(255, 0, 0), 2, 8, 0);
				if (i % 2 == 0)
				{
					putText(histImage, std::to_string(i * 256 / histSize), cv::Point(yAx + bin_w * (i - 0.5), hist_h + xAx - 10), 1, 0.4, cv::Scalar(0, 0, 255));
					putText(histImage, std::to_string((int)hist.at<float>(i - 1)), cv::Point(yAx + bin_w * (i - 0.8), hist_h - cvRound(hist.at<float>(i - 1)) - 10), 1, 0.5, cv::Scalar(0, 255, 255));
				}
			}
			cv::namedWindow(tilte, cv::WINDOW_NORMAL);
			imshow(tilte, histImage);
			//cv::imwrite(tilte + ".bmp", histImage);
		}

		cv::Rect CImageProcessorBase::RoiRefinement(cv::Rect selectROI, cv::Size imageSize) {
			int d_x = selectROI.x < 0 ? 0 : selectROI.x;
			int d_y = selectROI.y < 0 ? 0 : selectROI.y;
            int roi_width = selectROI.width < 0 ? 0 : selectROI.width;
            int roi_height = selectROI.height < 0 ? 0 : selectROI.height;

            int d_width = (static_cast<std::int64_t>(d_x) + roi_width < imageSize.width) ? roi_width : (imageSize.width - 1 - d_x);
            int d_height = (static_cast<std::int64_t>(d_y) + roi_height < imageSize.height) ? roi_height : (imageSize.height - 1 - d_y);

			if (d_x >= imageSize.width) {
				d_x = imageSize.width - 1;
				d_width = 0;
			}
			if (d_y >= imageSize.height) {
				d_y = imageSize.height - 1;
				d_height = 0;
			}

			return cv::Rect(d_x, d_y, d_width, d_height);
		}

		float CImageProcessorBase::getAccHistAtThreshold(cv::Mat image, int threshold) {
			cv::Mat hist;
			int histSize = 256;
			float range[] = { 0, 255 }; //the upper boundary is exclusive
			const float* histRange = { range };
			calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, true);
			hist = hist / (image.rows * image.cols);
			// Compute the CDF of histogram
			float cdfHist = 0.0;
			int cdfIdx = 1;
			cdfHist += hist.at<float>(0);
			for (int i = 1; i < threshold; i++) {
				cdfHist += hist.at<float>(i);
			}
			return cdfHist;
		}

		void CImageProcessorBase::fit_circle(std::vector<cv::Point>& pnts, cv::Point2d& centre, double& radius)
		{
			int cols = 3;
			cv::Mat X(static_cast<int>(pnts.size()), cols, CV_64F);
			cv::Mat Y(static_cast<int>(pnts.size()), 1, CV_64F);
			cv::Mat C;

			if (int(pnts.size()) >= 3)
			{
				for (size_t i = 0; i < pnts.size(); i++)
				{
					X.at<double>(static_cast<int>(i), 0) = 2 * pnts[i].x;
					X.at<double>(static_cast<int>(i), 1) = 2 * pnts[i].y;
					X.at<double>(static_cast<int>(i), 2) = -1.0;
					Y.at<double>(static_cast<int>(i), 0) = (pnts[i].x * pnts[i].x + pnts[i].y * pnts[i].y);
				}
				cv::solve(X, Y, C, cv::DECOMP_SVD);
			}
			std::vector<double> coefs;
			C.col(0).copyTo(coefs);
			centre.x = coefs[0];
			centre.y = coefs[1];
			radius = sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1] - coefs[2]);
		}

		inline double gauss(double sigma, double x) {
			double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
			double divider = sqrt(2 * M_PI * pow(sigma, 2));
			return (1 / divider) * exp(expVal);
		}

		std::vector<double> gaussKernel(int samples, double sigma) {
			std::vector<double> v;

			bool doubleCenter = false;
			if (samples % 2 == 0) {
				doubleCenter = true;
				samples--;
			}
			int steps = (samples - 1) / 2;
			double stepSize = (3 * sigma) / steps;

			for (int i = steps; i >= 1; i--) {
				v.push_back(gauss(sigma, i * stepSize * -1));
			}

			v.push_back(gauss(sigma, 0));
			if (doubleCenter) {
				v.push_back(gauss(sigma, 0));
			}

			for (int i = 1; i <= steps; i++) {
				v.push_back(gauss(sigma, i * stepSize));
			}
			assert(v.size() == samples);

			return v;
		}

		std::vector<double> CImageProcessorBase::weightedAvgSmoothen(std::vector<double> values) {
			std::vector<double> output;
			int lstSize = values.size();
			if (lstSize > 2) {
				output.push_back(values.front());
				for (int idx = 1; idx < lstSize - 1; idx++) {
					output.push_back(0.2 * values[idx - 1] + 0.6 * values[idx] + 0.2 * values[idx + 1]);
				}
				output.push_back(values.back());
				return output;
			}
			else
				return values;
		}

		std::vector<double> CImageProcessorBase::gaussSmoothen(std::vector<double> values, double sigma, int samples) {
			std::vector<double> out;
			if (values.size() < samples) {
				return out;
			}
			auto kernel = gaussKernel(samples, sigma);
			int sampleSide = samples / 2;
			double sumKernel = 0.0;
			for (int kIdx = 0; kIdx < kernel.size(); kIdx++) {
				sumKernel += kernel[kIdx];
			}
			double kernelScaleFactor = 1.0 / sumKernel;
			for (int kIdx = 0; kIdx < kernel.size(); kIdx++) {
				kernel[kIdx] *= kernelScaleFactor;
			}
			/*kernel[sampleSide] += (1.0 - sumKernel);*/

			int valueIdx = samples / 2 + 1;
			unsigned long ubound = values.size();
			std::vector<double> tmpValues(values);
			for (int i = 0; i < sampleSide; i++) {
				tmpValues.insert(tmpValues.begin(), values[ubound - i - 1]);
			}
			for (int i = 0; i < sampleSide; i++) {
				tmpValues.push_back(values[sampleSide * 2 - i]);
			}
			for (unsigned long i = sampleSide; i < ubound + sampleSide; i++) {
				double sample = 0;
				int sampleCtr = 0;
				for (long j = i - sampleSide; j <= i + sampleSide; j++) {
					if (j >= 0 && j < ubound + sampleSide * 2) {
						int sampleWeightIndex = sampleSide + (j - i);
						sample += kernel[sampleWeightIndex] * tmpValues[j];
						sampleCtr++;
					}
				}
				double smoothed = sample / (double)sampleCtr;
				//std::cout << " S: " << sample << " C: " << sampleCtr << " V: " << values[i] << " SM: " << smoothed << std::endl;
				out.push_back(sample);
			}
			return out;
		}

		cv::Point CImageProcessorBase::rotate180Point(cv::Point pntIn, cv::Point center) {
			cv::Point pntRotated;
			
			//Rotated a point around the center
			float pX = center.x - (pntIn.x - center.x);
			float pY = center.y - (pntIn.y - center.y);
			pntRotated = cv::Point(pX, pY);

			return pntRotated;
		}

		void CImageProcessorBase::WrProcessingData(std::string &wStr, int manualProcCount, int wrPoleNo, double poleDiffQueue[][MAX_POLE_NO], double poleDiffLength[], double poleDiffLength2[], double poleDiffLength3[], int commaNum)
		{
			for (int i = 0; i < wrPoleNo; i++) wStr += std::to_string(poleDiffLength[i]) + ",";
			for (int i = wrPoleNo; i < MAX_POLE_NO; i++) wStr += ",";
			wStr += ",";

			for (int i = 0; i < wrPoleNo; i++) wStr += std::to_string(poleDiffLength2[i]) + ",";
			for (int i = wrPoleNo; i < MAX_POLE_NO; i++) wStr += ",";
			wStr += ",";

			for (int i = 0; i < wrPoleNo; i++) wStr += std::to_string(poleDiffLength3[i]) + ",";
			for (int i = wrPoleNo; i < MAX_POLE_NO; i++) wStr += ",";

			return;

			int k = manualProcCount % MAX_QUEUE_COUNT;
			for (int j = 0; j < MAX_POLE_NO; j++) poleDiffQueue[k][j] = poleDiffLength[j];

			double minMin, maxMax, meanSum, stddevSum, maxminDiffSum;
			minMin = 10.0; maxMax = meanSum = stddevSum = maxminDiffSum = 0.0;

			if ((manualProcCount % MAX_QUEUE_COUNT == 0) && (manualProcCount > 1))
			{
				double dPolMinValue[MAX_POLE_NO], dPolMaxValue[MAX_POLE_NO], dPolMeanValue[MAX_POLE_NO], dPolStdDevValue[MAX_POLE_NO], dPolDiffValue[MAX_POLE_NO];
				std::string dPolGood[MAX_POLE_NO];
				int noOfGood = 0, noOfNG = 0;

				for (int j = 0; j < wrPoleNo; j++)
				{
					double minValue = 10.0;
					double maxValue = 0;

					double dataSum = 0;
					int dataNum = 0;

					for (int i = 0; i < MAX_QUEUE_COUNT; i++)
					{
						if ((minValue > poleDiffQueue[i][j]) && (poleDiffQueue[i][j] > -0.001)) minValue = poleDiffQueue[i][j];
						if ((maxValue < poleDiffQueue[i][j]) && (poleDiffQueue[i][j] > -0.001)) maxValue = poleDiffQueue[i][j];

						if (poleDiffQueue[i][j] > -0.001)
						{
							dataSum += poleDiffQueue[i][j];
							dataNum ++;
						}
					}
					double mean = dataSum / dataNum;
					dPolMeanValue[j] = mean;

					double sq_sum = 0;
					for (int i = 0; i < MAX_QUEUE_COUNT; i++)
					{
						if (poleDiffQueue[i][j] > -0.001) sq_sum += (poleDiffQueue[i][j] - mean) * (poleDiffQueue[i][j] - mean);
					}

					double stdev = 0.0;
					stdev = std::sqrt(sq_sum / dataNum);
					dPolStdDevValue[j] = stdev;

					dPolMinValue[j] = (minValue > 9.999) ? 0 : minValue;
					dPolMaxValue[j] = maxValue;
					dPolDiffValue[j] = dPolMaxValue[j] - dPolMinValue[j];

					if (minMin > minValue) minMin = minValue;
					if (maxMax < maxValue) maxMax = maxValue;

					meanSum += mean;
					stddevSum += stdev;
					maxminDiffSum += maxValue - minValue;

					if ((fabs(dPolDiffValue[j]) / 2 < GRR_CHECK_RESOL) && (minValue < 9.999))
					{
						dPolGood[j] = "G";
						noOfGood++;
					}
					else
					{
						if (minValue < 9.999)
						{
							dPolGood[j] = "N";
							noOfNG++;
						}
						else
						{
							dPolGood[j] = "-";
						}
					}
				}
				float avgMaxMinDiff = 0;
				float maxMaxMinDiff = 0;
				for (int j = 0; j < wrPoleNo; j++)
				{
					if (fabs(dPolDiffValue[j]) > 0.00001) avgMaxMinDiff += dPolDiffValue[j];
					if (fabs(dPolDiffValue[j]) > maxMaxMinDiff) maxMaxMinDiff = dPolDiffValue[j];
				}
				avgMaxMinDiff /= noOfGood + noOfNG;

				wStr += "\n";
				for (int i = 0; i < commaNum; i++) wStr += " ,";  // 중간에 띄어쓰는 개수
				wStr += "MAX,";
				for (int j = 0; j < wrPoleNo; j++) wStr += std::to_string(dPolMaxValue[j]) + ",";
				wStr += "\n";

				for (int i = 0; i < commaNum; i++) wStr += " ,";  // 중간에 띄어쓰는 개수
				wStr += "MIN,";
				for (int j = 0; j < wrPoleNo; j++) wStr += std::to_string(dPolMinValue[j]) + ",";
				wStr += "\n";

				for (int i = 0; i < commaNum; i++) wStr += " ,";  // 중간에 띄어쓰는 개수
				wStr += "Mean,";
				for (int j = 0; j < wrPoleNo; j++) wStr += std::to_string(dPolMeanValue[j]) + ",";
				wStr += "\n";

				for (int i = 0; i < commaNum; i++) wStr += " ,";  // 중간에 띄어쓰는 개수
				wStr += "StdDev,";
				for (int j = 0; j < wrPoleNo; j++) wStr += std::to_string(dPolStdDevValue[j]) + ",";
				wStr += "\n";

				for (int i = 0; i < commaNum - 2; i++) wStr += " ,";  // 중간에 띄어쓰는 개수
				wStr += "Max:" + std::to_string(maxMaxMinDiff) + ",";
				wStr += "Avg:" + std::to_string(avgMaxMinDiff) + ",";
				wStr += "MAX-MIN,";
				for (int j = 0; j < wrPoleNo; j++) wStr += std::to_string(dPolDiffValue[j]) + ",";
				wStr += "\n";

				for (int i = 0; i < commaNum - 1; i++) wStr += " ,";  // 중간에 띄어쓰는 개수
				wStr += "G:" + std::to_string(noOfGood) + "/N:" + std::to_string(noOfNG) + ",";
				wStr += "GOOD/NoGood,";
				for (int j = 0; j < wrPoleNo; j++) wStr += dPolGood[j] + ",";
				wStr += "\n";

				for (int i = 0; i < commaNum - 1; i++) wStr += " ,";  // 중간에 띄어쓰는 개수
				wStr += "Max," + std::to_string(maxMax) + ",Min," + std::to_string(minMin);
				wStr += ",,Mean," + std::to_string(meanSum / wrPoleNo) + ",StdDev," + std::to_string(stddevSum / wrPoleNo);
				wStr += ",,Max-Min," + std::to_string(maxMax - minMin);
				wStr += "\n";

				for (int i = 0; i < MAX_QUEUE_COUNT; i++) for (int j = 0; j < MAX_POLE_NO; j++) poleDiffQueue[i][j] = -1.0;
			}
		}

		void CImageProcessorBase::GRR_Cal_EV_AV(double* poleDiffLen, double &dEV, double &dAV)
		{
			double minRange[MAX_SAMPLE_NO][3];
			double maxRange[MAX_SAMPLE_NO][3];
			double rRange[MAX_SAMPLE_NO][3];
			int nNG_PoleNo[MAX_SAMPLE_NO][3];

			for (int j = 0; j < MAX_SAMPLE_NO; j++) nNG_PoleNo[j][0] = nNG_PoleNo[j][1] = nNG_PoleNo[j][2] = 0;

			double tmpValue;
			// 1, 2, 3 처리

			int rptNoMulSampleNo = MAX_SAMPLE_NO * MAX_POLE_NO;
			for (int j = 0; j < MAX_SAMPLE_NO; j++)
			{
				int offset = j * MAX_POLE_NO;
				tmpValue = poleDiffLen[0 * rptNoMulSampleNo + offset];

				if (tmpValue < 0) {  // 첫번째 불량
					tmpValue = poleDiffLen[1 * rptNoMulSampleNo + offset];
					nNG_PoleNo[j][0] ++;

					if (tmpValue < 0) { // 두번째 불량
						tmpValue = poleDiffLen[2 * rptNoMulSampleNo + offset];
						nNG_PoleNo[j][0] ++;

						if (tmpValue < 0)  // 세번째 불량
						{
							rRange[j][0] = maxRange[j][0] = minRange[j][0] = 0;
							nNG_PoleNo[j][0] ++;
						}
						else  // 세번째 정상
						{
							rRange[j][0] = 0;
							maxRange[j][0] = minRange[j][0] = tmpValue;
						}
					}
					else {  // 두번째 정상
						tmpValue = poleDiffLen[2 * rptNoMulSampleNo + offset];

						if (tmpValue < 0)  // 세번째 불량
						{
							rRange[j][0] = 0;
							maxRange[j][0] = minRange[j][0] = poleDiffLen[1 * rptNoMulSampleNo + offset];
						}
						else  // 세번째 정상
						{
							minRange[j][0] = maxRange[j][0] = poleDiffLen[1 * rptNoMulSampleNo + offset];

							if (minRange[j][0] > tmpValue)
							{
								minRange[j][0] = tmpValue;
								rRange[j][0] = maxRange[j][0] - minRange[j][0];
							}
							else
							{
								maxRange[j][0] = tmpValue;
								rRange[j][0] = maxRange[j][0] - minRange[j][0];
							}
						}
					}
				}
				else   // 첫번째 정상
				{
					minRange[j][0] = maxRange[j][0] = poleDiffLen[0 * rptNoMulSampleNo + offset];

					tmpValue = poleDiffLen[1 * rptNoMulSampleNo + offset];

					if (tmpValue < 0) { // 두번째 불량 있음
						tmpValue = poleDiffLen[2 * rptNoMulSampleNo + offset];
						nNG_PoleNo[j][0] ++;

						if (tmpValue < 0)  // 세번째 불량
						{
							nNG_PoleNo[j][0] ++;
							rRange[j][0] = 0;
						}
						else   // 세번째 정상
						{
							if (minRange[j][0] > tmpValue) minRange[j][0] = tmpValue;
							else 						   maxRange[j][0] = tmpValue;

							rRange[j][0] = maxRange[j][0] - minRange[j][0];
						}
					}
					else {   // 첫번째 정상, 두번째도 정상
						tmpValue = poleDiffLen[2 * rptNoMulSampleNo + offset];

						if (tmpValue < 0)  // 세번째 불량
						{
							nNG_PoleNo[j][0] ++;

							if (minRange[j][0] > poleDiffLen[1 * rptNoMulSampleNo + offset]) minRange[j][0] = poleDiffLen[1 * rptNoMulSampleNo + offset];
							else 								maxRange[j][0] = poleDiffLen[1 * rptNoMulSampleNo + offset];

							rRange[j][0] = maxRange[j][0] - minRange[j][0];
						}
						else    // 모두 정상
						{
							if (minRange[j][0] > poleDiffLen[1 * rptNoMulSampleNo + offset]) minRange[j][0] = poleDiffLen[1 * rptNoMulSampleNo + offset];
							if (minRange[j][0] > poleDiffLen[2 * rptNoMulSampleNo + offset]) minRange[j][0] = poleDiffLen[2 * rptNoMulSampleNo + offset];
							if (maxRange[j][0] < poleDiffLen[1 * rptNoMulSampleNo + offset]) maxRange[j][0] = poleDiffLen[1 * rptNoMulSampleNo + offset];
							if (maxRange[j][0] < poleDiffLen[2 * rptNoMulSampleNo + offset]) maxRange[j][0] = poleDiffLen[2 * rptNoMulSampleNo + offset];
							rRange[j][0] = maxRange[j][0] - minRange[j][0];
						}
					}
				}

				//--- 4,5,6번 처리
				tmpValue = poleDiffLen[3 * rptNoMulSampleNo + offset];

				if (tmpValue < 0) {  // 4번째 불량
					tmpValue = poleDiffLen[4 * rptNoMulSampleNo + offset];
					nNG_PoleNo[j][1] ++;

					if (tmpValue < 0) { // 5번째 불량
						tmpValue = poleDiffLen[5 * rptNoMulSampleNo + offset];
						nNG_PoleNo[j][1] ++;

						if (tmpValue < 0)  // 6번째 불량
						{
							rRange[j][1] = maxRange[j][1] = minRange[j][1] = 0;
							nNG_PoleNo[j][1] ++;
						}
						else  // 6번째 정상
						{
							rRange[j][1] = 0;
							maxRange[j][1] = minRange[j][1] = tmpValue;
						}
					}
					else {  // 5번째 정상
						tmpValue = poleDiffLen[5 * rptNoMulSampleNo + offset];

						if (tmpValue < 0)  // 6번째 불량
						{
							rRange[j][1] = 0;
							maxRange[j][1] = minRange[j][1] = poleDiffLen[4 * rptNoMulSampleNo + offset];
						}
						else  // 6번째 정상
						{
							minRange[j][1] = maxRange[j][1] = poleDiffLen[4 * rptNoMulSampleNo + offset];

							if (minRange[j][1] > tmpValue) minRange[j][1] = tmpValue;
							else 						   maxRange[j][1] = tmpValue;
							
							rRange[j][1] = maxRange[j][1] - minRange[j][1];
						}
					}
				}
				else   // 4번째 정상
				{
					minRange[j][1] = maxRange[j][1] = poleDiffLen[3 * rptNoMulSampleNo + offset];

					tmpValue = poleDiffLen[4 * rptNoMulSampleNo + offset];

					if (tmpValue < 0) { // 5번째 불량 있음
						tmpValue = poleDiffLen[5 * rptNoMulSampleNo + offset];
						nNG_PoleNo[j][1] ++;

						if (tmpValue < 0)  // 6번째 불량
						{
							nNG_PoleNo[j][1] ++;
							rRange[j][1] = 0;
						}
						else   // 6번째 정상
						{
							if (minRange[j][1] > tmpValue) minRange[j][1] = tmpValue;
							else						   maxRange[j][1] = tmpValue;

							rRange[j][1] = maxRange[j][1] - minRange[j][1];
						}
					}
					else {   // 4번째 정상, 5번째도 정상
						tmpValue = poleDiffLen[5 * rptNoMulSampleNo + offset];

						if (tmpValue < 0)  // 6번째 불량
						{
							nNG_PoleNo[j][1] ++;

							if (minRange[j][1] > poleDiffLen[4 * rptNoMulSampleNo + offset]) minRange[j][1] = poleDiffLen[4 * rptNoMulSampleNo + offset];
							else 								maxRange[j][1] = poleDiffLen[4 * rptNoMulSampleNo + offset];
							
							rRange[j][1] = maxRange[j][1] - minRange[j][1];
						}
						else    // 4, 5, 6 모두 정상
						{
							if (minRange[j][1] > poleDiffLen[4 * rptNoMulSampleNo + offset]) minRange[j][1] = poleDiffLen[4 * rptNoMulSampleNo + offset];
							if (minRange[j][1] > poleDiffLen[5 * rptNoMulSampleNo + offset]) minRange[j][1] = poleDiffLen[5 * rptNoMulSampleNo + offset];
							if (maxRange[j][1] < poleDiffLen[4 * rptNoMulSampleNo + offset]) maxRange[j][1] = poleDiffLen[4 * rptNoMulSampleNo + offset];
							if (maxRange[j][1] < poleDiffLen[5 * rptNoMulSampleNo + offset]) maxRange[j][1] = poleDiffLen[5 * rptNoMulSampleNo + offset];
							rRange[j][1] = maxRange[j][1] - minRange[j][1];
						}
					}
				}

				//--- 7, 8, 9번 처리
				tmpValue = poleDiffLen[6 * rptNoMulSampleNo + offset];

				if (tmpValue < 0) {  // 7번째 불량
					tmpValue = poleDiffLen[7 * rptNoMulSampleNo + offset];
					nNG_PoleNo[j][2] ++;

					if (tmpValue < 0) { // 8번째 불량
						tmpValue = poleDiffLen[8 * rptNoMulSampleNo + offset];
						nNG_PoleNo[j][2] ++;

						if (tmpValue < 0)  // 9번째 불량
						{
							rRange[j][2] = maxRange[j][2] = minRange[j][2] = 0;
							nNG_PoleNo[j][2] ++;
						}
						else  // 9번째 정상
						{
							rRange[j][2] = 0;
							maxRange[j][2] = minRange[j][2] = tmpValue;
						}
					}
					else {  // 8번째 정상
						tmpValue = poleDiffLen[8 * rptNoMulSampleNo + offset];

						if (tmpValue < 0)  // 9번째 불량
						{
							rRange[j][2] = 0;
							maxRange[j][2] = minRange[j][2] = poleDiffLen[7 * rptNoMulSampleNo + offset];
						}
						else  // 9번째 정상
						{
							minRange[j][2] = maxRange[j][2] = poleDiffLen[7 * rptNoMulSampleNo + offset];

							if (minRange[j][2] > tmpValue) minRange[j][2] = tmpValue;
							else 						   maxRange[j][2] = tmpValue;
							
							rRange[j][2] = maxRange[j][2] - minRange[j][2];
						}
					}
				}
				else   // 7번째 정상
				{
					minRange[j][2] = maxRange[j][2] = poleDiffLen[6 * rptNoMulSampleNo + offset];

					tmpValue = poleDiffLen[7 * rptNoMulSampleNo + offset];

					if (tmpValue < 0) { // 8번째 불량 있음
						tmpValue = poleDiffLen[8 * rptNoMulSampleNo + offset];
						nNG_PoleNo[j][2] ++;

						if (tmpValue < 0)  // 9번째 불량
						{
							nNG_PoleNo[j][2] ++;
							rRange[j][2] = 0;
						}
						else   // 9번째 정상
						{
							if (minRange[j][2] > tmpValue) minRange[j][2] = tmpValue;
							else					       maxRange[j][2] = tmpValue;
							
							rRange[j][2] = maxRange[j][2] - minRange[j][2];
						}
					}
					else {   // 7번째 정상, 8번째도 정상
						tmpValue = poleDiffLen[8 * rptNoMulSampleNo + offset];

						if (tmpValue < 0)  // 9번째 불량
						{
							nNG_PoleNo[j][2] ++;

							if (minRange[j][2] > poleDiffLen[7 * rptNoMulSampleNo + offset]) minRange[j][2] = poleDiffLen[7 * rptNoMulSampleNo + offset];
							else 													         maxRange[j][2] = poleDiffLen[7 * rptNoMulSampleNo + offset];
							
							rRange[j][2] = maxRange[j][2] - minRange[j][2];
						}
						else    // 7, 8, 9 모두 정상
						{
							if (minRange[j][2] > poleDiffLen[7 * rptNoMulSampleNo + offset]) minRange[j][2] = poleDiffLen[7 * rptNoMulSampleNo + offset];
							if (minRange[j][2] > poleDiffLen[8 * rptNoMulSampleNo + offset]) minRange[j][2] = poleDiffLen[8 * rptNoMulSampleNo + offset];
							if (maxRange[j][2] < poleDiffLen[7 * rptNoMulSampleNo + offset]) maxRange[j][2] = poleDiffLen[7 * rptNoMulSampleNo + offset];
							if (maxRange[j][2] < poleDiffLen[8 * rptNoMulSampleNo + offset]) maxRange[j][2] = poleDiffLen[8 * rptNoMulSampleNo + offset];
							rRange[j][2] = maxRange[j][2] - minRange[j][2];
						}
					}
				}
			}
			double sum0[3] = { 0.0, 0.0, 0.0 };
			int nTotalNG[3] = { 0, 0, 0 };

			for (int j = 0; j < MAX_SAMPLE_NO; j++)
			{
				for (int k = 0; k < 3; k++) {
					sum0[k] += rRange[j][k];
					nTotalNG[k] += nNG_PoleNo[j][k];
				}
			}

			double dR_dbar = sum0[0] / (30. - (double)nTotalNG[0]) + sum0[1] / (30. - (double)nTotalNG[1]) + sum0[2] / (30.0 - (double)nTotalNG[2]);
			dEV = dR_dbar * 3.042;

			double sum1[3] = { 0.0, 0.0, 0.0 };
			int nNG_No[3] = { 0, 0, 0 };

			for (int j = 0; j < MAX_SAMPLE_NO; j++)
			{
				int offset = j * MAX_POLE_NO;

				for (int k = 0; k < 9; k++)
				{
					if (poleDiffLen[k * rptNoMulSampleNo + offset] > 0.) {
						sum1[k%3] += poleDiffLen[k * rptNoMulSampleNo + offset];
					}
					else nNG_No[k%3] ++;
				}
			}
			for (int j=0; j<3; j++) sum1[j] = sum1[j] / (double)(MAX_SAMPLE_NO * 3. - (double)nNG_No[j]);

			double maxAvg = sum1[0];
			double minAvg = sum1[0];

			if (maxAvg < sum1[1]) maxAvg = sum1[1];
			if (maxAvg < sum1[2]) maxAvg = sum1[2];
			if (minAvg > sum1[1]) minAvg = sum1[1];
			if (minAvg > sum1[2]) minAvg = sum1[2];
			double X_bar_diff = maxAvg - minAvg;

			double dAV_tmp = (X_bar_diff * 2.696) * (X_bar_diff * 2.696) - dEV * dEV / 30.0;
			dAV = (dAV_tmp > 0) ? sqrt(dAV_tmp) : 0.0;
		}

		cv::Rect CImageProcessorBase::CellWidthCalcul(cv::Mat Img, cv::Rect Roi, BOOL displayMode)
		{
			//===================Find contour of the battery======================//

			cv::Mat grayImg, dst;
			cv::medianBlur(Img(Roi), dst, 5);
			cv::threshold(dst, grayImg, 0, 255, cv::THRESH_OTSU);

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			findContours(~grayImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			if (displayMode > 0) {
				namedWindow("CellGray", cv::WINDOW_NORMAL);
				cv::imshow("CellGray", grayImg);
			}

			//===================Select only the ball area======================//
			double largestWidth = 0;
			int largestcontour = 0;

			for (int i = 0; i < contours.size(); i++)
			{
				cv::Rect boundary = cv::boundingRect(contours[i]);
				if (boundary.width > largestWidth)
				{
					largestWidth = boundary.width;
					largestcontour = i;
				}
			}
			cv::Rect cellRect = cv::boundingRect(contours[largestcontour]);

			if ((contours.size() == 0) || (cellRect.x < 10)) {  // 에러시에 다시 threshold 조건 바꾸어 실행
				cv::threshold(Img, grayImg, 50., 255, cv::THRESH_BINARY);
				cv::medianBlur(grayImg, grayImg, 5);
				findContours(~grayImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

				if (displayMode > 0) {
					namedWindow("CellGray2", cv::WINDOW_NORMAL);
					cv::imshow("CellGray2", grayImg);
				}

				largestWidth = largestcontour = 0;

				for (int i = 0; i < contours.size(); i++)
				{
					cv::Rect boundary = cv::boundingRect(contours[i]);
					if (boundary.width > largestWidth)
					{
						largestWidth = boundary.width;
						largestcontour = i;
					}
				}
				cellRect = cv::boundingRect(contours[largestcontour]);

				if (contours.size() == 0)
				{
					cv::Rect retRect(0, 0, 0, 0);
					return retRect;
				}
			}

			if (displayMode > 0) {
				cv::Mat drawingContour = cv::Mat::zeros(grayImg.size(), CV_8UC3);;
				for (size_t cIdx = 0; cIdx < contours.size(); cIdx++) {
					cv::Rect boundary = cv::boundingRect(contours[largestcontour]);
					cv::drawContours(drawingContour, contours, (int)cIdx, cv::Scalar(255, 0, 0), 4);
					cv::rectangle(drawingContour, boundary, cv::Scalar(0, 0, 255), 4);
				}

				namedWindow("contour2", cv::WINDOW_NORMAL);
				cv::imshow("contour2", drawingContour);

				printf("cellRect rect x = %d y = %d width = %d height = %d\n", cellRect.x, cellRect.y, cellRect.width, cellRect.height);
			}

			return cellRect;
		}

		cv::Rect CImageProcessorBase::BallOutsideBoundFinding(cv::Mat Img, cv::Mat grayImg, cv::Rect SettingRoi, int displayMode)   // 1 or 2mm ball 찾기
		{
			//===================우선 ball 찾기 : Find contour of the battery======================//

			cv::Rect retBallRect;

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			findContours(~grayImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			//===================Select only the ball area======================//
			double largestWidth = -1.0;
			int contourIdx = 0;

			for (int i = 0; i < contours.size(); i++)
			{
				cv::Rect boundary = cv::boundingRect(contours[i]);
				if ((boundary.width != 0) && (boundary.height != 0))
				{
					double ratio = (double)boundary.width / (double)boundary.height;

					if ((ratio > 0.8) && (ratio < 1.2) && (boundary.width > 45) & (boundary.width < 600))  // 340, 1 ~ 5mm, 15 ~ 21um
					{
						if (largestWidth < boundary.width)
						{
							largestWidth = boundary.width;
							contourIdx = i;
						}
					}
				}
			}
			if (largestWidth < 0.0) return(cv::Rect(0, 0, 0, 0));

			if (contours.size() > 0)
			{
				retBallRect = cv::boundingRect(contours[contourIdx]);

				if (displayMode > 0) {
					cout << "Contour size = " << contours.size() << endl;

					cv::Mat contourImg = cv::Mat::zeros(grayImg.size(), CV_8UC3);;
					for (size_t cIdx = 0; cIdx < contours.size(); cIdx++) {
						cv::drawContours(contourImg, contours, (int)cIdx, cv::Scalar(255, 0, 0), 3);
					}
					cv::rectangle(contourImg, retBallRect, cv::Scalar(0, 0, 255), 3);

					namedWindow("contour", cv::WINDOW_NORMAL);
					cv::imshow("contour", contourImg);
				}

				retBallRect.x += SettingRoi.x;
				retBallRect.y += SettingRoi.y;
				retBallRect.x -= 1; retBallRect.y -= 1; retBallRect.width += 2;	retBallRect.height += 2; // 외곽으로 1픽셀 정도 작게 잡혀 늘려줌

				cout << "Ball rect x = " << retBallRect.x << ", y =" << retBallRect.y << ", w =" << retBallRect.width << ", h =" << retBallRect.height << endl;
			}
			else
			{
				retBallRect = cv::Rect(0, 0, 0, 0);
			}
			return(retBallRect);
		}

		cv::Rect CImageProcessorBase::RectangleSteelOutsideBoundFinding(cv::Mat Img, cv::Mat grayImg, cv::Rect SettingRoi, int displayMode)   // 6 ~ 8mm rectangle steel, depth 2mm 찾기
		{
			cv::Rect retRect;

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			findContours(~grayImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			//===================Select only the steel rectangle area======================//
			double largestWidth = -1.0;
			int contourIdx = 0;

			for (int i = 0; i < contours.size(); i++)
			{
				cv::Rect boundary = cv::boundingRect(contours[i]);
				if ((boundary.width != 0) && (boundary.height != 0))
				{
					double ratio = (double)boundary.width / (double)boundary.height;

					if ((ratio > 2.1) && (boundary.height > 120) & (boundary.height < 250))  // 2 mm, 10 ~ 15um
					{
						if (largestWidth < boundary.width)
						{
							largestWidth = boundary.width;
							contourIdx = i;
						}
					}
				}
			}
			if (largestWidth < 0) return(cv::Rect(0, 0, 0, 0));

			if (contours.size() > 0)
			{
				retRect = cv::boundingRect(contours[contourIdx]);

				if (displayMode > 0) {
					cout << "Contour size = " << contours.size() << endl;

					cv::Mat recContourImg = cv::Mat::zeros(grayImg.size(), CV_8UC3);;
					for (size_t cIdx = 0; cIdx < contours.size(); cIdx++) {
						cv::drawContours(recContourImg, contours, (int)cIdx, cv::Scalar(255, 0, 0), 3);
					}
					cv::rectangle(recContourImg, retRect, cv::Scalar(0, 0, 255), 3);

					namedWindow("contour", cv::WINDOW_NORMAL);
					cv::imshow("contour", recContourImg);
				}

				retRect.x += SettingRoi.x;
				retRect.y += SettingRoi.y;

				cout << "Steel rect x = " << retRect.x << ", y =" << retRect.y << ", w =" << retRect.width << ", h =" << retRect.height << endl;
			}
			else
			{
				retRect = cv::Rect(0, 0, 0, 0);
			}
			return(retRect);
		}

		std::vector<cv::Rect> CImageProcessorBase::MultiBallOutsideBoundFinding(cv::Mat Img, cv::Mat grayImg, cv::Rect SettingRoi, int displayMode, int ballNo)   // 1 or 2mm ball 찾기
		{
			//===================우선 ball 찾기 : Find contour of the battery======================//

			std::vector<cv::Rect> retBallRect;

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			findContours(~grayImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			//===================Select only the ball area======================//

			cv::Rect boundary[10];
			int findingBallNo = 0;
			cout << "Contour size = " << contours.size() << endl;
			cv::Mat contourImg = cv::Mat::zeros(grayImg.size(), CV_8UC3);;

			for (int i = 0; i < contours.size(); i++)
			{
				boundary[findingBallNo] = cv::boundingRect(contours[i]);
				if ((boundary[findingBallNo].width > 45) && (boundary[findingBallNo].width < 300))
				{
					float ratio = (float)boundary[findingBallNo].width / (float)boundary[findingBallNo].height;

					cv::Rect rRect;
					if ((ratio > 0.8) && (ratio < 1.2))  // 340, 1 ~ 3mm, 15 ~ 21um
					{
						rRect.x = boundary[findingBallNo].x + SettingRoi.x - 1;
						rRect.y = boundary[findingBallNo].y + SettingRoi.y - 1;
						rRect.width = boundary[findingBallNo].width + 2;
						rRect.height = boundary[findingBallNo].height + 2;

						cout << "Ball rect(" << findingBallNo << ") x = " << rRect.x << ", y = " << rRect.y << ", w = " << rRect.width << ", h = " << rRect.height << endl;
						retBallRect.emplace_back(rRect);
						findingBallNo++;

						if (displayMode > 0) {
							for (size_t cIdx = 0; cIdx < contours.size(); cIdx++) {
								cv::drawContours(contourImg, contours, (int)cIdx, cv::Scalar(255, 0, 0), 3);
							}
							cv::rectangle(contourImg, boundary[findingBallNo], cv::Scalar(0, 0, 255), 3);
						}
					}
				}
			}

			if ((contours.size() > 0) && (retBallRect.size() > 0))
			{
				if (displayMode > 0) {
					namedWindow("contour", cv::WINDOW_NORMAL);
					cv::imshow("contour", contourImg);
				}
			}
			else
			{
				retBallRect.emplace_back(cv::Rect(0, 0, 0, 0));
			}
			return retBallRect;
		}

		cv::Rect CImageProcessorBase::FindBatteryRoi(cv::Mat image, cv::Rect Roi, int displayMode) 
		{
			cv::Mat grayImg;
			//===================Find contour of the battery======================//
			cv::Rect Rroi;
			Rroi = CImageProcessorBase::RoiRefinement(Roi, image.size());

			cv::Mat dst;
			cv::medianBlur(image(Rroi), dst, 21);
			double otsuThresh = cv::threshold(dst, grayImg, 0, 255, cv::THRESH_OTSU);
			Rroi = CImageProcessorBase::RoiRefinement(Rroi, image.size());

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			findContours(~grayImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			//===================Select only the battery area======================//
			double largestWidth = 0;
			int largestcontour = 0;

			if (contours.size() == 0) return cv::Rect(0, 0, 0, 0);

			for (int i = 0; i < contours.size(); i++)
			{
				cv::Rect boundary = cv::boundingRect(contours[i]);
				if (boundary.width > largestWidth)
				{
					largestWidth = boundary.width;
					largestcontour = i;
				}
			}

			cv::Point extLeft = *std::min_element(contours[largestcontour].begin(), contours[largestcontour].end(),
				[](const cv::Point& lhs, const cv::Point& rhs) {
					return lhs.x < rhs.x;
				});
			cv::Point extRight = *std::max_element(contours[largestcontour].begin(), contours[largestcontour].end(),
				[](const cv::Point& lhs, const cv::Point& rhs) {
					return lhs.x < rhs.x;
				});
			cv::Point extTop = *std::min_element(contours[largestcontour].begin(), contours[largestcontour].end(),
				[](const cv::Point& lhs, const cv::Point& rhs) {
					return lhs.y < rhs.y;
				});
			cv::Rect outputROI(Rroi.x + extLeft.x, Rroi.y + extTop.y, extRight.x - extLeft.x, grayImg.rows - extTop.y);

			if (abs(outputROI.x - Roi.x) < 10)
			{
				cv::threshold(dst, grayImg, 40, 255, cv::THRESH_BINARY);
				Rroi = CImageProcessorBase::RoiRefinement(Rroi, image.size());

				findContours(~grayImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

				//===================Select only the battery area======================//
				double largestWidth = 0;
				int largestcontour = 0;

				if (contours.size() == 0) return cv::Rect(0, 0, 0, 0);

				for (int i = 0; i < contours.size(); i++)
				{
					cv::Rect boundary = cv::boundingRect(contours[i]);
					if (boundary.width > largestWidth)
					{
						largestWidth = boundary.width;
						largestcontour = i;
					}
				}

				cv::Point extLeft = *std::min_element(contours[largestcontour].begin(), contours[largestcontour].end(),
					[](const cv::Point& lhs, const cv::Point& rhs) {
						return lhs.x < rhs.x;
					});
				cv::Point extRight = *std::max_element(contours[largestcontour].begin(), contours[largestcontour].end(),
					[](const cv::Point& lhs, const cv::Point& rhs) {
						return lhs.x < rhs.x;
					});
				cv::Point extTop = *std::min_element(contours[largestcontour].begin(), contours[largestcontour].end(),
					[](const cv::Point& lhs, const cv::Point& rhs) {
						return lhs.y < rhs.y;
					});

				cv::Rect outputROI1(Rroi.x + extLeft.x, Rroi.y + extTop.y, extRight.x - extLeft.x, grayImg.rows - extTop.y);
				outputROI = outputROI1;
			}

			if (displayMode > 0) {
				std::cout << "outputROI x, y, width, height = " << outputROI.x << ", " << outputROI.y << ", " << outputROI.width << ", " << outputROI.height << std::endl;

				cv::Mat drawingContour = cv::Mat::zeros(grayImg.size(), CV_8UC3);;
				for (size_t cIdx = 0; cIdx < contours.size(); cIdx++) {
					cv::Rect boundary = cv::boundingRect(contours[largestcontour]);
					cv::drawContours(drawingContour, contours, (int)cIdx, cv::Scalar(255, 0, 0), 4);
					cv::rectangle(drawingContour, boundary, cv::Scalar(0, 0, 255), 4);
				}

				namedWindow("Binary ROI1 battery0", cv::WINDOW_NORMAL);
				cv::imshow("Binary ROI1 battery0", grayImg);

				namedWindow("ROI1 battery contour0", cv::WINDOW_NORMAL);
				cv::imshow("ROI1 battery contour0", drawingContour);

				printf("otsuThresh = %f, outputROI rect x = %d y = %d width = %d height = %d\n", otsuThresh, outputROI.x, outputROI.y, outputROI.width, outputROI.height);
			}

			return outputROI;
		}

		cv::Rect CImageProcessorBase::FindRoughBatteryROI(int *Img1, int rowNum, int colNum, cv::Rect Roi, float depthScaleFactor, bool display)  // Img1 : 16bit
		{
			static int ImgResizeArray[IMAGE_MAX_SIZE / (RESIZE_SCALE * RESIZE_SCALE)];
			int rowResizeNum = rowNum / RESIZE_SCALE;
			int colResizeNum = colNum / RESIZE_SCALE;

			cv::Mat ImgGray = cv::Mat(rowResizeNum, colResizeNum, CV_8UC1);
			cv::Mat Img1Resize = cv::Mat_<int>(rowResizeNum, colResizeNum, ImgResizeArray);

			int tmpPosition = 0;
			for (int y = 0; y < rowResizeNum; y++)
			{
				for (int x = 0; x < colResizeNum; x++)
				{
					ImgGray.at<BYTE>(y, x) = Img1[tmpPosition + x * RESIZE_SCALE] / depthScaleFactor;
				}
				tmpPosition += colNum * RESIZE_SCALE;
			}

			////////////////////////////////////////////////////////////////////////////////

			cv::Rect outputROI;

			int tempWidth = (Roi.x + Roi.width) / RESIZE_SCALE < colResizeNum ? Roi.width / RESIZE_SCALE : (colResizeNum - Roi.x / RESIZE_SCALE);
			int tempHeight = (Roi.height + Roi.y) / RESIZE_SCALE < rowResizeNum ? Roi.height / RESIZE_SCALE : (rowResizeNum - Roi.y / RESIZE_SCALE);
			cv::Rect ROI(Roi.x / RESIZE_SCALE, Roi.y / RESIZE_SCALE, tempWidth, tempHeight);

			cv::Mat grayRoiImg;
			//===================Find contour of the battery======================//

			cv::Mat dst;
			cv::medianBlur(ImgGray(ROI), dst, 15);
			double otsuThresh = cv::threshold(dst, grayRoiImg, 0, 255, cv::THRESH_OTSU);
			if (display)
			{
				cout << "otsuThresh = " << otsuThresh << endl;
			}

			cv::Mat kernelOpGray = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 7), cv::Point(-1, -1));
			cv::morphologyEx(grayRoiImg, grayRoiImg, cv::MORPH_CLOSE, kernelOpGray, cv::Point(-1, -1), 1, cv::BORDER_ISOLATED);

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			findContours(~grayRoiImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

			//===================Select only the battery area======================//
			int largestWidth = 0;
			int largestcontour = 0;
			for (int i = 0; i < contours.size(); i++)
			{
				if (display) cout << "... contours[" << i << "].size() = " << contours[i].size() << endl;
				if (contours[i].size() > 15) {

					cv::Rect boundary = cv::boundingRect(contours[i]);
					if (boundary.width > largestWidth)
					{
						largestWidth = boundary.width;
						largestcontour = i;
					}
				}
			}

			if (display) cout << "1st contour size = " << contours.size() <<", largest contour = " << largestcontour << endl;
			int okFlag = 1;
			if (contours.size() == 0)
			{
				okFlag = 0;
			}
			else {
				cv::Point extLeft = *std::min_element(contours[largestcontour].begin(), contours[largestcontour].end(),
					[](const cv::Point& lhs, const cv::Point& rhs) {
						return lhs.x < rhs.x;
					});
				cv::Point extRight = *std::max_element(contours[largestcontour].begin(), contours[largestcontour].end(),
					[](const cv::Point& lhs, const cv::Point& rhs) {
						return lhs.x < rhs.x;
					});
				cv::Point extTop = *std::min_element(contours[largestcontour].begin(), contours[largestcontour].end(),
					[](const cv::Point& lhs, const cv::Point& rhs) {
						return lhs.y < rhs.y;
					});
				outputROI = cv::Rect(ROI.x + extLeft.x, ROI.y + extTop.y, extRight.x - extLeft.x, grayRoiImg.rows);

				if (display) {
					cout << endl;
					cout << "okFlag = " << okFlag << ", extleft.x = " << extLeft.x << " extRight.x = " << extRight.x << " extTop.y = " << extTop.y << endl;
					cout << "contour size = " << contours.size() << ", outROI.width = " << outputROI.width << endl;
					cv::namedWindow("grayBinROI", cv::WINDOW_NORMAL); cv::imshow("grayBinROI", grayRoiImg);
				}
				if (outputROI.width < Roi.width * 0.3 / RESIZE_SCALE) // 주어진 ROI 폭보다 30% 이내로 작음
				{
					if (display) cout << " outROI.width is too small 1 : " << outputROI.width << endl;
					okFlag = 0;
				}
			}

			cv::Point extLeft;
			if (okFlag == 0)  // finding error and recheck with hard threshold image
			{
//				cv::imshow("gray", grayRoiImg);

				cv::threshold(dst, grayRoiImg, 90, 255, cv::THRESH_BINARY);

				cv::Mat kernelOpGray = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 7), cv::Point(-1, -1));
				cv::morphologyEx(grayRoiImg, grayRoiImg, cv::MORPH_CLOSE, kernelOpGray, cv::Point(-1, -1), 1, cv::BORDER_ISOLATED);

				std::vector<std::vector<cv::Point>> contours;
				std::vector<cv::Vec4i> hierarchy;
				findContours(~grayRoiImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

				//===================Select only the battery area======================//
				int largestWidth = 0;
				int largestcontour = 0;
				for (int i = 0; i < contours.size(); i++)
				{
					if (contours[i].size() > 15) {
						cv::Rect boundary = cv::boundingRect(contours[i]);
						if (boundary.width > largestWidth)
						{
							largestWidth = boundary.width;
							largestcontour = i;
						}
					}
				}

				if (contours.size() == 0)
				{
					outputROI = cv::Rect(0, 0, 0, 0);
					return outputROI;
				}
				else {
					extLeft = *std::min_element(contours[largestcontour].begin(), contours[largestcontour].end(),
						[](const cv::Point& lhs, const cv::Point& rhs) {
							return lhs.x < rhs.x;
						});
					cv::Point extRight = *std::max_element(contours[largestcontour].begin(), contours[largestcontour].end(),
						[](const cv::Point& lhs, const cv::Point& rhs) {
							return lhs.x < rhs.x;
						});
					cv::Point extTop = *std::min_element(contours[largestcontour].begin(), contours[largestcontour].end(),
						[](const cv::Point& lhs, const cv::Point& rhs) {
							return lhs.y < rhs.y;
						});
					outputROI = cv::Rect(ROI.x + extLeft.x, ROI.y + extTop.y, extRight.x - extLeft.x, grayRoiImg.rows);

					if (display)
						cout << "extleft.x = " << extLeft.x << " extRight.x = " << extRight.x << " extTop.y = " << extTop.y << endl;

				}
			}

			outputROI.x *= RESIZE_SCALE; 
			outputROI.y *= RESIZE_SCALE;
			outputROI.width *= RESIZE_SCALE;
			outputROI.height *= RESIZE_SCALE;

			if (display)
			{
				cv::Mat drawingContour = cv::Mat::zeros(grayRoiImg.size(), CV_8UC3);
				//cv::cvtColor(image(ROI), drawingContour, cv::COLOR_GRAY2BGR);//Reporting
				for (size_t cIdx = 0; cIdx < contours.size(); cIdx++) {
					std::cout << "contour[" << cIdx << "].size = " << contours[cIdx].size() << std::endl;
					if (contours[cIdx].size() > 10) {
						cv::Rect boundary = cv::boundingRect(contours[largestcontour]);
						cv::rectangle(drawingContour, boundary, cv::Scalar(0, 0, 255), 4);
						cv::Point2f vertices[4];
						//minRect.points(vertices);
						for (int i = 0; i < 4; i++)
							cv::line(drawingContour, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 4);
					}
				}

				namedWindow("Binary ROI1 battery4", cv::WINDOW_NORMAL); cv::imshow("Binary ROI1 battery4", grayRoiImg);
				namedWindow("ROI1 battery contour4", cv::WINDOW_NORMAL); cv::imshow("ROI1 battery contour4", drawingContour);

				cout << "contour size = " << contours.size() << " largestcontour = " << largestcontour << endl;
				cout << "outROI.x = " << outputROI.x  << endl;
				cout << "outROI.y = " << outputROI.y  << endl;
				cout << "outROI.width = " << outputROI.width << endl;
				cout << "outROI.height = " << outputROI.height << endl;
				cout << "ROI.x = " << Roi.x << endl;
				cout << "ROI.y = " << Roi.y << endl;
				cout << "ROI.width = " << Roi.width << endl;
				cout << "ROI.height = " << Roi.height << endl;
			}
			if (0){
				std::string fileNameStr = "Result";
				std::string wStr = "";

				wStr += "contour size = " + std::to_string(contours.size()) + "\n ";
				wStr += "largestcontour = " + std::to_string(largestcontour) + "\n ";

				wStr += "outputROI.x = " + std::to_string(outputROI.x) + "\n ";
				wStr += "outputROI.y = " + std::to_string(outputROI.y) + "\n ";
				wStr += "outputROI.width = " + std::to_string(outputROI.width) + "\n ";
				wStr += "outputROI.height = " + std::to_string(outputROI.height) + "\n ";
				wStr += "okFlag = " + std::to_string(okFlag) + "\n ";
				wStr += "extLeftX = " + std::to_string(extLeft.x) + "\n ";

				std::ofstream fout;
				std::string fileName = "D:\\RESULT\\" + fileNameStr + "_" + "tmp.txt";

				fout.open(fileName, std::ios_base::out);

				fout << wStr << std::endl;
				fout.close();

				CString strOpen;
				CString sFile(fileName.c_str());
				strOpen = _T("notepad.exe ") + sFile;
				WinExec(CT2CA(strOpen), SW_NORMAL);  // 프로그램 실행.
			}

			return outputROI;
		}

		std::vector<float> CImageProcessorBase::FindMasterDistance(cv::Mat srcImg) {
			cv::Mat Hcumulative;
			std::vector<float> vt_fStep;
			cv::reduce(srcImg, Hcumulative, 1, cv::REDUCE_AVG, CV_32FC1);
			int ImgHeight = Hcumulative.rows;
			int hIdx = 0;
			//Step 1
			while (Hcumulative.at<float>(hIdx) == 0) {
				hIdx++;
				if (hIdx >= ImgHeight) {
					return vt_fStep;
				}
			}
			float beginStep = hIdx;
			while (Hcumulative.at<float>(hIdx) > 0 || Hcumulative.at<float>(hIdx + 1) > 0) {
				hIdx++;
				if (hIdx + 1 >= ImgHeight) {
					return vt_fStep;
				}
			}
			float endStep = hIdx;
			float step1 = (endStep + beginStep) / 2;
			//Step 2
			while (Hcumulative.at<float>(hIdx) == 0) {
				hIdx++;
				if (hIdx >= ImgHeight) {
					return vt_fStep;
				}
			}
			beginStep = hIdx;
			while (Hcumulative.at<float>(hIdx) > 0 || Hcumulative.at<float>(hIdx + 1) > 0) {
				hIdx++;
				if (hIdx + 1 >= ImgHeight) {
					return vt_fStep;
				}
			}
			endStep = hIdx;
			float step2 = (endStep + beginStep) / 2;
			//Step 3
			while (Hcumulative.at<float>(hIdx) == 0) {
				hIdx++;
				if (hIdx >= ImgHeight) {
					return vt_fStep;
				}
			}
			beginStep = hIdx;
			while (Hcumulative.at<float>(hIdx) > 0 || Hcumulative.at<float>(hIdx + 1) > 0) {
				hIdx++;
				if (hIdx + 1 >= ImgHeight) {
					return vt_fStep;
				}
			}
			endStep = hIdx;
			float step3 = (endStep + beginStep) / 2;

			vt_fStep.push_back(step1);
			vt_fStep.push_back(step2);
			vt_fStep.push_back(step3);
			return vt_fStep;
		}

		void CImageProcessorBase::GammaCorrection(cv::Mat& Img, int gamma)  // global Preprocessing table을 참조, gamma는 실제는 나누기 10값임
		{
			if (gamma == 10) return;

			int rowNum = Img.rows;
			int colNum = Img.cols;
			for (int y = 0; y < rowNum; y++) 
			{
				for (int x = 0; x < colNum; x++)
				{
					Img.at<BYTE>(y, x) = 255 - DC_Prep.gammaLUT[gamma][Img.at<BYTE>(y, x)];
				}
			}
		}

		void CImageProcessorBase::GainCorrection(int* ImgArray, cv::Mat& gammaImg8bit, int gamma, int rowNum, int colNum, cv::Rect outputROI, int whiteCalLine, int option, int addionalHeight)  // global Preprocessing table을 참조
		{
			float* projY = new float[rowNum];
			float* projY1 = new float[rowNum];
			float* calibArray = new float[rowNum * colNum];
			float invWhiteCalLine = 1.0 / whiteCalLine;
			int tmpPosition = 0;
			for (int y = 0; y < rowNum; y++) // 좌측 라인
			{
				projY[y] = 0;
				for (int x = 10; x < whiteCalLine+10; x++)
				{
					projY[y] += ImgArray[tmpPosition + x];  // Img1.at<UINT16>(y, x);
				}
				tmpPosition += colNum;
				projY[y] *= invWhiteCalLine;
			}

			tmpPosition = 0;
			for (int y = 0; y < rowNum; y++)  // 우측 라인
			{
				projY1[y] = 0;
				for (int x = colNum - whiteCalLine - 10; x < colNum - 10; x++)
				{
					projY1[y] += ImgArray[tmpPosition + x];  // Img1.at<UINT16>(y, x);
				}
				tmpPosition += colNum;
				projY1[y] *= invWhiteCalLine;
			}
			for (int y = 0; y < rowNum; y ++)   // 큰값을 선택
			{
				if (projY[y] < projY1[y]) projY[y] = projY1[y];
			}

			float maxVal = -1000;

			int xStart = outputROI.x - 1;
			if (xStart < 0) xStart = 0;
			int yStart = outputROI.y - 1;
			if (yStart < 0) yStart = 0;
			int xEnd = xStart + outputROI.width + 2;
			if (xEnd > colNum) xEnd = colNum;
			int yEnd = yStart + outputROI.height + 2 + addionalHeight;
			if (yEnd > rowNum) yEnd = rowNum;

			cv::Mat invCalibImg8bit;
			float tmpGain;

			//////////////////////////////// Log transform /////////////////////////

			tmpPosition = yStart * rowNum;
			for (int y = yStart; y < yEnd; y++)
			{
				for (int x = xStart; x < xEnd; x++)
				{
					float calValue = 0;
					calValue = DC_Prep.logTable[(int)(projY[y])] - DC_Prep.logTable[(int)ImgArray[tmpPosition + x]];

					if (calValue < 0) calValue = 0;
					if (maxVal < calValue) maxVal = calValue;
					calibArray[tmpPosition + x] = calValue;
				}
				tmpPosition += colNum;
			}
		
			tmpGain = 255.0 / maxVal;
			if (option == 0)
			{
				tmpPosition = yStart * rowNum;
				for (int y = yStart; y < yEnd; y++)     // minValue는 항상 0
				{
					BYTE* data = gammaImg8bit.ptr<BYTE>(y);
					for (int x = xStart; x < xEnd; x++)
					{
						int tValue = 255.0 / maxVal * calibArray[tmpPosition + x];
						if (tValue > 255)    data[x] = 0; // gammaImg8bit.at<BYTE>(y, x) = 0;
						else   			     data[x] = 255 - (BYTE)tValue;
					}
					tmpPosition += colNum;
				}
			}
			else if (option == 1)
			{
				tmpPosition = yStart * rowNum;
				for (int y = yStart; y < yEnd; y++)     // minValue는 항상 0
				{
					BYTE* data = gammaImg8bit.ptr<BYTE>(y);
					for (int x = xStart; x < xEnd; x++)
					{
						int tValue = 255.0 / maxVal * calibArray[tmpPosition + x];
						if (tValue > 255)    tValue = 255;
						else if (tValue < 0) tValue = 0;
						data[x] = DC_Prep.gammaLUT[gamma][tValue];
					}
					tmpPosition += colNum;
				}
			}
			delete projY;
			delete projY1;
			delete calibArray;
		}

		void CImageProcessorBase::GainCorrection2(int* ImgArray, int rowNum, int colNum, cv::Rect outputROI, int whiteCalLine)  // global Preprocessing table을 참조
		{
			float* projY = new float[rowNum];
			float* projY1 = new float[rowNum];
			float* calibArray = new float[rowNum * colNum];
			float invWhiteCalLine = 1.0 / whiteCalLine;
			int tmpPosition = 0;
			for (int y = 0; y < rowNum; y++) // 좌측 라인
			{
				projY[y] = 0;
				for (int x = 0; x < whiteCalLine; x++)
				{
					projY[y] += ImgArray[tmpPosition + x];  // Img1.at<UINT16>(y, x);
				}
				tmpPosition += colNum;
				projY[y] *= invWhiteCalLine;
			}

			tmpPosition = 0;
			for (int y = 0; y < rowNum; y++)  // 우측 라인
			{
				projY1[y] = 0;
				for (int x = colNum - whiteCalLine; x < colNum; x++)
				{
					projY1[y] += ImgArray[tmpPosition + x];  // Img1.at<UINT16>(y, x);
				}
				tmpPosition += colNum;
				projY1[y] *= invWhiteCalLine;
			}
			for (int y = 0; y < rowNum; y++)   // 큰값을 선택
			{
				if (projY[y] < projY1[y]) projY[y] = projY1[y];
			}

			float maxVal = -10000;
			int xStart = outputROI.x;
			if (xStart < 0) xStart = 0;
			int yStart = outputROI.y;
			if (yStart < 0) yStart = 0;
			int xEnd = xStart + outputROI.width;
			if (xEnd > colNum) xEnd = colNum;
			int yEnd = yStart + outputROI.height;
			if (yEnd > rowNum) yEnd = rowNum;

			float tmpGain;

			//////////////////////////////// Log transform /////////////////////////

			tmpPosition = yStart * colNum;
			for (int y = yStart; y < yEnd; y++)
			{
				for (int x = xStart; x < xEnd; x++)
				{
					float calValue = 0;
					calValue = DC_Prep.logTable[(int)(projY[y])] - DC_Prep.logTable[(int)ImgArray[tmpPosition + x]];

					if (calValue < 0) calValue = 0;
					if (maxVal < calValue) maxVal = calValue;
					calibArray[tmpPosition + x] = calValue;
				}
				tmpPosition += colNum;
			}

			memset(ImgArray, 65280, sizeof(int)*(rowNum * colNum));

			tmpGain = 255.0 / maxVal;

			tmpPosition = yStart * colNum;
			for (int y = yStart; y < yEnd; y++)     // minValue는 항상 0
			{
				for (int x = xStart; x < xEnd; x++)
				{
					int tValue = tmpGain * calibArray[tmpPosition + x];
					if (tValue > 255)    tValue = 255;
					else if (tValue < 0) tValue = 0;
					ImgArray[tmpPosition + x] = DC_Prep.gammaLUT2[tValue];
				}
				tmpPosition += colNum;
			}
			delete projY;
			delete projY1;
			delete calibArray;
		}

		int CImageProcessorBase::MedianValueFrom3Value(int* value)
		{
			int max, min, sum;
			max = min = sum = value[0];

			for (int i = 1; i < 3; i++)
			{
				if (max < value[i]) max = value[i];
				if (min > value[i]) min = value[i];
				sum += value[i];
			}
			return(sum - max - min);
		}

		int CImageProcessorBase::MechaCalibrationDataLoad(double HeadCalibValue[], double RefLoc[], double CellWidth[], std::string fileName)
		{
			std::ifstream fin;

			fin.open(fileName);
			if (!fin)
			{
//				AfxMessageBox(fileName.c_str(), MB_SYSTEMMODAL);
				return 0;
			}

			std::string lineBuf, rStr, c;
			int lineNum = 0;

			double tmpValueD[5], pixelSize, avgRefLoc, avgCellWidth;
			int tmpValueI[5];

			rStr = "";

			getline(fin, lineBuf);
			if ((lineBuf.length() < 15) || (lineBuf.length() > 100)) return 0;

			int k = 0;
			for (int i = 0; i < 3; i++) {
				rStr = "";
				while ((c = lineBuf[k++]) != ",")
					rStr += c;

				tmpValueD[i] = stof(rStr);
			}

			pixelSize = tmpValueD[0]; // no use
			avgRefLoc = tmpValueD[1]; // no use
			avgCellWidth = tmpValueD[2]; // no use

			avgCellWidth = tmpValueD[2];
			cout << "pixelSize :" << tmpValueD[0] << ", avgRefLoc: " << tmpValueD[1] << ", avgCellWidth: " << tmpValueD[2] << endl;

			while (getline(fin, lineBuf))
			{
				int k = 0;

				if (lineBuf.length() < 5) break;
				for (int i = 0; i < 4; i++) {
					rStr = "";
					while ((c = lineBuf[k++]) != ",")
						rStr += c;

					tmpValueD[i] = stof(rStr);
					tmpValueI[i] = stoi(rStr);
				}

				if (tmpValueI[0] > MAX_HEAD_NUM)
				{
					cout << "data file err! HeadNum is bigger than MAX_HEAD_NUM" << endl;
					return 0;
				}
				cout << "HD" << tmpValueI[0] << ", " << tmpValueD[1] << ", " << tmpValueD[2] << ", " << tmpValueD[3] << endl;

				RefLoc[tmpValueI[0]] = tmpValueD[1];
				CellWidth[tmpValueI[0]] = tmpValueD[2];
				HeadCalibValue[tmpValueI[0]] = tmpValueD[3];

				lineNum++;
			}
			RefLoc[0] = CellWidth[0] = HeadCalibValue[0] = 1.0;

			fin.close();
			return 1;
		}

		cv::Rect CImageProcessorBase::FindCTCellOutsideRoi(cv::Mat image, cv::Rect Roi, int displayMode)
		{
			cv::Mat grayImg;
			//===================Find contour of the battery======================//
			cv::Rect Rroi;
			Rroi = CImageProcessorBase::RoiRefinement(Roi, image.size());

			cv::Mat dst = cv::Mat(image.size(), CV_8U);
			dst = 0;
			image(Rroi).copyTo(dst(Rroi));
			cv::medianBlur(dst, dst, 15);

			double otsuThresh = cv::threshold(dst, grayImg, 0, 255, cv::THRESH_OTSU);
			Rroi = CImageProcessorBase::RoiRefinement(Rroi, image.size());

			vector<int> vtLeftX, vtRightX, vtTopY, vtBottomY;
			for (int y = 0; y < image.rows; y++)    vtLeftX.push_back(image.cols);
			for (int y = image.rows - 1; y >= 0; y--) vtRightX.push_back(0);
			for (int x = 0; x < image.cols; x++)    vtTopY.push_back(image.rows);
			for (int x = image.cols - 1; x >= 0; x--) vtBottomY.push_back(0);

			for (int y = 1; y < image.rows; y++)
			{
				for (int x = 1; x < image.cols; x++)
				{
					if ((grayImg.at<BYTE>(y, x - 1) == 0) && (grayImg.at<BYTE>(y, x) == 255))
					{
						vtLeftX[y] = x;
						break;
					}
				}
				for (int x = image.cols - 2; x >= 0; x--)
				{
					if ((grayImg.at<BYTE>(y, x + 1) == 0) && (grayImg.at<BYTE>(y, x) == 255))
					{
						vtRightX[y] = x;
						break;
					}
				}
			}
			for (int x = 1; x < image.cols; x++)
			{
				for (int y = 1; y < image.rows; y++)
				{
					if ((grayImg.at<BYTE>(y - 1, x) == 0) && (grayImg.at<BYTE>(y, x) == 255))
					{
						vtTopY[x] = y;
						break;
					}
				}
			}
			for (int x = 1; x < image.cols*0.3; x++)
			{
				for (int y = image.rows - 2; y >= 0; y--)
				{
					if ((grayImg.at<BYTE>(y + 1, x) == 0) && (grayImg.at<BYTE>(y, x) == 255))
					{
						vtBottomY[x] = y;
						break;
					}
				}
			}
			for (int x = image.cols*0.7; x < image.cols; x++)
			{
				for (int y = image.rows - 2; y >= 0; y--)
				{
					if ((grayImg.at<BYTE>(y + 1, x) == 0) && (grayImg.at<BYTE>(y, x) == 255))
					{
						vtBottomY[x] = y;
						break;
					}
				}
			}

			int leftX = *std::min_element(vtLeftX.begin(), vtLeftX.end());
			int rightX = *std::max_element(vtRightX.begin(), vtRightX.end());
			int topY = *std::min_element(vtTopY.begin(), vtTopY.end());
			int bottomY = *std::max_element(vtBottomY.begin(), vtBottomY.end());

			if (0) {  		// bottom check with another method
				vector<int> vtBottomX2, vtBottomY2;
				for (int y = Roi.y + Roi.height / 2; y < Roi.y + Roi.height; y++)
				{
					for (int x = Roi.x; x < image.cols * 0.3; x++)
					{
						if ((grayImg.at<BYTE>(y, x) == 0) && (grayImg.at<BYTE>(y, x + 1) == 255))
						{
							vtBottomX2.push_back(x);
							vtBottomY2.push_back(y);
							break;
						}
					}
				}
				for (int i = 0; i < vtBottomX2.size(); i++)
				{
					if (vtBottomX2[i + 2] - vtBottomX2[i] > 2) bottomY = vtBottomY2[i];
				}
			}

			cv::Rect outputROI;
			if ((rightX <= leftX) || (bottomY <= topY)) outputROI = cv::Rect(0, 0, 0, 0);
			else outputROI = cv::Rect(leftX, topY, rightX - leftX, bottomY - topY);

			if (displayMode > 0) {
				std::cout << "otsuThresh: " << otsuThresh << ", outputROI x, y, width, height = " << outputROI.x << ", " << outputROI.y << ", " << outputROI.width << ", " << outputROI.height << std::endl;

				cv::Mat drawingContour = cv::Mat::zeros(grayImg.size(), CV_8UC3);;
				cv::rectangle(drawingContour, outputROI, cv::Scalar(0, 0, 255), 3);

				namedWindow("Cell outROI", cv::WINDOW_NORMAL);
				cv::imshow("Cell outROI", drawingContour);
			}

			return outputROI;
		}

		namespace ed {
			/**
			 * @brief direction of edge
			 * @brief if |Gx|>|Gy|, it is a proposal of vertical edge(EDGE_VER), or it is horizontal edge(EDGE_HOR)
			 */
			enum EDGE_DIR
			{
				EDGE_HOR,
				EDGE_VER
			};

			/**
			 * @brief: a sign for recording status of each pixel in tracing
			 */
			enum STATUS
			{
				STATUS_UNKNOWN = 0,
				STATUS_BACKGROUND = 1,
				STATUS_EDGE = 255
			};

			/**
			 * @brief: Trace direction
			 */
			enum TRACE_DIR
			{
				TRACE_LEFT,
				TRACE_RIGHT,
				TRACE_UP,
				TRACE_DOWN
			};

			/**
			 * @brief calculate gradient magnitude and orientation
			 * @param gray [in] input grayscale image
			 * @param M [out] gradient magnitude, actually |Gx|+|Gy|
			 * @param O [out] gradient orientation, refer to the definition of EDGE_DIR
			 */
			void getGradient(const cv::Mat& gray,
				cv::Mat& M,
				cv::Mat& O) {
				cv::Mat Gx, Gy;
				cv::Sobel(gray, Gx, CV_16SC1, SOBEL_ORDER, 0, SOBEL_SIZE);
				cv::Sobel(gray, Gy, CV_16SC1, 0, SOBEL_ORDER, SOBEL_SIZE);

				M.create(gray.rows, gray.cols, CV_16SC1);
				O.create(gray.rows, gray.cols, CV_8UC1);

				for (int r = 0; r < gray.rows; ++r) {
					for (int c = 0; c < gray.cols; ++c) {
						auto dx = std::abs(Gx.at<short>(r, c));
						auto dy = std::abs(Gy.at<short>(r, c));
						M.at<short>(r, c) = dx + dy;
						O.at<uchar>(r, c) = (dx > dy ? EDGE_VER : EDGE_HOR);
					}
				}
			}

			/**
			 * @brief get anchors
			 * @param M [in] gradient magnitude
			 * @param O [in] gradient orientation
			 * @param proposal_thresh [in] minimum gradient magnitude to be an edge point
			 * @param anchor_interval [in] interval to search anchors
			 * @param anchor_thresh [in] minimum gradient diff of anchors
			 * @param anchors [out] anchors
			 */
			void getAnchors(const cv::Mat& M,
				const cv::Mat& O,
				const int proposal_thresh,
				const int anchor_interval,
				const int anchor_thresh,
				std::vector<cv::Point>& anchors) {
				anchors.clear();
				anchors.reserve(M.cols * M.rows);

				for (int r = 1; r < M.rows - 1; r += anchor_interval) {
					for (int c = 1; c < M.cols - 1; c += anchor_interval) {
						// ignore non-proposal pixels
						if (M.at<short>(r, c) < proposal_thresh)
							continue;

						// horizontal edge
						if (O.at<uchar>(r, c) == EDGE_HOR) {
							if (M.at<short>(r, c) - M.at<short>(r - 1, c) >= anchor_thresh &&
								M.at<short>(r, c) - M.at<short>(r + 1, c) >= anchor_thresh)
								anchors.emplace_back(c, r);
						}

						// vertical edge
						else {
							if (M.at<short>(r, c) - M.at<short>(r, c - 1) >= anchor_thresh &&
								M.at<short>(r, c) - M.at<short>(r, c + 1) >= anchor_thresh)
								anchors.emplace_back(c, r);
						}
					}
				}
			}

			void getAnchors2(const cv::Mat& M,
				const cv::Mat& O,
				const int proposal_thresh,
				const int anchor_interval,
				const int anchor_thresh,
				std::vector<cv::Point>& anchors) {
				anchors.clear();
				anchors.reserve(M.cols * M.rows);

				for (int r = anchor_interval / 2; r < M.rows - 1; r += anchor_interval) {
					for (int c = anchor_interval / 2; c < M.cols - 1; c += anchor_interval) {
						// ignore non-proposal pixels

						/*int maxRows = M.rows > (r + anchor_interval) ? M.rows : (r + anchor_interval);
						int maxCols = M.cols > (c + anchor_interval) ? M.cols : (c + anchor_interval);*/

						int maxRows = M.rows > (r + anchor_interval) ? anchor_interval : (r + anchor_interval) - M.rows + 1;
						int maxCols = M.cols > (c + anchor_interval) ? anchor_interval : (c + anchor_interval) - M.cols + 1;
						try {
							cv::Mat roi;
							if (O.at<uchar>(r, c) == EDGE_HOR) {
								roi = M.colRange(c, c + 1).rowRange(r - anchor_interval / 2, r + anchor_interval / 2);
							}
							else {
								roi = M.colRange(c - anchor_interval / 2, c + anchor_interval / 2).rowRange(r, r + 1);
							}
							//cv::Mat roi = M.colRange(c, c + maxCols).rowRange(r, r + maxRows);
							double min = 0.0, max = 0.0;
							cv::Point minLoc, maxLoc;
							cv::minMaxLoc(roi, &min, &max, &minLoc, &maxLoc);
							if ((max < proposal_thresh) || (max != M.at<short>(r, c)))
								continue;

							//std::cout << "Roi: \n" << roi << std::endl;
							//anchors.emplace_back(c+maxLoc.x, r+maxLoc.y);
							anchors.emplace_back(c, r);
						}
						catch (...) {
							std::cout << "errors";
						}

						/*if (M.at<short>(r, c) < proposal_thresh)
							continue;*/

							// horizontal edge
							//if (O.at<uchar>(r, c) == EDGE_HOR) {
							//    if (M.at<short>(r, c) - M.at<short>(r - 1, c) >= anchor_thresh &&
							//        M.at<short>(r, c) - M.at<short>(r + 1, c) >= anchor_thresh)
							//        anchors.emplace_back(c, r);
							//}

							//// vertical edge
							//else {
							//    if (M.at<short>(r, c) - M.at<short>(r, c - 1) >= anchor_thresh &&
							//        M.at<short>(r, c) - M.at<short>(r, c + 1) >= anchor_thresh)
							//        anchors.emplace_back(c, r);
							//}
					}
				}
			}


			void getAnchors3(const cv::Mat& M,
				const cv::Mat& O,
				const int proposal_thresh,
				const int anchor_interval,
				const int anchor_thresh,
				std::vector<cv::Point>& anchors) {
				anchors.clear();
				anchors.reserve(M.cols * M.rows);

				for (int r = 1; r < M.rows - 1; r += 1) {
					for (int c = 1; c < M.cols - 1; c += anchor_interval) {
						// ignore non-proposal pixels
						if (M.at<short>(r, c) < proposal_thresh)
							continue;

						// horizontal edge
						if (O.at<uchar>(r, c) == EDGE_HOR) {
							if (M.at<short>(r, c) - M.at<short>(r - 1, c) >= anchor_thresh &&
								M.at<short>(r, c) - M.at<short>(r + 1, c) >= anchor_thresh)
								anchors.emplace_back(c, r);
						}

						// vertical edge
						else {
							if (M.at<short>(r, c) - M.at<short>(r, c - 1) >= anchor_thresh &&
								M.at<short>(r, c) - M.at<short>(r, c + 1) >= anchor_thresh)
								anchors.emplace_back(c, r);
						}
					}
				}
			}
			/**
			 * @brief main loop of tracing edge
			 * @param M [in] gradient magnitude
			 * @param O [in] gradient orientation
			 * @param pt_last [in] last visited point
			 * @param pt_cur [in] current point to be evaluated
			 * @param dir_last [in] last trace direction
			 * @param push_back [in] push the traced point to the back or front of list
			 * @param status [in|out] status record
			 * @param edge [out] traced edge
			 */
			void trace(const cv::Mat& M,
				const cv::Mat& O,
				const int proposal_thresh,
				cv::Point pt_last,
				cv::Point pt_cur,
				TRACE_DIR dir_last,
				bool push_back,
				cv::Mat& status,
				std::list<cv::Point>& edge) {
				// current direction
				TRACE_DIR dir_cur;

				// repeat until reaches the visited pixel or non-proposal
				while (true) {
					// terminate trace if that point has already been visited
					if (status.at<uchar>(pt_cur.y, pt_cur.x) != STATUS_UNKNOWN)
						break;

					// set it to background and terminate trace if that point is not a proposal edge
					if (M.at<short>(pt_cur.y, pt_cur.x) < proposal_thresh) {
						status.at<uchar>(pt_cur.y, pt_cur.x) = STATUS_BACKGROUND;
						break;
					}

					// set point pt_cur as edge
					status.at<uchar>(pt_cur.y, pt_cur.x) = STATUS_EDGE;
					if (push_back)
						edge.push_back(pt_cur);
					else
						edge.push_front(pt_cur);

					// if its direction is EDGE_HOR, trace left or right
					if (O.at<uchar>(pt_cur.y, pt_cur.x) == EDGE_HOR) {
						// calculate trace direction
						if (dir_last == TRACE_UP || dir_last == TRACE_DOWN) {
							if (pt_cur.x < pt_last.x)
								dir_cur = TRACE_LEFT;
							else
								dir_cur = TRACE_RIGHT;
						}
						else
							dir_cur = dir_last;

						// update last state
						pt_last = pt_cur;
						dir_last = dir_cur;

						// go left
						if (dir_cur == TRACE_LEFT) {
							auto leftTop = M.at<short>(pt_cur.y - 1, pt_cur.x - 1);
							auto left = M.at<short>(pt_cur.y, pt_cur.x - 1);
							auto leftBottom = M.at<short>(pt_cur.y + 1, pt_cur.x - 1);

							if (leftTop >= left && leftTop >= leftBottom)
								pt_cur = cv::Point(pt_cur.x - 1, pt_cur.y - 1);
							else if (leftBottom >= left && leftBottom >= leftTop)
								pt_cur = cv::Point(pt_cur.x - 1, pt_cur.y + 1);
							else
								pt_cur.x -= 1;

							// break if reaches the border of image, the same below
							if (pt_cur.x == 0 || pt_cur.y == 0 || pt_cur.y == M.rows - 1)
								break;
						}

						// go right
						else {
							auto rightTop = M.at<short>(pt_cur.y - 1, pt_cur.x + 1);
							auto right = M.at<short>(pt_cur.y, pt_cur.x + 1);
							auto rightBottom = M.at<short>(pt_cur.y + 1, pt_cur.x + 1);

							if (rightTop >= right && rightTop >= rightBottom)
								pt_cur = cv::Point(pt_cur.x + 1, pt_cur.y - 1);
							else if (rightBottom >= right && rightBottom >= rightTop)
								pt_cur = cv::Point(pt_cur.x + 1, pt_cur.y + 1);
							else
								pt_cur.x += 1;

							if (pt_cur.x == M.cols - 1 || pt_cur.y == 0 || pt_cur.y == M.rows - 1)
								break;
						}
					}

					// its direction is EDGE_VER, trace up or down
					else {
						// calculate trace direction
						if (dir_last == TRACE_LEFT || dir_last == TRACE_RIGHT)
						{
							if (pt_cur.y < pt_last.y)
								dir_cur = TRACE_UP;
							else
								dir_cur = TRACE_DOWN;
						}
						else
							dir_cur = dir_last;

						// update last state
						pt_last = pt_cur;
						dir_last = dir_cur;

						// go up
						if (dir_cur == TRACE_UP) {
							auto leftTop = M.at<short>(pt_cur.y - 1, pt_cur.x - 1);
							auto top = M.at<short>(pt_cur.y - 1, pt_cur.x);
							auto rightTop = M.at<short>(pt_cur.y - 1, pt_cur.x + 1);

							if (leftTop >= top && leftTop >= rightTop)
								pt_cur = cv::Point(pt_cur.x - 1, pt_cur.y - 1);
							else if (rightTop >= top && rightTop >= leftTop)
								pt_cur = cv::Point(pt_cur.x + 1, pt_cur.y - 1);
							else
								pt_cur.y -= 1;

							if (pt_cur.y == 0 || pt_cur.x == 0 || pt_cur.x == M.cols - 1)
								break;
						}

						// go down
						else {
							auto leftBottom = M.at<short>(pt_cur.y + 1, pt_cur.x - 1);
							auto bottom = M.at<short>(pt_cur.y + 1, pt_cur.x);
							auto rightBottom = M.at<short>(pt_cur.y + 1, pt_cur.x + 1);

							if (leftBottom >= bottom && leftBottom >= rightBottom)
								pt_cur = cv::Point(pt_cur.x - 1, pt_cur.y + 1);
							else if (rightBottom >= bottom && rightBottom >= leftBottom)
								pt_cur = cv::Point(pt_cur.x + 1, pt_cur.y + 1);
							else
								pt_cur.y += 1;

							if (pt_cur.y == M.rows - 1 || pt_cur.x == 0 || pt_cur.x == M.cols - 1)
								break;
						}
					}
				}
			}

			/**
			 * @brief trace edge from an anchor
			 * @param M [in] gradient magnitude
			 * @param O [in] gradient orientation
			 * @param anchor [in] anchor point to be traced from
			 * @param status [in|out] status record of each pixel, see the definition of STATUS
			 * @param edges [out] traced edge would be push_back to
			 */
			void traceFromAnchor(const cv::Mat& M,
				const cv::Mat& O,
				const int proposal_thresh,
				const cv::Point& anchor,
				cv::Mat& status,
				std::vector<std::list<cv::Point>>& edges) {
				// if this anchor point has already been visited
				if (status.at<uchar>(anchor.y, anchor.x) != STATUS_UNKNOWN)
					return;

				std::list<cv::Point> edge;
				cv::Point pt_last;
				TRACE_DIR dir_last;

				// if horizontal edge, go left and right
				if (O.at<uchar>(anchor.y, anchor.x) == EDGE_HOR) {
					// go left first
					// sssume the last visited point is the right hand side point and TRACE_LEFT to current point, the same below
					pt_last = cv::Point(anchor.x + 1, anchor.y);
					dir_last = TRACE_LEFT;
					trace(M, O, proposal_thresh, pt_last, anchor, dir_last, false, status, edge);

					// reset anchor point
					// it has already been set in the previous traceEdge(), reset it to satisfy the initial while condition, the same below
					status.at<uchar>(anchor.y, anchor.x) = STATUS_UNKNOWN;

					// go right then
					pt_last = cv::Point(anchor.x - 1, anchor.y);
					dir_last = TRACE_RIGHT;
					trace(M, O, proposal_thresh, pt_last, anchor, dir_last, true, status, edge);
				}

				// vertical edge, go up and down
				else {
					// go up first
					pt_last = cv::Point(anchor.x, anchor.y + 1);
					dir_last = TRACE_UP;
					trace(M, O, proposal_thresh, pt_last, anchor, dir_last, false, status, edge);

					// reset anchor point
					status.at<uchar>(anchor.y, anchor.x) = STATUS_UNKNOWN;

					// go down then
					pt_last = cv::Point(anchor.x, anchor.y - 1);
					dir_last = TRACE_DOWN;
					trace(M, O, proposal_thresh, pt_last, anchor, dir_last, true, status, edge);
				}

				edges.push_back(edge);
			}

			std::vector<std::list<cv::Point>> detectEdges(const cv::Mat& image,
				const int proposal_thresh,
				const int anchor_interval,
				const int anchor_thresh) {
				// 0.preparation
				cv::Mat gray;
				if (image.empty()) {
					std::cout << "Empty image input!" << std::endl;
					return std::vector<std::list<cv::Point>>();
				}
				if (image.type() == CV_8UC1)
					gray = image.clone();
				else if (image.type() == CV_8UC3)
					cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
				else {
					std::cout << "Unknow image type!" << std::endl;
					return std::vector<std::list<cv::Point>>();
				}

				// 1.Gauss blur
				cv::GaussianBlur(gray, gray, cv::Size(GAUSS_SIZE, GAUSS_SIZE), GAUSS_SIGMA, GAUSS_SIGMA);
				//cv::GaussianBlur(gray, gray, cv::Size(7, 7), 0.0, 0.0);

				// 2.get gradient magnitude and orientation
				cv::Mat M, O;
				getGradient(gray, M, O);

				// 3.get anchors
				std::vector<cv::Point> anchors;
				getAnchors(M, O, proposal_thresh, anchor_interval, anchor_thresh, anchors);

				// 4.trace edges from anchors
				cv::Mat status(gray.rows, gray.cols, CV_8UC1, cv::Scalar(STATUS_UNKNOWN)); //init status to STATUS_UNKNOWN
				std::vector<std::list<cv::Point>> edges;
				for (const auto& anchor : anchors)
					traceFromAnchor(M, O, proposal_thresh, anchor, status, edges);

				return edges;
			}

		}
	}
}

void SaveInspectionResult(cv::Mat orgImg, cv::Mat resImg, BOOL isUpper, std::string nameStr, int cellID, std::string wStr)
{
	CString resultFolder = _T("D:\\RESULT");
	if (GetFileAttributes(resultFolder) == 0xFFFFFFFF) CreateDirectory(resultFolder, NULL);
	CString saveFolder = _T("D:\\RESULT\\") + (CString)nameStr.c_str();

	CString tmpTime = _T("");
	CTime cTime = CTime::GetCurrentTime();
	tmpTime.AppendFormat(_T("%02d%02d_%02d%02d%02d"), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());

	CString curDate = _T("");
	curDate.AppendFormat(_T("%04d%02d%02d"), cTime.GetYear(), cTime.GetMonth(), cTime.GetDay());

	std::string wrImg = (std::string)CT2CA(saveFolder + _T("\\") + curDate + _T("\\") + tmpTime) + std::to_string(cellID);

	if (isUpper) {
		if (GetFileAttributes(saveFolder) == 0xFFFFFFFF) CreateDirectory(saveFolder, NULL);
		if (GetFileAttributes(saveFolder + _T("\\") + curDate) == 0xFFFFFFFF) CreateDirectory(saveFolder + _T("\\") + curDate, NULL);
		Sleep(1);

		cv::imwrite(wrImg + "_A_" + nameStr + ".BMP", orgImg);
		cv::imwrite(wrImg + "_A_" + nameStr + ".JPG", resImg);
	}
	else {
		if (GetFileAttributes(saveFolder) == 0xFFFFFFFF) CreateDirectory(saveFolder, NULL);
		if (GetFileAttributes(saveFolder + _T("\\") + curDate) == 0xFFFFFFFF) CreateDirectory(saveFolder + _T("\\") + curDate, NULL);
		Sleep(1);
		cv::imwrite(wrImg + "_B_" + nameStr + ".BMP", orgImg);
		cv::imwrite(wrImg + "_B_" + nameStr + ".JPG", resImg);
	}

	std::string csvFileName = (std::string)CT2CA(saveFolder + _T("\\") + curDate + _T("\\") + curDate);
	if (isUpper) csvFileName += "_A_" + nameStr + ".CSV";
	else                csvFileName += "_B_" + nameStr + ".CSV";

	std::ofstream fout;
	fout.open(csvFileName, std::ios_base::out | std::ios_base::app);

	fout << wStr << std::endl;
	fout.close();
}
