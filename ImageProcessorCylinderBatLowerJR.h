#pragma once
#include "ImageProcessorBase.h"
#include "GFuncCylinderBaterryJR.h"
//#include "InspectCylinderBattery.h"

namespace xf
{
	namespace ImageProcessor
	{
		class CImageProcessorCylinderBatLowerJR :
			public CImageProcessorBase
		{
		public:
			CImageProcessorCylinderBatLowerJR();
			~CImageProcessorCylinderBatLowerJR();
			
//			virtual void ImageProcessing(cv::Mat img, xvt::CylinderJRInspection BatLowerInsp, xvt::battery::BatteryInspectionResult &BIresult);
			void ImageProcessing(const cv::Mat img, xvt::CylinderJRInspection BatLowerInsp, BatResultStruct& bInspResult); // xvt::battery::BatteryInspectionResult& BIresult);

			xvt::CylinderJRInspection cylUpperParam;
		protected:
			int Inspection(cv::Mat img, xvt::CylinderJRInspection BatLowerInsp, xvt::battery::BatteryInspectionResult& BIresult);
		};
	}
}