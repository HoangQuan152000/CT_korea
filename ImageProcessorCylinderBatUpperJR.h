#pragma once
#include "ImageProcessorBase.h"
#include "GFuncCylinderBaterryJR.h"
//#include "InspectCylinderBattery.h"

namespace xf
{
	namespace ImageProcessor
	{
		class CImageProcessorCylinderBatUpperJR
			: public CImageProcessorBase
		{
		public:
			CImageProcessorCylinderBatUpperJR();
			~CImageProcessorCylinderBatUpperJR();
//			virtual void ImageProcessing(cv::Mat img, xvt::CylinderJRInspection batInsp, xvt::battery::BatteryInspectionResult& bIresult);
			void ImageProcessing(const cv::Mat img, xvt::CylinderJRInspection batInsp, BatResultStruct& bInspResult); // xvt::battery::BatteryInspectionResult& bIresult);

		protected:
			int Inspection(cv::Mat img, xvt::CylinderJRInspection BatInsp, xvt::battery::BatteryInspectionResult& BIresult);
		};
	}
}