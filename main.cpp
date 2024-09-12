// InspectionCylBatt.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <xvtBattery/CylinderUtils.h>
#include <omp.h>
#include <ctime>
#include <fstream>
#include <numeric>
#include <string>
#include "ImageProcessorBase.h"
#include "GFuncCylinderBaterryJR.h"
#include "ImageProcessorCylinderBatUpperJR.h"
#include "ImageProcessorCylinderBatLowerJR.h"

int main()
{
	std::cout << "Inspection Cyl Battery Start" << std::endl;

	{ // Upper inspection
		xf::ImageProcessor::CImageProcessorCylinderBatUpperJR cylUpperAlgo;
		struct xvt::CylinderJRInspection cylUpperParam;
		struct BatResultStruct cylUpperResult;

		std::string fileName = "1 1 3.tif";
		cv::Mat imgSrc = cv::imread(fileName, cv::IMREAD_ANYDEPTH);  // tiff, 16bit image 
		
		cylUpperParam.UseBatteryInspection = 1;
		cylUpperParam.inspectingItems.ANODE_TO_CASE_GAP = true;
		cylUpperParam.inspectingItems.ANODE_TO_CASE_VARIATION = true;
		cylUpperParam.inspectingItems.CATHODE_TO_CASE_GAP = true;
		cylUpperParam.BatteryDirection = 0;
		cylUpperParam.RoiX = 0;
		cylUpperParam.RoiY = 0;
		cylUpperParam.RoiWidth = 3000;
		cylUpperParam.RoiHeight = 3000;

		cylUpperParam.batteryThreshold = 70;
		cylUpperParam.minBeadingHeight = 50;
		cylUpperParam.D1StartPosition = 0.5;
		cylUpperParam.poleRegionHeight = 300;
		cylUpperParam.PoleLeaningThreshold = 0;

		cylUpperParam.JR_ROIX = -35;
		cylUpperParam.JR_ROIY = -50;
		cylUpperParam.CenterNeglectionWidth = 300;
		cylUpperParam.tolerenceShift = -50;

		cylUpperParam.cathodeLineThredsholdInner = 0.0;
		cylUpperParam.cathodeLineThredsholdMiddle = 0.0;
		cylUpperParam.cathodeLineThredsholdOuter = 0.0;
		cylUpperParam.cathodeLineWindowSize = 5;
		cylUpperParam.polesHeight = 80;
		cylUpperParam.minProminence = 1;

		cylUpperParam.polesMinDistance = 8;
		cylUpperParam.polesMaxDistance = 30;

		cylUpperParam.anodeThresholdInner = 1.2;
		cylUpperParam.anodeThresholdMiddle = 2.2;
		cylUpperParam.anodeThresholdOuter = 2.2;
		cylUpperParam.OneSidePoleNumb = 26;
		cylUpperParam.isCheckPoleNo = 1;
		cylUpperParam.isCheckPin = 0;

		cylUpperParam.skipPolesDistance = 0;
		cylUpperParam.MinCathode2Anode = 0.3;
		cylUpperParam.MaxCathode2Anode = 1.5;

		cylUpperParam.MinAnode2Case = 3.5;
		cylUpperParam.MaxAnode2Case = 6.0;
		cylUpperParam.MinCathode2Case = 5.3;
		cylUpperParam.MaxCathode2Case = 8.0;


		cylUpperParam.mAnode2CaseOffset = 0;
		cylUpperParam.mCathode2AnodeOffset = 0;
		cylUpperParam.mCathode2CaseOffset = 0;

		cylUpperParam.variationAnode2Case = 3;

		cylUpperParam.pixelSize = 0.0105;
		cylUpperParam.gamma = 20;

		cylUpperParam.minLeaningDistance = 0;

		cylUpperParam.displayMode = 1;
		cylUpperParam.isSaveResults = 0;  // > 0 : csv 저장, 2nd bit set : org image and result image save 
		cylUpperParam.resultPath = "D:\\Result1";
		cylUpperParam.cellID = "0";

		if (GetFileAttributes((CString)cylUpperParam.resultPath.c_str()) == 0xFFFFFFFF) CreateDirectory((CString)cylUpperParam.resultPath.c_str(), NULL);
		cylUpperAlgo.ImageProcessing(imgSrc, cylUpperParam, cylUpperResult);
		//std::cout << "Lower csvTitle :" << (std::string)CT2CA(cylUpperResult.csvTitleStr) << std::endl;
		//std::cout << "Lower csvResult :" << (std::string)CT2CA(cylUpperResult.csvResultStr) << std::endl << std::endl;

		//std::cout << "Lower csvTitleXM :" << (std::string)CT2CA(cylUpperResult.csvTitleStrXM4) << std::endl;
		//std::cout << "Lower csvResultXM :" << (std::string)CT2CA(cylUpperResult.csvResultStrXM4) << std::endl << std::endl;

		cv::imwrite("1 1 3.jpeg", cylUpperResult.resImg);
	}

	{ // Lower inspection
		xf::ImageProcessor::CImageProcessorCylinderBatLowerJR cylLowerAlgo;
		struct xvt::CylinderJRInspection cylLowerParam;
		struct BatResultStruct cylLowerResult;

		std::string fileName = "1 2 3.tif";
		cv::Mat imgSrc = cv::imread(fileName, cv::IMREAD_ANYDEPTH);  // tiff, 16bit image 
		
		cylLowerParam.UseBatteryInspection = 1;
		cylLowerParam.inspectingItems.ANODE_TO_CASE_GAP = true;
		cylLowerParam.inspectingItems.ANODE_TO_CASE_VARIATION = false;
		cylLowerParam.inspectingItems.CATHODE_TO_CASE_GAP = true;
		cylLowerParam.BatteryDirection = 1;

		//cylLowerParam.InspItem = 0;
		cylLowerParam.RoiX = 0;
		cylLowerParam.RoiY = 0;
		cylLowerParam.RoiWidth = 3000;
		cylLowerParam.RoiHeight = 1200;
		cylLowerParam.D1StartPosition = 0;

		cylLowerParam.batteryThreshold = 50;
		cylLowerParam.poleRegionHeight = 300;
		cylLowerParam.PoleLeaningThreshold = 0;

		cylLowerParam.JR_ROIX = -30;
		cylLowerParam.JR_ROIY = -50;
		cylLowerParam.CenterNeglectionWidth = 300;
		cylLowerParam.skipPolesDistance = 0;
		cylLowerParam.tolerenceShift = -50;

		cylLowerParam.cathodeLineThredsholdInner = 0.0;
		cylLowerParam.cathodeLineThredsholdMiddle = 0.0;
		cylLowerParam.cathodeLineThredsholdOuter = 0.0;
		cylLowerParam.cathodeLineWindowSize = 3;
		cylLowerParam.polesHeight = 100;
		cylLowerParam.minProminence = 1;

		cylLowerParam.polesMinDistance = 8;
		cylLowerParam.polesMaxDistance = 30;

		cylLowerParam.anodeThresholdInner = 0.4;
		cylLowerParam.anodeThresholdMiddle = 0.7;
		cylLowerParam.anodeThresholdOuter = 0.5;
		cylLowerParam.OneSidePoleNumb = 26;
		cylLowerParam.isCheckPoleNo = 0;
		cylLowerParam.isCheckPin = 0;

		cylLowerParam.skipPolesDistance = 0;
		cylLowerParam.MinCathode2Anode = 0.3;
		cylLowerParam.MaxCathode2Anode = 1.5;

		cylLowerParam.MinAnode2Case = 0;
		cylLowerParam.MaxAnode2Case = 6.0;
		cylLowerParam.MinCathode2Case = 0;
		cylLowerParam.MaxCathode2Case = 8.0;

		cylLowerParam.mAnode2CaseOffset = 0;
		cylLowerParam.mCathode2AnodeOffset = 0;
		cylLowerParam.mCathode2CaseOffset = 0;

		cylLowerParam.variationAnode2Case = 0;

		cylLowerParam.pixelSize = 0.0105;
		cylLowerParam.additionalLine = 2;
		cylLowerParam.gamma = 20;

		cylLowerParam.minLeaningDistance = 0;

		cylLowerParam.displayMode = 1;
		cylLowerParam.isSaveResults = 0; // > 0 : csv 저장, 2nd bit set : org image and result image save 
		cylLowerParam.resultPath = "D:\\Result1";
		cylLowerParam.cellID = "0";

		cylLowerAlgo.ImageProcessing(imgSrc, cylLowerParam, cylLowerResult);

		//std::cout << "Lower csvTitle :" << (std::string)CT2CA(cylLowerResult.csvTitleStr) << std::endl;
		//std::cout << "Lower csvResult :" << (std::string)CT2CA(cylLowerResult.csvResultStr) << std::endl << std::endl;

		//std::cout << "Lower csvTitleXM :" << (std::string)CT2CA(cylLowerResult.csvTitleStrXM4) << std::endl;
		//std::cout << "Lower csvResultXM :" << (std::string)CT2CA(cylLowerResult.csvResultStrXM4) << std::endl << std::endl;

		cv::imwrite("1 2 3.jpeg", cylLowerResult.resImg);
	}
	cv::waitKey(0);

}

