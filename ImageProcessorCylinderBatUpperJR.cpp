#include "ImageProcessorCylinderBatUpperJR.h"
//#include "InspectCylinderBattery.h"
#include <ctime>
#include <iostream>
#include <fstream>
#include <numeric>
#include <chrono>
#include <sys/timeb.h>
#include <xvtBattery/CylinderBatteryUpper.h>
#include "ImageProcessorBase.h"
#include "GFuncCylinderBaterryJR.h"


#pragma warning(disable : 4996) //_CRT_SECURE_NO_WARNINGS

using namespace std;
using namespace xvt;
using namespace xvt::battery;

int nUpperProcessingCount = 0;
static double gUpperPoleDiffQueue[10][MAX_POLE_NO];
extern CString gsFilePath;

CString gUpperRecentResultStr;

namespace xf
{
	namespace ImageProcessor
	{
		CImageProcessorCylinderBatUpperJR::CImageProcessorCylinderBatUpperJR()
		{
		}

		CImageProcessorCylinderBatUpperJR::~CImageProcessorCylinderBatUpperJR()
		{
		}

		void CImageProcessorCylinderBatUpperJR::ImageProcessing(const cv::Mat srcImg16bit, xvt::CylinderJRInspection batInsp, BatResultStruct& bUpperResult) // xvt::battery::BatteryInspectionResult& bIresult);
		{
			xvt::battery::BatteryInspectionResult BIresult;

			{
				int t0 = clock();
				CString strTitle, cellStr = _T("");
				cellStr.Format(_T("Cell : %s"), "0");

				CString sCellID = batInsp.sCellID = cellStr;

				batInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH = true;
				//batInsp.inspectingItems.ANODE_TO_CASE_GAP = false;
				//batInsp.inspectingItems.ANODE_TO_CASE_VARIATION = false;
				//batInsp.inspectingItems.CATHODE_TO_CASE_GAP = false;
				batInsp.inspectingItems.CHECK_BEADING = false;

				std::vector<cv::Point> vtPointDefect;
				DWORD dwStart = GetTickCount();

				CString  csvTitleStr = _T("");
				CString  csvTitleStrXM4 = _T("");

				csvTitleStr = "Opcode, Angle,CellID, NgCode, Final, Time, MinCath2Base, MaxCath2Base, AvgCath2Base, ";
				csvTitleStrXM4 = "NgCode, R0000337P, R0000338P, R0000339P, Opcode, Angle, CellID, Final, Time,  ";

				for (int i = 0; i < MAX_POLE_NO; i++) csvTitleStr.AppendFormat(_T("Cat2Base%d, "), i + 1);
				CString csvResultStr = _T("");
				CString csvResultStrXM4 = _T("");

				int inspectionErr = 0;

				{
					int nImageWidth = srcImg16bit.cols;
					int nImageHeight = srcImg16bit.rows;

					cv::Mat Img = srcImg16bit;

					cv::Mat resImg;
					xvt::ConvertRGB(Img, resImg);

					int ngCode = 0;

					CString timeStr = _T("");
					CTime cTime = CTime::GetCurrentTime();
					timeStr.AppendFormat(_T("%02d/%02d %02d:%02d:%02d."), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
					struct _timeb timebuffer;
					_ftime64_s(&timebuffer);
					CString strMiliSec = _T("");
					strMiliSec.Format(_T("%03d"), (int)(timebuffer.millitm));

					if (batInsp.UseBatteryInspection == NO_BAT_INSPECTION)
					{
						CString strA = timeStr + strMiliSec;
						strA.AppendFormat(_T(", %d x %d"), nImageWidth, nImageHeight);
						cv::putText(resImg, (std::string)CT2CA(cellStr), cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
						cv::putText(resImg, "Upper - No Inspection", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, (std::string)CT2CA(strA), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 0.8);

						nUpperProcessingCount = 0;
						
						csvResultStrXM4.AppendFormat(_T("0, 0, 0, 0, ")); // NG code, MinCath2Base, MaxCath2Base, AvgCath2Base
						csvResultStrXM4.AppendFormat(_T("1, %d"), batInsp.Angle); // opcode, angle   
						csvResultStrXM4.AppendFormat(_T("%s, "), sCellID);
						csvResultStrXM4.AppendFormat(_T("OK, "));
						csvResultStrXM4.AppendFormat(_T("%s, "), timeStr + strMiliSec);

						Img.copyTo(bUpperResult.img8bit);
						resImg.copyTo(bUpperResult.resImg);

						gUpperRecentResultStr = csvTitleStrXM4 + "\n" + csvResultStrXM4;
//						theApp.WriteNormalLog(_T("1st Upper no insp end"));
						
						//DrawTextResult(bUpperResult.resImg, BIresult);
						//DrawPoleTextResult(bUpperResult.resImg, batInsp, BIresult);
						return;
					}

					BIresult.finalDecision = "NG";
					cv::Point errOutPosition = cv::Point(50, resImg.rows - 100);

					try {
						int resValue = Inspection(Img, batInsp, BIresult);
						nUpperProcessingCount++;
						if (resValue > 0) 
						{
							if(!BIresult.resImg.empty()) resImg = BIresult.resImg.clone();

							inspectionErr = resValue;
				
							cv::Rect settingROI(batInsp.RoiX, batInsp.RoiY, batInsp.RoiWidth, batInsp.RoiHeight);
							cv::rectangle(resImg, settingROI, cv::Scalar(0, 255, 255), 3);

							std::string description;
							if (resValue == errImageFormat) {description = "Error: ROI or Image format"; ngCode = NG_UpperImageFormat;}
							else if (resValue == errBatterySize) { description = "Error: ROI Selection or Input Image"; ngCode = NG_UpperEtc;}
							else if (resValue == errWidthBattery) { description = BIresult.Description; ngCode = NG_UpperCellWidth;} // "Err Battery Width";
							else if (resValue == errPixelSize) { description = "Error: Pixel Size cannot be 0"; ngCode = NG_UpperEtc;}
							else if (resValue == errBeadingArea) { description = "Error: ROI Selection"; ngCode = NG_UpperEtc;}
							else if (resValue == errBeadingInsp) { description = "Error: Beading Inspection"; ngCode = NG_UpperEtc;}
							else if (resValue == errJRFinding) { description = "Error: JR Finding "; ngCode = NG_UpperEtc;}
							else if (resValue == errReferenceCase) { description = "Error: Finding Reference Case Line"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection) { description = "Error: Pole X Detection "; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection2) { description = "Error: Pole X Detection 2"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection3) { description = "Error: Pole X Detection 3"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection4) { description = "Error: Pole X Detection 4"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleXDetection5) { description = "Error: Pole X Detection 5"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine) { description = "Error: Pole Cathode Line "; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine2) { description = "Error: Pole Cathode Line 2"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine3) { description = "Error: Pole Cathode Line 3"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine4) { description = "Error: Pole Cathode Line 4"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine5) { description = "Error: Pole Cathode Line 5"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleCathodeLine6) { description = "Error: Pole Cathode Line 6"; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleAnodeLine) { description = "Error: Pole Anode Line "; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleNotRefinable) { description = "Error: Pole Not Refinable "; ngCode = NG_UpperEtc;}
							else if (resValue == errPoleRefinement) { description = "Error: Pole Refinement  "; ngCode = NG_UpperEtc;	}
							else { description = "Error: " + BIresult.Description; ngCode = NG_UpperEtc;}
							cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
							cv::putText(resImg, description, errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
					
							bUpperResult.Description = BIresult.Description;

							CString timeStrA = _T("");
							CTime cTime = CTime::GetCurrentTime();
							timeStrA.AppendFormat(_T("%02d%02d_%02d%02d%02d."), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
							struct _timeb timebuffer;
							_ftime64_s(&timebuffer);
							CString strMiliSecA = _T("");
							strMiliSecA.Format(_T("%03d"), (int)(timebuffer.millitm));

							BIresult.cTime = CT2CA(timeStrA + strMiliSecA);
							BIresult.finalDecision = "NG";
							BIresult.minCathode2Anode = 0.0;
							BIresult.maxCathode2Anode = 0.0;
							BIresult.avgCathode2Anode = 0.0;
							BIresult.minAnode2Case = 0.0;
							BIresult.maxAnode2Case = 0.0;
							BIresult.avgAnode2Case = 0.0;
							BIresult.minCathode2Case = 0.0;
							BIresult.maxCathode2Case = 0.0;
							BIresult.avgCathode2Case = 0.0;

							bUpperResult.minCathode2Case = BIresult.minCathode2Case;
							bUpperResult.maxCathode2Case = BIresult.maxCathode2Case;
							bUpperResult.avgCathode2Case = BIresult.avgCathode2Case;

							//int baselineLoc = CheckUpperBaseline(Img, batInsp);

							//bUpperResult.baselineLoc = baselineLoc;

							//CString strB = timeStr; strB.AppendFormat(_T(", %d x %d (%d)"), nImageWidth, nImageHeight, baselineLoc);
							//cv::line(resImg, cv::Point(0, baselineLoc), cv::Point(3, baselineLoc), cv::Scalar(252, 10, 0), 2);

							//cv::putText(resImg, (std::string)CT2CA(strB), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
							cv::putText(resImg, (std::string)CT2CA(cellStr), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
							if (batInsp.UseBatteryInspection == NORMAL_MULTI_INSPECTION)
							{
								cv::putText(resImg, "Angle: " + std::to_string(batInsp.Angle), cv::Point(nImageWidth - 200, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
							}

							gUpperRecentResultStr = "";

							Img.copyTo(bUpperResult.img8bit);
							resImg.copyTo(bUpperResult.resImg);
						}
					}
					catch (const std::exception& ex) {
						Img.copyTo(bUpperResult.img8bit);
						resImg.copyTo(bUpperResult.resImg);
						gUpperRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err1", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
//						theApp.WriteAlarmLog(_T("Upper algo : exception 1:"));
					}
					catch (const std::string& ex) {
						std::exception_ptr p = std::current_exception();

						Img.copyTo(bUpperResult.img8bit);
						resImg.copyTo(bUpperResult.resImg);
						gUpperRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err2", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
//						theApp.WriteAlarmLog(_T("Upper algo : exception 2:") + (CString)ex.c_str());
					}
					catch (...)
					{
						Img.copyTo(bUpperResult.img8bit);
						resImg.copyTo(bUpperResult.resImg);
						gUpperRecentResultStr = "";

						std::exception_ptr p = std::current_exception();
						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err3", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
//						theApp.WriteAlarmLog(_T("Upper algo : exception 3:"));
					}

					DWORD dTime = GetTickCount() - dwStart;
					if (dTime != 0)
					{
					}

					bUpperResult.finalDecision = BIresult.finalDecision;

					bUpperResult.cTime = BIresult.cTime;
					bUpperResult.minCathode2Case = BIresult.minCathode2Case;
					bUpperResult.maxCathode2Case = BIresult.maxCathode2Case;
					bUpperResult.avgCathode2Case = BIresult.avgCathode2Case;

					CString eFinalDecision = CString(BIresult.finalDecision.c_str());
					CString eTime          = CString(BIresult.cTime.c_str());
					std::vector<double> eCathode2BaseMeas = BIresult.sCathode2Case;
					double eCathode2BaseMin = BIresult.minCathode2Case;
					double eCathode2BaseMax = BIresult.maxCathode2Case;
					double eCathode2BaseAvg = BIresult.avgCathode2Case;

					std::vector<cv::Point> vtCathodeCPoints;
					for(const auto& cat : BIresult.vtCathodes) vtCathodeCPoints.emplace_back(cat.x, cat.y);

//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatUpperCT::R::eCathodes, vtCathodeCPoints);

					std::vector<cv::Point> vtAnodeCPoints;
					for(const auto& ano : BIresult.vtAnodes) vtAnodeCPoints.emplace_back(ano.x, ano.y);
//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatUpperCT::R::eAnodes, vtAnodeCPoints);

					int opCode = 0;
					int setJudge = 0;

					if (BIresult.finalDecision == "OK") opCode = 1;
					else if (BIresult.finalDecision == "NG") opCode = 2; //	opCode = BIresult.finalDecision ? 2 : 1;  // OK 1, NG 2 

					if (inspectionErr >= 100) opCode = inspectionErr;
	
					if (inspectionErr >= 100) csvResultStr.AppendFormat(_T("%d, "), inspectionErr);
					else
					{
						csvResultStr.AppendFormat(_T("%d, "), opCode);
						if      (BIresult.Anode2CathodeDecision == "NG") ngCode = NG_UpperAno2CathDistance;
						else if (BIresult.Cathode2CaseDecision   == "NG") ngCode = NG_UpperCat2CaseDistance;
						else if (BIresult.Anode2CaseVariationDecision   == "NG") ngCode = NG_UpperAno2CaseVariation;
					}

					csvResultStr.AppendFormat(_T("%d, "), batInsp.Angle);
					csvResultStr.AppendFormat(_T("%s, "), sCellID);
					if (BIresult.finalDecision == "OK") csvResultStr.AppendFormat(_T("%d, "), 0);
					else csvResultStr.AppendFormat(_T("%d, "), ngCode);

					csvResultStr.AppendFormat(_T("%s, "), (CString)BIresult.finalDecision.c_str());
					csvResultStr.AppendFormat(_T("%s, "), (CString)BIresult.cTime.c_str());

					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.minCathode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Case);

					//-----------------------------------------------------------------------------------------
					if (BIresult.finalDecision == "OK") csvResultStrXM4.AppendFormat(_T("%d, "), 0);
					else csvResultStrXM4.AppendFormat(_T("%d, "), ngCode);

					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.minCathode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Case);
					if (BIresult.finalDecision == "OK") csvResultStrXM4.AppendFormat(_T("%d, "), 1);  // opcode
					else csvResultStrXM4.AppendFormat(_T("%d, "), inspectionErr);

					csvResultStrXM4.AppendFormat(_T("%d, "), batInsp.Angle);
					csvResultStrXM4.AppendFormat(_T("%s, "), sCellID);

					csvResultStrXM4.AppendFormat(_T("%s, "), (CString)BIresult.finalDecision.c_str());
					csvResultStrXM4.AppendFormat(_T("%s, "), (CString)BIresult.cTime.c_str());
					// ------------------------------------------------------------------------------------------

					if (BIresult.sCathode2Case.size() > MAX_POLE_NO)
					{
						for (int i = 0; i < MAX_POLE_NO; i++)
						{
							csvResultStr.AppendFormat(_T("%.3f, "), BIresult.sCathode2Case[i]);
						}
					}
					else
					{
						for (int i = 0; i < BIresult.sCathode2Case.size(); i++)
						{
							csvResultStr.AppendFormat(_T("%.3f, "), BIresult.sCathode2Case[i]);
						}
						for (int i = BIresult.sCathode2Case.size(); i < MAX_POLE_NO; i++)
						{
							csvResultStr.AppendFormat(_T(" -1, "));
						}
					}

//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatUpperCT::R::eCSVTitle, csvTitleStr);
//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatUpperCT::R::eCSVResult, csvResultStr);
			
					bUpperResult.opCode = opCode;
					bUpperResult.ngCode = ngCode;
					bUpperResult.setJudge = setJudge;
					gUpperRecentResultStr = csvTitleStrXM4 + "\n" + csvResultStrXM4;
					if (inspectionErr == 0) {
						bUpperResult.resImg = BIresult.resImg;
					}					

					return;
				}
//				std::cout << " Algo time :" << clock() - t0 << "ms" << std::endl;
			}
			// end of pGI
		}

		int CImageProcessorCylinderBatUpperJR::Inspection(cv::Mat img16bit, xvt::CylinderJRInspection BatInsp, xvt::battery::BatteryInspectionResult& BIresult)
		{
			BIresult.sHeader = "";
			BIresult.sData = "";
			BIresult.iResult = 0;
			BIresult.Description = "";
			BIresult.finalDecision = "";

			CString FinalDecision;

			int nImageWidth = img16bit.cols;
			int nImageHeight = img16bit.rows;

			cv::Mat Img = img16bit;
			if (Img.empty()) return -1;

			nImageHeight = Img.rows;

			// check predefined setting 
			cv::Rect settingOuterROI = cv::Rect(BatInsp.RoiX, BatInsp.RoiY, BatInsp.RoiWidth, BatInsp.RoiHeight);
			settingOuterROI = RoiRefinement(settingOuterROI, cv::Size(nImageWidth, nImageHeight));
			BatInsp.RoiX = settingOuterROI.x;
			BatInsp.RoiY = settingOuterROI.y;
			BatInsp.RoiWidth = settingOuterROI.width;
			BatInsp.RoiHeight = settingOuterROI.height;

			if (BatInsp.RoiX >= nImageWidth || BatInsp.RoiY >= nImageHeight || BatInsp.RoiWidth <= RESIZE_SCALE || BatInsp.RoiHeight <= RESIZE_SCALE) {
				BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
				return errBatterySize;
			}

			//-----end of pre-processing ------------------------------------------------------------------------
			xvt::battery::CylinderBatteryUpper jrUpperIsp;

			jrUpperIsp.mInspectingItems.ANODE_TO_CASE_GAP = BatInsp.inspectingItems.ANODE_TO_CASE_GAP;
			jrUpperIsp.mInspectingItems.ANODE_TO_CASE_VARIATION = BatInsp.inspectingItems.ANODE_TO_CASE_VARIATION;
			jrUpperIsp.mInspectingItems.ANODE_TO_CATHODE_LENGTH = BatInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH;
			jrUpperIsp.mInspectingItems.CATHODE_TO_CASE_GAP = BatInsp.inspectingItems.CATHODE_TO_CASE_GAP;
			jrUpperIsp.mInspectingItems.CHECK_BEADING = BatInsp.inspectingItems.CHECK_BEADING;

			jrUpperIsp.mRoi = {BatInsp.RoiX, BatInsp.RoiY, BatInsp.RoiWidth, BatInsp.RoiHeight};
			jrUpperIsp.mThreshold = BatInsp.batteryThreshold;

			jrUpperIsp.mPoleRegionHeight = BatInsp.poleRegionHeight;
			jrUpperIsp.mPoleLeaningThreshold = BatInsp.PoleLeaningThreshold;
			jrUpperIsp.JR_ROIX = BatInsp.JR_ROIX;
			jrUpperIsp.JR_ROIY = BatInsp.JR_ROIY;
			jrUpperIsp.mCenterNeglectionWidth = BatInsp.CenterNeglectionWidth;
			jrUpperIsp.mEnableAutoMode = true;

			jrUpperIsp.mCaseLineOffset = BatInsp.tolerenceShift;

			jrUpperIsp.mCathodeLineThredsholdInner = BatInsp.cathodeLineThredsholdInner;
			jrUpperIsp.mCathodeLineThredsholdMiddle = BatInsp.cathodeLineThredsholdMiddle;
			jrUpperIsp.mCathodeLineThredsholdOuter = BatInsp.cathodeLineThredsholdOuter;
			jrUpperIsp.mCathodeLineWindowSize = BatInsp.cathodeLineWindowSize;

			//jrUpperIsp.mheight = BatInsp.polesHeight;
			jrUpperIsp.mPolesProminenceThreshold = BatInsp.minProminence;
			jrUpperIsp.mPolesDistanceRange.Set(BatInsp.polesMinDistance, BatInsp.polesMaxDistance);

			jrUpperIsp.mAnodeThresholdInner = BatInsp.anodeThresholdInner;
			jrUpperIsp.mAnodeThresholdMiddle = BatInsp.anodeThresholdMiddle;
			jrUpperIsp.mAnodeThresholdOuter = BatInsp.anodeThresholdOuter;

			jrUpperIsp.mValidCathode2AnodeRange.Set(BatInsp.MinCathode2Anode, BatInsp.MaxCathode2Anode);
			jrUpperIsp.mValidAnode2CaseRange.Set(BatInsp.MinAnode2Case, BatInsp.MaxAnode2Case);
			jrUpperIsp.mValidCathode2CaseRange.Set(BatInsp.MinCathode2Case, BatInsp.MaxCathode2Case);

			jrUpperIsp.mCathode2AnodeOffset = BatInsp.mCathode2AnodeOffset;
			jrUpperIsp.mAnode2CaseOffset = BatInsp.mAnode2CaseOffset;
			jrUpperIsp.mCathode2CaseOffset = BatInsp.mCathode2CaseOffset;
			jrUpperIsp.mVariationAnode2Case = BatInsp.variationAnode2Case;
			jrUpperIsp.mLeaningDistanceMin = BatInsp.minLeaningDistance;

			jrUpperIsp.mInspectingItems.ANODE_TO_CASE_GAP = BatInsp.inspectingItems.ANODE_TO_CASE_GAP;
			jrUpperIsp.mInspectingItems.ANODE_TO_CASE_VARIATION = (BatInsp.variationAnode2Case != 0) || BatInsp.inspectingItems.ANODE_TO_CASE_VARIATION;
			jrUpperIsp.mInspectingItems.ANODE_TO_CATHODE_LENGTH = BatInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH;
			jrUpperIsp.mInspectingItems.CATHODE_TO_CASE_GAP = BatInsp.inspectingItems.CATHODE_TO_CASE_GAP;
			jrUpperIsp.setGamma(BatInsp.gamma);
			jrUpperIsp.mPixelSize = BatInsp.pixelSize;
			jrUpperIsp.mDisplayMode = static_cast<xvt::battery::DisplayMode>(BatInsp.displayMode);
			jrUpperIsp.mTextFontScale = 0.7;
			jrUpperIsp.mTextLineSpace = 30;
			jrUpperIsp.mTextPosition = cv::Point(0,200);

			xvt::battery::ERR_CODE err = jrUpperIsp.Inspection(Img, BIresult);
			
			//--- pre-preocessing --------------------------------------------------------------------------
			xvt::Convert8Bits(Img, Img, false);

			if(err != xvt::battery::ERR_CODE::OK)
			{
				Img.release();
				//jrUpperIsp.mIspResult.DrawResult(BIresult.resImg);
				return static_cast<int>(err);
			}

			cv::Mat res = BIresult.resImg;
			//cv::line(res, cv::Point(0, baselineLoc), cv::Point(3, baselineLoc), cv::Scalar(252, 10, 0), 2);

			if ((BatInsp.isSaveResults == 1) || (BatInsp.UseBatteryInspection == DATA_WRITE_BAT_INSPECTION))
			{
				std::string fileNameStr = "Result";
				std::string tStr = BIresult.cTime;

				std::string filepath;
				TCHAR szTmp[500];
				StrCpy(szTmp, gsFilePath); // pGI->GetImagePath()는 제대로 못받아옴, gsFilePath에는 path + file이 넘어옴
				PathRemoveFileSpec(szTmp);
				filepath = (std::string)CT2CA(szTmp);
				std::string outFileName = (std::string)CT2CA(gsFilePath);

				std::string fileName;
				if (filepath.size() == 0) fileName = "D:\\Result\\" + fileNameStr + "_" + tStr.substr(0, 4) + "_Upper_Data_1.csv";
				else                      fileName = filepath + "\\" + fileNameStr + "_" + tStr.substr(0, 4) + "_Upper_Data_1.csv";

				if (nUpperProcessingCount == 0)
				{
					for (int i = 0; i < 10; i++) for (int j = 0; j < MAX_POLE_NO; j++) gUpperPoleDiffQueue[i][j] = -1.0;

					std::string wStr = "";
					wStr += "No,Final, time, filename, Angle, OutRoiX, OutRoiY, OutRoiW, OutRoiH, BatThresh, BeadingH, D1_Start, PoleRegion, PoleLeaning, JRRoiX, JRRoiY,";
					wStr += "NeglectCenterArea, TolShift, CatInner, CatMiddle, CatOuter, CatWinSize, PoleHeight, PoleDetecThresh, MinDistance, ";
					wStr += "MaxDistance, AnodeInner, AnodeMiddle, AnodeOuter, No.oneSidePole, isCheckPoleNo, SkipPoleDist, PixSize,delthScale, additionalLine, SVminAn2CatMM, SVmaxAn2CatMM, SVminAn2BaseMM, SVmaxAn2BaseMM, SVminCat2BaseMM, SVmaxCat2BaseMM, ";
//					wStr += "MinAno2Cath, MaxAno2Cath, AvgAno2Cath, ,MinAno2Base, MaxAno2Base, AvgAno2Base,";
					wStr += ", PMinCath2Base, PMaxCath2Base, PAvgCath2Base, ,";
//					wStr += ", Cat2An, ";
//					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';
					wStr += "Cat2Base,";
					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';
//					wStr += "An2Case,";
//					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';

					std::ofstream fout;
					fout.open(fileName, std::ios_base::out | std::ios_base::app);
					fout << wStr << std::endl;
					fout.close();
				}

//				double poleDiffLength[MAX_POLE_NO];
//				if (BIresult.sCathode2Anode.size() > MAX_POLE_NO) {
//					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength[i] = BIresult.sCathode2Anode[i];
//				}
//				else
//				{
//					for (int i = 0; i < BIresult.sCathode2Anode.size(); i++) poleDiffLength[i] = BIresult.sCathode2Anode[i];
//					for (int i = BIresult.sCathode2Anode.size(); i < MAX_POLE_NO; i++) poleDiffLength[i] = -1;
//				}

				double poleDiffLength2[MAX_POLE_NO];

				if (BIresult.sCathode2Case.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sCathode2Case.size(); i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
					for (int i = BIresult.sCathode2Case.size(); i < MAX_POLE_NO; i++) poleDiffLength2[i] = -1;
				}

//				double poleDiffLength3[MAX_POLE_NO];
//				if (BIresult.sAnode2Case.size() > MAX_POLE_NO) {
//					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength3[i] = BIresult.sAnode2Case[i];
//				}
//				else
//				{
//					for (int i = 0; i < BIresult.sAnode2Case.size(); i++) poleDiffLength3[i] = BIresult.sAnode2Case[i];
//					for (int i = BIresult.sAnode2Case.size(); i < MAX_POLE_NO; i++) poleDiffLength3[i] = -1;
//				}

				std::string wStr = std::to_string(nUpperProcessingCount + 1) + ", ";
//				if (pGI->IsLeft()) wStr += std::to_string(0) + ",";
//				else wStr += std::to_string(90) + ",";
				wStr += BIresult.finalDecision + ",";
//				wStr += BIresult.Anode2CathodeDecision + ",";
//				wStr += BIresult.Anodes2CaseDecision + ",";
				wStr += BIresult.cTime + ",";

				wStr += outFileName + ",";
				wStr += std::to_string(BatInsp.Angle) + ",";

				wStr += std::to_string(BatInsp.RoiX) + ",";
				wStr += std::to_string(BatInsp.RoiY) + ",";
				wStr += std::to_string(BatInsp.RoiWidth) + ",";
				wStr += std::to_string(BatInsp.RoiHeight) + ",";
				wStr += std::to_string(BatInsp.batteryThreshold) + ",";
				wStr += std::to_string(BatInsp.minBeadingHeight) + ",";
				wStr += std::to_string(BatInsp.D1StartPosition) + ",";
				wStr += std::to_string(BatInsp.poleRegionHeight) + ",";
				wStr += std::to_string(BatInsp.PoleLeaningThreshold) + ",";
				wStr += std::to_string(BatInsp.JR_ROIX) + ",";
				wStr += std::to_string(BatInsp.JR_ROIY) + ",";
				wStr += std::to_string(BatInsp.CenterNeglectionWidth) + ",";
				wStr += std::to_string(BatInsp.tolerenceShift) + ",";
				wStr += std::to_string(BatInsp.cathodeLineThredsholdInner) + ",";
				wStr += std::to_string(BatInsp.cathodeLineThredsholdMiddle) + ",";
				wStr += std::to_string(BatInsp.cathodeLineThredsholdOuter) + ",";
				wStr += std::to_string(BatInsp.cathodeLineWindowSize) + ",";
				wStr += std::to_string(BatInsp.polesHeight) + ",";
				wStr += std::to_string(BatInsp.minProminence) + ",";
				wStr += std::to_string(BatInsp.polesMinDistance) + ",";
				wStr += std::to_string(BatInsp.polesMaxDistance) + ",";

				wStr += std::to_string(BatInsp.anodeThresholdInner) + ",";
				wStr += std::to_string(BatInsp.anodeThresholdMiddle) + ",";
				wStr += std::to_string(BatInsp.anodeThresholdOuter) + ",";
				wStr += std::to_string(BatInsp.OneSidePoleNumb) + ",";
				wStr += std::to_string(BatInsp.isCheckPoleNo) + ",";
				wStr += std::to_string(BatInsp.skipPolesDistance) + ",";
				wStr += std::to_string(BatInsp.pixelSize) + ",";
				wStr += std::to_string(BatInsp.depthScale) + ",";
				wStr += std::to_string(BatInsp.additionalLine) + ",";
				wStr += std::to_string(BatInsp.MinCathode2Anode) + ",";
				wStr += std::to_string(BatInsp.MaxCathode2Anode) + ",";
				wStr += std::to_string(BatInsp.MinAnode2Case) + ",";
				wStr += std::to_string(BatInsp.MaxAnode2Case) + ",";
				wStr += std::to_string(BatInsp.MinCathode2Case) + ",";
				wStr += std::to_string(BatInsp.MaxCathode2Case) + ", ,";

//				wStr += std::to_string(BIresult.minCathode2Anode) + ",";
//				wStr += std::to_string(BIresult.maxCathode2Anode) + ",";
//				wStr += std::to_string(BIresult.avgCathode2Anode) + ", ,";
//				wStr += std::to_string(BIresult.minAnode2Case) + ",";
//				wStr += std::to_string(BIresult.maxAnode2Case) + ",";
//				wStr += std::to_string(BIresult.avgAnode2Case) + ", ,";
				wStr += std::to_string(BIresult.minCathode2Case) + ",";
				wStr += std::to_string(BIresult.maxCathode2Case) + ",";
				wStr += std::to_string(BIresult.avgCathode2Case) + ", , ,";

				//wStr += std::to_string(BIresult.corverHeight) + ", ";
				//wStr += std::to_string(BIresult.corverDiameter) + ",";
				//wStr += std::to_string(BIresult.outerDiameter) + ",";
				//wStr += std::to_string(BIresult.grooveDepth) + ",";
				//wStr += std::to_string(BIresult.grooveHeight) + ", ,";

				if (BIresult.sCathode2Case.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sCathode2Case.size(); i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
					for (int i = BIresult.sCathode2Case.size(); i < MAX_POLE_NO; i++) poleDiffLength2[i] = -1;
				}

				for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(poleDiffLength2[i]) + ",";

//				CImageProcessorBase::WrProcessingData(wStr, nUpperProcessingCount, 2*BatInsp.OneSidePoleNumb, gUpperPoleDiffQueue, poleDiffLength2, poleDiffLength, poleDiffLength3, 45);

				std::ofstream fout;

				fout.open(fileName, std::ios_base::out | std::ios_base::app);

				fout << wStr << std::endl;
				fout.close();

//				cv::imwrite(filepath + "\\" + fileNameStr + "_" + tStr + "jpg", res);
				//cv::imwrite(outFileName + "_" + tStr + "jpg", res);
			}

			//change image from cv::Mat to *int 

			CString strB = _T("");
			CTime cTime = CTime::GetCurrentTime();
			strB.AppendFormat(_T("%02d/%02d %02d:%02d:%02d"), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
			//strB.AppendFormat(_T(", %d x %d (%d)"), nImageWidth, nImageHeight, baselineLoc);
			cv::putText(res, (std::string)CT2CA(strB), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);

			if (BatInsp.UseBatteryInspection == NORMAL_MULTI_INSPECTION)
			{
				cv::putText(res, "Angle: " + std::to_string(BatInsp.Angle), cv::Point(nImageWidth - 200, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
			}

			CString strTitle, cellStr = _T("");
			cellStr.Format(_T("Cell : %s"), "0");
			cv::putText(res, (std::string)CT2CA(cellStr), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);

			return 0;
		}
	}
}