#include "ImageProcessorCylinderBatLowerJR.h"
#include "ImageProcessorBase.h"
#include <sys/timeb.h>
#include <fstream>
#include <xvtBattery/CylinderBatteryLower.h>
#include "GFuncCylinderBaterryJR.h"

using namespace xvt;
using namespace xvt::battery;

int nLowerProcessingCount = 0;
int gnLowerAngle = 0;
static double gLowerPoleDiffQueue[10][MAX_POLE_NO];
CString gsFilePath;
CString gLowerRecentResultStr;

namespace xf
{
	namespace ImageProcessor
	{
		CImageProcessorCylinderBatLowerJR::CImageProcessorCylinderBatLowerJR()
		{
		}

		CImageProcessorCylinderBatLowerJR::~CImageProcessorCylinderBatLowerJR()
		{
		}

		void CImageProcessorCylinderBatLowerJR::ImageProcessing(const cv::Mat srcImg16bit, xvt::CylinderJRInspection BatLowerInsp, BatResultStruct& bLowerResult) //xvt::battery::BatteryInspectionResult &BIresult)
		{
			xvt::battery::BatteryInspectionResult BIresult;

			CString sCellID = _T("0");

			{
				BatLowerInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH = true;
				//BatLowerInsp.inspectingItems.ANODE_TO_CASE_GAP = false;
				//BatLowerInsp.inspectingItems.CATHODE_TO_CASE_GAP = false;

				CString sCellID = BatLowerInsp.sCellID = _T("0"); 

//				cIU.GetSettingValue((UINT)CFactoryAlgorithm::CCylinderBatLowerCT::S::eBatteryDirection, BatLowerInsp.BatteryDirection);
				//BatLowerInsp.BatteryDirection = 1;

				BIresult.sData = "";
				std::vector<cv::Point> vtPointDefect;

				DWORD dwStart = GetTickCount();

				CString  csvTitleStr = _T("");
				CString  csvTitleStrXM4 = _T("");

				csvTitleStr = "Opcode,Angle,CellID, NGCode,Final, Ano2Cath, Ano2Base, Time, MinAno2Cath, MaxAno2Cath, AvgAno2Cath, MinAno2Base, MaxAno2Base, AvgAno2Base, MinCath2Base, MaxCath2Base, AvgCath2Base, ";
				csvTitleStrXM4 = "NGCode, R0000343P, R0000344P, R0000345P, R0000346P, R0000347P, R0000348P, Opcode, Angle, CellID, Final, Ano2Cath, Ano2Base, Time, MinCath2Base, MaxCath2Base, AvgCath2Base, ";

				for (int i = 0; i < MAX_POLE_NO; i++) csvTitleStr.AppendFormat(_T("Ano2Cat%d, "), i + 1);
				CString csvResultStr = _T("");
				CString csvResultStrXM4 = _T("");

				int inspectionErr = 0;

				{
					int nImageWidth = srcImg16bit.cols;
					int nImageHeight = srcImg16bit.rows;

					cv::Mat Img = srcImg16bit;

					cv::Mat resImg;
					xvt::Convert8Bits(Img, resImg);
					GammaCorrection(resImg, BatLowerInsp.gamma);
					xvt::ConvertRGB(resImg, resImg);

					CString timeStr = _T("");
					CTime cTime = CTime::GetCurrentTime();
					timeStr.AppendFormat(_T("%02d/%02d %02d:%02d:%02d"), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
					struct _timeb timebuffer;
					_ftime64_s(&timebuffer);
					CString strMiliSec = _T("");
					strMiliSec.Format(_T("%03d"), (int)(timebuffer.millitm));

					if (BatLowerInsp.UseBatteryInspection == NO_BAT_INSPECTION)
					{
						CString strA = timeStr;
						strA.AppendFormat(_T(", %d x %d"), nImageWidth, nImageHeight);

						cv::putText(resImg, "Lower - No Inspection", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, (std::string)CT2CA(strA), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 0.8);

						csvResultStrXM4.AppendFormat(_T("0, 0, 0, 0, 0, 0, 0,"));  // NGCode, MinAno2Cath, MaxAno2Cath, AvgAno2Cath, MinAno2Base, MaxAno2Base, AvgAno2Base
						csvResultStrXM4.AppendFormat(_T("1, %d"), BatLowerInsp.Angle); // opcode, angle   
						csvResultStrXM4.AppendFormat(_T("OK, OK, OK, ")); // Final, Ano2Cath, Ano2Base, MinCath2Base, MaxCath2Base, AvgCath2Base
						csvResultStrXM4.AppendFormat(_T("%s,"), timeStr + strMiliSec); // MinCath2Base, MaxCath2Base, AvgCath2Base
						csvResultStrXM4.AppendFormat(_T("0, 0, 0, "));

						gLowerRecentResultStr = csvTitleStrXM4 + "\n" + csvResultStrXM4;
	
						Img.copyTo(bLowerResult.img8bit);
						resImg.copyTo(bLowerResult.resImg);

						return;
					}

					BIresult.finalDecision = "NG";
					BIresult.Anodes2CaseDecision = "NG";
					BIresult.Anode2CathodeDecision = "NG";

					cv::Point errOutPosition = cv::Point(50, resImg.rows - 100);

					try
					{
						int resValue = Inspection(Img, BatLowerInsp, BIresult);
						nLowerProcessingCount++;

						if (resValue > 0) {
							inspectionErr = resValue;

							cv::Rect settingROI(BatLowerInsp.RoiX, BatLowerInsp.RoiY, BatLowerInsp.RoiWidth, BatLowerInsp.RoiHeight);
							cv::rectangle(resImg, settingROI, cv::Scalar(0, 255, 255), 3);

							std::string description;
							if (resValue == errImageFormat) { description = "Error: ROI or Image format";}
							else if (resValue == errBatterySize) { description = "Error: ROI Selection or Input Image";}
							else if (resValue == errWidthBattery) { description = BIresult.Description;}
							else if (resValue == errPixelSize) { description = "Error: Pixel Size cannot be 0";}
							else if (resValue == errBeadingArea) { description = "Error: ROI Selection";}
							else if (resValue == errBeadingInsp) { description = "Error: Beading Insp ";}
							else if (resValue == errJRFinding) { description = "Error: JR Finding ";}
							else if (resValue == errCenterPin) { description = "Error: Cannot find CENTER PIN ";}
							else if (resValue == errPoleXDetection) { description = "Error: Pole X Detection ";}
							else if (resValue == errPoleXDetection2) { description = "Error: Pole X Detection 2";}
							else if (resValue == errPoleXDetection3) { description = "Error: Pole X Detection 3";}
							else if (resValue == errPoleXDetection4) { description = "Error: Pole X Detection 4";}
							else if (resValue == errPoleXDetection5) { description = "Error: Pole X Detection 5";}
							else if (resValue == errPoleCathodeLine) { description = "Error: Pole Cathode Line ";}
							else if (resValue == errPoleCathodeLine1) { description = "Error: Pole Cathode Line 1";}
							else if (resValue == errPoleCathodeLine2) { description = "Error: Pole Cathode Line 2";}
							else if (resValue == errPoleCathodeLine3) { description = "Error: Pole Cathode Line 3";}
							else if (resValue == errPoleCathodeLine4) { description = "Error: Pole Cathode Line 4";}
							else if (resValue == errPoleCathodeLine5) { description = "Error: Pole Cathode Line 5";}
							else if (resValue == errPoleCathodeLine6) { description = "Error: Pole Cathode Line 6";}
							else if (resValue == errPoleAnodeLine) { description = "Error: Pole Anode Line ";}
							else if (resValue == errPoleNotRefinable) { description = "Error: Pole Not Refinable ";}
							else if (resValue == errPoleRefinement) { description = "Error: Pole Refinement  ";}
							else { description = "Error: " + BIresult.Description;}

							bLowerResult.Description = BIresult.Description;

							cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
							cv::putText(resImg, description, errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);

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

							bLowerResult.minCathode2Case = 0.0;
							bLowerResult.maxCathode2Case = 0.0;
							bLowerResult.avgCathode2Case = 0.0;

//							int baselineLoc = CheckLowerBaseline(pGI, BatLowerInsp);
							int baselineLoc = BIresult.yA2; 
							bLowerResult.baselineLoc = BIresult.yA2;

							cv::line(resImg, cv::Point(0, baselineLoc), cv::Point(3, baselineLoc), cv::Scalar(252, 10, 0), 2);

							CString strB = timeStr; strB.AppendFormat(_T(", %d x %d (%d)"), nImageWidth, nImageHeight, baselineLoc);
							cv::putText(resImg, (std::string)CT2CA(strB), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);

							if (BatLowerInsp.UseBatteryInspection == NORMAL_MULTI_INSPECTION)
							{
								cv::putText(resImg, "Angle: " + std::to_string(BatLowerInsp.Angle), cv::Point(nImageWidth - 200, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
							}

							gLowerRecentResultStr = "";

							Img.copyTo(bLowerResult.img8bit);
							resImg.copyTo(bLowerResult.resImg);
							//throw BIresult.Description;
						}
					}
					catch (const std::exception& ex) {
						// ...
						Img.copyTo(bLowerResult.img8bit);
						resImg.copyTo(bLowerResult.resImg);
						gLowerRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err1", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

						//						std::cout << ex.what() << std::endl;
						// theApp.WriteAlarmLog(_T("Lower algo : exception 1 :"));
					}
					catch (const std::string& ex)
					{
						std::exception_ptr p = std::current_exception();
						std::cout << ex.c_str() << std::endl;

						Img.copyTo(bLowerResult.img8bit);
						resImg.copyTo(bLowerResult.resImg);
						gLowerRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err2", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

						// theApp.WriteAlarmLog(_T("Lower algo : exception 2:") + (CString)ex.c_str());
					}
					catch (...)
					{
						std::exception_ptr p = std::current_exception();

						Img.copyTo(bLowerResult.img8bit);
						resImg.copyTo(bLowerResult.resImg);
						gLowerRecentResultStr = "";

						cv::putText(resImg, "Result: NG", cv::Point(440, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
						cv::putText(resImg, "Unknown Image Err3", errOutPosition, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

						// theApp.WriteAlarmLog(_T("Lower algo : exception 3:"));
					}
				
					DWORD dTime = GetTickCount() - dwStart;

					if (dTime != 0)
					{
					}

					bLowerResult.finalDecision = BIresult.finalDecision;
					bLowerResult.Anodes2CaseDecision = BIresult.Anodes2CaseDecision;
					bLowerResult.Anode2CathodeDecision = BIresult.Anode2CathodeDecision;
					bLowerResult.Cathode2CaseDecision = BIresult.Cathode2CaseDecision;

					bLowerResult.cTime = BIresult.cTime;
					bLowerResult.minCathode2Anode = BIresult.minCathode2Anode;
					bLowerResult.maxCathode2Anode = BIresult.maxCathode2Anode;
					bLowerResult.avgCathode2Anode = BIresult.avgCathode2Anode;
					bLowerResult.minAnode2Case    =  BIresult.minAnode2Case;
					bLowerResult.maxAnode2Case = BIresult.maxAnode2Case;
					bLowerResult.avgAnode2Case = BIresult.avgAnode2Case;
					bLowerResult.minCathode2Case = BIresult.minCathode2Case;
					bLowerResult.maxCathode2Case = BIresult.maxCathode2Case;
					bLowerResult.avgCathode2Case =  BIresult.avgCathode2Case;
					bLowerResult.sCathode2Anode = BIresult.sCathode2Anode;

					std::vector<cv::Point> vtCathodeCPoints;
					for(const auto& cat : BIresult.vtCathodes) vtCathodeCPoints.emplace_back(cat.x, cat.y);

					std::vector<cv::Point> vtAnodeCPoints;
					for(const auto& ano : BIresult.vtAnodes) vtAnodeCPoints.emplace_back(ano.x, ano.y);

					int opCode = 0;
					int ngCode = 0;
					int setJudge = 0;

					if (BIresult.finalDecision == "OK") opCode = 1;
					else if (BIresult.finalDecision == "NG") opCode = 2; //	opCode = bLowerResult.finalDecision ? 2 : 1;  // OK 1, NG 2 

					if (inspectionErr >= 100) csvResultStr.AppendFormat(_T("%d, "), inspectionErr);
					else
					{
						csvResultStr.AppendFormat(_T("%d, "), opCode);
						if (BIresult.Anode2CathodeDecision == "NG")            ngCode = NG_LowerAno2CathDistance;
						else if (BIresult.Anodes2CaseDecision == "NG")         ngCode = NG_LowerAno2CaseDistance;
                        else if (BIresult.Anode2CaseVariationDecision == "NG") ngCode = NG_LowerAno2CaseVariation;
					}

					csvResultStr.AppendFormat(_T("%d, "), BatLowerInsp.Angle);
					csvResultStr.AppendFormat(_T("%s, "), sCellID);
					if (BIresult.finalDecision == "OK") csvResultStr.AppendFormat(_T("%d, "), 0);
					else csvResultStr.AppendFormat(_T("%d, "), ngCode);

					csvResultStr.AppendFormat(_T("%s, "), CString(BIresult.finalDecision.c_str()));
					csvResultStr.AppendFormat(_T("%s, "), CString(BIresult.Anode2CathodeDecision.c_str()));
					csvResultStr.AppendFormat(_T("%s, "), CString(BIresult.Anodes2CaseDecision.c_str()));
					csvResultStr.AppendFormat(_T("%s, "), CString(BIresult.cTime.c_str()));
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.minCathode2Anode);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Anode);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Anode);

					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.minAnode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.maxAnode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.avgAnode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.minCathode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Case);
					csvResultStr.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Case);

			//============================================================================================
					if (BIresult.finalDecision == "OK") csvResultStrXM4.AppendFormat(_T("%d, "), 0);
					else csvResultStrXM4.AppendFormat(_T("%d, "), ngCode);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.minCathode2Anode);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Anode);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Anode);

					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.minAnode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.maxAnode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.avgAnode2Case);

					if (BIresult.finalDecision == "OK") csvResultStrXM4.AppendFormat(_T("%d, "), 1);  // opcode
					else csvResultStrXM4.AppendFormat(_T("%d, "), inspectionErr);

					csvResultStrXM4.AppendFormat(_T("%d, "), BatLowerInsp.Angle);
					csvResultStrXM4.AppendFormat(_T("%s, "), sCellID);
					csvResultStrXM4.AppendFormat(_T("%s, "), CString(BIresult.finalDecision.c_str()));
					csvResultStrXM4.AppendFormat(_T("%s, "), CString(BIresult.Anode2CathodeDecision.c_str()));
					csvResultStrXM4.AppendFormat(_T("%s, "), CString(BIresult.Anodes2CaseDecision.c_str()));
					csvResultStrXM4.AppendFormat(_T("%s, "), CString(BIresult.cTime.c_str()));
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.minCathode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.maxCathode2Case);
					csvResultStrXM4.AppendFormat(_T("%.3f, "), BIresult.avgCathode2Case);

			//============================================================================================

					if (BIresult.sCathode2Anode.size() > MAX_POLE_NO)
					{
						for (int i = 0; i < MAX_POLE_NO; i++)
						{
							csvResultStr.AppendFormat(_T("%.3f, "), BIresult.sCathode2Anode[i]);
						}
					}
					else
					{
						for (int i = 0; i < BIresult.sCathode2Anode.size(); i++)
						{
							csvResultStr.AppendFormat(_T("%.3f, "), BIresult.sCathode2Anode[i]);
						}
						for (int i = BIresult.sCathode2Anode.size(); i < MAX_POLE_NO; i++)
						{
							csvResultStr.AppendFormat(_T(" -1, "));
						}
					}

//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatLowerCT::R::eCSVTitle, csvTitleStr);
//					cIU.SetResultValue((UINT)CFactoryAlgorithm::CCylinderBatLowerCT::R::eCSVResult, csvResultStr);

					bLowerResult.opCode = opCode;
					bLowerResult.ngCode = ngCode;
					bLowerResult.setJudge = setJudge;

					if (inspectionErr >= 100) setJudge = inspectionErr;
					else  setJudge = opCode;

					gLowerRecentResultStr = csvTitleStrXM4 + "\n" + csvResultStrXM4;
					
					if (inspectionErr == 0) {
						bLowerResult.resImg = BIresult.resImg;
					}
					//Img.copyTo(bLowerResult.img8bit);
					//BIresult.resImg.copyTo(resImg);
					
					//BIresult.resImg.copyTo(bLowerResult.resImg);

					return;
				}
			}
		}

		int CImageProcessorCylinderBatLowerJR::Inspection(cv::Mat img, xvt::CylinderJRInspection BatLowerInsp, xvt::battery::BatteryInspectionResult& BIresult)
		{
			BIresult = {};

			int nImageWidth = img.cols;
			int nImageHeight = img.rows;

			cv::Mat Img = img;
			if (Img.empty()) return -1;

			nImageHeight = Img.rows;

			// check predefined setting 
			cv::Rect settingOuterROI = cv::Rect(BatLowerInsp.RoiX, BatLowerInsp.RoiY, BatLowerInsp.RoiWidth, BatLowerInsp.RoiHeight);
			settingOuterROI = RoiRefinement(settingOuterROI, cv::Size(nImageWidth, nImageHeight));
			BatLowerInsp.RoiX = settingOuterROI.x;
			BatLowerInsp.RoiY = settingOuterROI.y;
			BatLowerInsp.RoiWidth = settingOuterROI.width;
			BatLowerInsp.RoiHeight = settingOuterROI.height;

			if (BatLowerInsp.RoiX >= nImageWidth || BatLowerInsp.RoiY >= nImageHeight || BatLowerInsp.RoiWidth <= RESIZE_SCALE || BatLowerInsp.RoiHeight <= RESIZE_SCALE) {
				BIresult.Description = "Cannot find bounder battery, please check again outer ROI Setting or input image";
				return errBatterySize;
			}

			//-----end of pre-processing ------------------------------------------------------------------------
			xvt::battery::CylinderBatteryLower jrLowerIsp;

			jrLowerIsp.mRoi = {BatLowerInsp.RoiX, BatLowerInsp.RoiY, BatLowerInsp.RoiWidth, BatLowerInsp.RoiHeight};
			jrLowerIsp.mThreshold = BatLowerInsp.batteryThreshold;

			jrLowerIsp.mPoleRegionHeight = BatLowerInsp.poleRegionHeight;
			jrLowerIsp.mPoleLeaningThreshold = BatLowerInsp.PoleLeaningThreshold;
			jrLowerIsp.JR_ROIX = BatLowerInsp.JR_ROIX;
			jrLowerIsp.JR_ROIY = BatLowerInsp.JR_ROIY;
			jrLowerIsp.mCenterNeglectionWidth = BatLowerInsp.CenterNeglectionWidth;
			jrLowerIsp.mEnableAutoMode = true;

			jrLowerIsp.mCaseLineOffset = BatLowerInsp.tolerenceShift;

			jrLowerIsp.mCathodeLineThredsholdInner = BatLowerInsp.cathodeLineThredsholdInner;
			jrLowerIsp.mCathodeLineThredsholdMiddle = BatLowerInsp.cathodeLineThredsholdMiddle;
			jrLowerIsp.mCathodeLineThredsholdOuter = BatLowerInsp.cathodeLineThredsholdOuter;
			jrLowerIsp.mCathodeLineWindowSize = BatLowerInsp.cathodeLineWindowSize;

			//jrLowerIsp.mPoleHeight = BatLowerInsp.polesHeight;
			jrLowerIsp.mPolesProminenceThreshold = BatLowerInsp.minProminence;
			jrLowerIsp.mPolesDistanceRange.Set(BatLowerInsp.polesMinDistance, BatLowerInsp.polesMaxDistance);

			jrLowerIsp.mAnodeThresholdInner = BatLowerInsp.anodeThresholdInner;
			jrLowerIsp.mAnodeThresholdMiddle = BatLowerInsp.anodeThresholdMiddle;
			jrLowerIsp.mAnodeThresholdOuter = BatLowerInsp.anodeThresholdOuter;

			jrLowerIsp.mOneSidePoleNumber = BatLowerInsp.OneSidePoleNumb;
			jrLowerIsp.mIsCheckPoleNo = BatLowerInsp.isCheckPoleNo;
			jrLowerIsp.mSkipPolesDistance = BatLowerInsp.skipPolesDistance;

			jrLowerIsp.mValidCathode2AnodeRange.Set(BatLowerInsp.MinCathode2Anode, BatLowerInsp.MaxCathode2Anode);
			jrLowerIsp.mValidAnode2CaseRange.Set(BatLowerInsp.MinAnode2Case, BatLowerInsp.MaxAnode2Case);
			jrLowerIsp.mValidCathode2CaseRange.Set(BatLowerInsp.MinCathode2Case, BatLowerInsp.MaxCathode2Case);

			jrLowerIsp.mVariationAnode2Case = BatLowerInsp.variationAnode2Case;
			jrLowerIsp.mCathode2AnodeOffset = BatLowerInsp.mCathode2AnodeOffset;
			jrLowerIsp.mAnode2CaseOffset = BatLowerInsp.mAnode2CaseOffset;
			jrLowerIsp.mCathode2CaseOffset = BatLowerInsp.mCathode2CaseOffset;
			jrLowerIsp.mLeaningDistanceMin = BatLowerInsp.minLeaningDistance;

			jrLowerIsp.mInspectingItems.ANODE_TO_CASE_GAP = BatLowerInsp.inspectingItems.ANODE_TO_CASE_GAP;
			jrLowerIsp.mInspectingItems.ANODE_TO_CASE_VARIATION = (BatLowerInsp.variationAnode2Case != 0) || BatLowerInsp.inspectingItems.ANODE_TO_CASE_VARIATION;
			jrLowerIsp.mInspectingItems.ANODE_TO_CATHODE_LENGTH = BatLowerInsp.inspectingItems.ANODE_TO_CATHODE_LENGTH;
			jrLowerIsp.mInspectingItems.CATHODE_TO_CASE_GAP = BatLowerInsp.inspectingItems.CATHODE_TO_CASE_GAP;
			jrLowerIsp.mPixelSize = BatLowerInsp.pixelSize;
			jrLowerIsp.mDisplayMode = static_cast<xvt::battery::DisplayMode>(BatLowerInsp.displayMode);
			jrLowerIsp.setGamma(BatLowerInsp.gamma);
			jrLowerIsp.mTextFontScale = 0.7;
			jrLowerIsp.mTextLineSpace = 30;
			jrLowerIsp.mTextPosition = cv::Point(0,1200);

			xvt::battery::ERR_CODE err = jrLowerIsp.Inspection(Img, BIresult);

			//int baselineLoc = BIresult.yA2;
			xvt::Convert8Bits(Img, Img, false);

			if(err != xvt::battery::ERR_CODE::OK)
			{
				Img.release();
				//ctIsp.mIspResult.DrawResult(BIresult.resImg);
				//BIresult.resImg.release();
				return static_cast<int>(err);
			}

			cv::Mat res = BIresult.resImg;
		
			if ((BatLowerInsp.isSaveResults == 1) || (BatLowerInsp.UseBatteryInspection == DATA_WRITE_BAT_INSPECTION))
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
				if (filepath.size() == 0) fileName = "D:\\Result\\" + fileNameStr + "_" + tStr.substr(0, 4) + "_Lower_Data_1.csv";
				else                      fileName = filepath + "\\" + fileNameStr + "_" + tStr.substr(0, 4) + "_Lower_Data_1.csv";

				if (nLowerProcessingCount == 0)
				{
					for (int i = 0; i < 10; i++) for (int j = 0; j < MAX_POLE_NO; j++) gLowerPoleDiffQueue[i][j] = -1.0;

					std::string wStr = "";

					wStr += "No,Final, An2Cat, An2Case,time,filename,Angle,OutRoiX,OutRoiY,OutRoiW, OutRoiH, BatThresh, PoleRegion, PoleLeaning, JRRoiX, JRRoiY,";
					wStr += "NeglectCenterArea, TolShift, CatInner, CatMiddle, CatOuter, CatWinSize, PoleHeight, PoleDetecThresh, MinDistance, ";
					wStr += "MaxDistance, AnodeInner, AnodeMiddle, AnodeOuter, No.oneSidePole, isCheckPoleNo, CheckingPin, SkipPoleDist, PixSize, depthScale, additionalLine, SVminAn2CatMM, SVmaxAn2CatMM, SVminAn2CaseMM, SVmaxAn2CaseMM, SVminCat2BaseMM, SVmaxCat2BaseMM,,";
					wStr += "PMinAno2Cath, PMaxAno2Cath, PAvgAno2Cath, ,PMinAno2Base, PMaxAno2Base, PAvgAno2Base, ,PMinCath2Base, PMaxCath2Base, PAvgCath2Base,, PCath2Ano,";
					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';
					wStr += "Cat2Base,";
					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';
					wStr += "An2Base,";
					for (int i = 0; i < MAX_POLE_NO; i++) wStr += std::to_string(i+1) + ',';

					std::ofstream fout;
					fout.open(fileName, std::ios_base::out | std::ios_base::app);
					fout << wStr << std::endl;
					fout.close();
				}

				double poleDiffLength[MAX_POLE_NO];

				if (BIresult.sCathode2Anode.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++)	poleDiffLength[i] = BIresult.sCathode2Anode[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sCathode2Anode.size(); i++) poleDiffLength[i] = BIresult.sCathode2Anode[i];
					for (int i = BIresult.sCathode2Anode.size(); i < MAX_POLE_NO; i++) poleDiffLength[i] = -1;;
				}

				double poleDiffLength2[MAX_POLE_NO];

				if (BIresult.sCathode2Case.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength2[i] = BIresult.sCathode2Case[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sCathode2Case.size(); i++)	poleDiffLength2[i] = BIresult.sCathode2Case[i];
					for (int i = BIresult.sCathode2Case.size(); i < MAX_POLE_NO; i++) poleDiffLength2[i] = -1;;
				}

				double poleDiffLength3[MAX_POLE_NO];

				if (BIresult.sAnode2Case.size() > MAX_POLE_NO) {
					for (int i = 0; i < MAX_POLE_NO; i++) poleDiffLength3[i] = BIresult.sAnode2Case[i];
				}
				else
				{
					for (int i = 0; i < BIresult.sAnode2Case.size(); i++) poleDiffLength3[i] = BIresult.sAnode2Case[i];
					for (int i = BIresult.sAnode2Case.size(); i < MAX_POLE_NO; i++)	poleDiffLength3[i] = -1;;
				}

				std::string wStr = std::to_string(nLowerProcessingCount + 1) + ",";
//				if (pGI->IsLeft() ) wStr += std::to_string(0) + ",";
//				else wStr += std::to_string(90) + ",";
				wStr += BIresult.finalDecision + ",";
				wStr += BIresult.Anode2CathodeDecision + ",";
				wStr += BIresult.Anodes2CaseDecision + ",";
				wStr += BIresult.cTime + ",";

				wStr += outFileName + ",";
				wStr += std::to_string(BatLowerInsp.Angle) + ",";
				wStr += std::to_string(BatLowerInsp.RoiX) + ",";
				wStr += std::to_string(BatLowerInsp.RoiY) + ",";
				wStr += std::to_string(BatLowerInsp.RoiWidth) + ",";
				wStr += std::to_string(BatLowerInsp.RoiHeight) + ",";
				wStr += std::to_string(BatLowerInsp.batteryThreshold) + ",";
				wStr += std::to_string(BatLowerInsp.poleRegionHeight) + ",";
				wStr += std::to_string(BatLowerInsp.PoleLeaningThreshold) + ",";
				wStr += std::to_string(BatLowerInsp.JR_ROIX) + ",";
				wStr += std::to_string(BatLowerInsp.JR_ROIY) + ",";
				wStr += std::to_string(BatLowerInsp.CenterNeglectionWidth) + ",";
				wStr += std::to_string(BatLowerInsp.tolerenceShift) + ",";
				wStr += std::to_string(BatLowerInsp.cathodeLineThredsholdInner) + ",";
				wStr += std::to_string(BatLowerInsp.cathodeLineThredsholdMiddle) + ",";
				wStr += std::to_string(BatLowerInsp.cathodeLineThredsholdOuter) + ",";
				wStr += std::to_string(BatLowerInsp.cathodeLineWindowSize) + ",";
				wStr += std::to_string(BatLowerInsp.polesHeight) + ",";
				wStr += std::to_string(BatLowerInsp.minProminence) + ",";
				wStr += std::to_string(BatLowerInsp.polesMinDistance) + ",";
				wStr += std::to_string(BatLowerInsp.polesMaxDistance) + ",";

				wStr += std::to_string(BatLowerInsp.anodeThresholdInner) + ",";
				wStr += std::to_string(BatLowerInsp.anodeThresholdMiddle) + ",";
				wStr += std::to_string(BatLowerInsp.anodeThresholdOuter) + ",";
				wStr += std::to_string(BatLowerInsp.OneSidePoleNumb) + ",";
				wStr += std::to_string(BatLowerInsp.isCheckPoleNo) + ",";
				wStr += std::to_string(BatLowerInsp.isCheckPin) + ",";
				wStr += std::to_string(BatLowerInsp.skipPolesDistance) + ",";
				wStr += std::to_string(BatLowerInsp.pixelSize) + ",";
				wStr += std::to_string(BatLowerInsp.depthScale) + ",";
				wStr += std::to_string(BatLowerInsp.additionalLine) + ",";
				wStr += std::to_string(BatLowerInsp.MinCathode2Anode) + ",";
				wStr += std::to_string(BatLowerInsp.MaxCathode2Anode) + ",";
				wStr += std::to_string(BatLowerInsp.MinAnode2Case) + ",";
				wStr += std::to_string(BatLowerInsp.MaxAnode2Case) + ",";
				wStr += std::to_string(BatLowerInsp.MinCathode2Case) + ",";
				wStr += std::to_string(BatLowerInsp.MaxCathode2Case) + ", ,";

				wStr += std::to_string(BIresult.minCathode2Anode) + ",";
				wStr += std::to_string(BIresult.maxCathode2Anode) + ",";
				wStr += std::to_string(BIresult.avgCathode2Anode) + ", ,";
				wStr += std::to_string(BIresult.minAnode2Case) + ",";
				wStr += std::to_string(BIresult.maxAnode2Case) + ",";
				wStr += std::to_string(BIresult.avgAnode2Case) + ", ,";
				wStr += std::to_string(BIresult.minCathode2Case) + ",";
				wStr += std::to_string(BIresult.maxCathode2Case) + ",";
				wStr += std::to_string(BIresult.avgCathode2Case) + ", , ,";

				CImageProcessorBase::WrProcessingData(wStr, nLowerProcessingCount, 2*BatLowerInsp.OneSidePoleNumb, gLowerPoleDiffQueue, poleDiffLength, poleDiffLength2, poleDiffLength3, 54);

				std::ofstream fout;

				fout.open(fileName, std::ios_base::out | std::ios_base::app);
				fout << wStr << std::endl;
				fout.close();

//				cv::imwrite(filepath + "\\" + fileNameStr + "_" + tStr + "jpg", res);
			}

			CString strB = _T("");
			CTime cTime = CTime::GetCurrentTime();
			strB.AppendFormat(_T("%02d/%02d %02d:%02d:%02d"), cTime.GetMonth(), cTime.GetDay(), cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
			//strB.AppendFormat(_T(", %d x %d (%d)"), nImageWidth, nImageHeight, baselineLoc);
			cv::putText(res, (std::string)CT2CA(strB), cv::Point(50, nImageHeight - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 0.8);
			//cv::line(res, cv::Point(0, baselineLoc), cv::Point(3, baselineLoc), cv::Scalar(252, 10, 0), 2);

			return 0;
		}

	}
}



