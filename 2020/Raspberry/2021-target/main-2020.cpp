/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "GripPipeline.h"

#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "cameraserver/CameraServer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <networktables/NetworkTableInstance.h>

#include <iostream>
#include <cmath>

//
// Timing code from http://www.firstobject.com/getmillicount-milliseconds-portable-c++.htm
// Usage:
//   int start = GetMilliCount();
//   ... do processing ...
//   int elapsed = GetMilliSpan(start);
//
#include <sys/timeb.h>

int GetMilliCount()
{
  // Something like GetTickCount but portable
  // It rolls over every ~ 12.1 days (0x100000/24/60/60)
  // Use GetMilliSpan to correct for rollover
  timeb tb;
  ftime(&tb);
  int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
  return nCount;
}

int GetMilliSpan( int nTimeStart )
{
  int nSpan = GetMilliCount() - nTimeStart;
  if ( nSpan < 0 )
    nSpan += 0x100000 * 1000;
  return nSpan;
}

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

static const char* configFile = "/boot/frc.json";

namespace {

unsigned int team;
bool server = false;

struct CameraConfig {
  std::string name;
  std::string path;
  wpi::json config;
  wpi::json streamConfig;
};

std::vector<CameraConfig> cameraConfigs;

wpi::raw_ostream& ParseError() {
  return wpi::errs() << "config error in '" << configFile << "': ";
}

bool ReadCameraConfig(const wpi::json& config) {
  CameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read camera name: " << e.what() << '\n';
    return false;
  }

  // path
  try {
    c.path = config.at("path").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "camera '" << c.name
                 << "': could not read path: " << e.what() << '\n';
    return false;
  }

  // stream properties
  if (config.count("stream") != 0) c.streamConfig = config.at("stream");

  c.config = config;

  cameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadConfig() {
  // open config file
  std::error_code ec;
  wpi::raw_fd_istream is(configFile, ec);
  if (ec) {
    wpi::errs() << "could not open '" << configFile << "': " << ec.message()
                << '\n';
    return false;
  }

  // parse file
  wpi::json j;
  try {
    j = wpi::json::parse(is);
  } catch (const wpi::json::parse_error& e) {
    ParseError() << "byte " << e.byte << ": " << e.what() << '\n';
    return false;
  }

  // top level must be an object
  if (!j.is_object()) {
    ParseError() << "must be JSON object\n";
    return false;
  }

  // team number
  try {
    team = j.at("team").get<unsigned int>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read team number: " << e.what() << '\n';
    return false;
  }

  // ntmode (optional)
  if (j.count("ntmode") != 0) {
    try {
        server = false;
        auto str = j.at("ntmode").get<std::string>();
        wpi::StringRef s(str);
        if (s.equals_lower("client")) {
      } else if (s.equals_lower("server")) {
        server = true;
      } else {
        ParseError() << "could not understand ntmode value '" << str << "'\n";
      }
    } catch (const wpi::json::exception& e) {
      ParseError() << "could not read ntmode: " << e.what() << '\n';
    }
  }

  // cameras
  try {
    for (auto&& camera : j.at("cameras")) {
      if (!ReadCameraConfig(camera)) return false;
    }
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read cameras: " << e.what() << '\n';
    return false;
  }

  return true;
}

cs::UsbCamera StartCamera(const CameraConfig& config) {
  wpi::outs() << "Starting camera '" << config.name << "' on " << config.path
              << '\n';
  auto inst = frc::CameraServer::GetInstance();
  cs::UsbCamera camera{config.name, config.path};
  auto server = inst->StartAutomaticCapture(camera);

  camera.SetConfigJson(config.config);
//  // It takes a while to open a new connection so keep it open
//  camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

  if (config.streamConfig.is_object())
    server.SetConfigJson(config.streamConfig);

  return camera;
}

// example pipeline
class MyPipeline : public frc::VisionPipeline {
 public:
  int val = 0;

  void Process(cv::Mat& mat) override {
    ++val;
  }
};

}  // namespace

int main(int argc, char* argv[]) {

  if (argc >= 2) configFile = argv[1];

  // read configuration
  if (!ReadConfig()) return EXIT_FAILURE;

  // start cameras
  std::vector<cs::VideoSource> cameras;
  for (auto&& cameraConfig : cameraConfigs)
    cameras.emplace_back(StartCamera(cameraConfig));

  //
  // On a Raspberry Pi 3B+, if all the USB ports connect to USB cameras then the
  // cameras can be uniquely identified by the USB device pathnames as follows:
  //
  //	/----------------------------\
  //	| |      | | USB1 | | USB3 | |
  //	| |  IP  |  ======   ======  |
  //	| |      | | USB2 | | USB4 | |
  //	\----------------------------/
  //
  //  USB1: /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.1.2:1.0-video-index0
  //  USB2: /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.1.3:1.0-video-index0
  //  USB3: /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.3:1.0-video-index0
  //  USB4: /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.2:1.0-video-index0
  //

  //UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture("Front",
  //  "/.dev/v4l/by-path/platform-3f980000.usb-usb-0:1.2:1.0-video-index0");
  //frontCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
  //frontCamera.setBrightness(50);
  //frontCamera.setWhiteBalanceHoldCurrent();
  //frontCamera.setExposureManual(15);

  // Set up a pivision network table for sending results
  bool server = false;  // Start a server unless roboRIO is running
  int team = 6996;
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  if (server) {
    std::cout << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } else {
    std::cout << "Setting up NetworkTables client for team " << team << '\n';
    ntinst.StartClientTeam(team);
  }
  // Create a new table for vision stuff
  //std::cout << "Getting /SmartDashboard\n";
  //auto dashboard1 = ntinst.GetTable("/SmartDashboard");

  std::cout << "Getting pivision\n";
  auto dashboard = ntinst.GetTable("pivision");
  int cx = 160;
  int cy = 120;
  dashboard->PutNumber("cx", cx);
  dashboard->PutNumber("cy", cy);
  dashboard->PutNumber("view", 0.0);
  // start separate image processing threads for each camera if present
  if (cameras.size() >= 1) {

    // c.name = config.at("name").get<std::string>();
    // auto str = cameras.at("name").get<std::string>();
    // wpi::StringRef s(str);
    // if (s.equals_lower("front")) {

    std::thread([&] {
      // Control bandwidth by defining output resolution and camera frame rate divider
      const double kWidth = 320.0;
      const double kHeight = 240.0;
      const int kFrameRateDivider = 2;

      // Front facing drive camera. We just want to draw cross hairs on this.
      cs::CvSink FrontCam = frc::CameraServer::GetInstance()->GetVideo(cameras[0]);
      // Setup a CvSource. This will send images back to the Dashboard
      cs::CvSource FrontSvr =
        frc::CameraServer::GetInstance()->PutVideo("FrontCam", kWidth, kHeight);
      // FrontSvr.SetFPS(10); // This does not seem to work
      cs::CvSource DebugSvr =
        frc::CameraServer::GetInstance()->PutVideo("DebugCam", kWidth, kHeight);

      // A smaller window for lower latency
      cs::CvSource FrontSmall =
        frc::CameraServer::GetInstance()->PutVideo("FrontSmall", kWidth/2, kHeight/2);

      auto *Pipeline = new grip::GripPipeline();

      // Create mats to hold images
      cv::Mat frontMat;
      //cv::Mat frontProc;
      cv::Mat frontView;
      cv::Mat frontSmall;
      int counter = 0;

      while (true) {
        int start = GetMilliCount();

        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (FrontCam.GrabFrame(frontMat) == 0) {
          // Send error to the output
          FrontSvr.NotifyError(FrontCam.GetError());
          // skip the rest of the current iteration
          continue;
        }

        cx = dashboard->GetNumber("cx", 0.0);
        cy = dashboard->GetNumber("cy", 0.0);

        //std::cout << GetMilliSpan(start) << "ms, ";

        //cv::resize(frontMat, frontProc, cv::Size(160, 120), 0.0, 0.0, cv::INTER_AREA);

        Pipeline->Process(frontMat);

        //std::cout << GetMilliSpan(start) << "ms, ";

        std::vector<grip::Line> lines = *Pipeline->GetFilterLinesOutput();
        std::vector<grip::Line> lines2;
        lines2.clear();

	printf("Found %d lines\n", lines.size());
        int mmax = 50;//replace (define as a variable) for 90-max in the code before (min as 15, max as 35)
        int mmin = 78; //replace 90-min
        int pmin = 100; //replace 90+min
        int pmax = 125; //replace 90+max
        for (int i = 0; i < lines.size(); i++) {
          double x1, y1, x2, y2;
          double angle = lines[i].angle();
          double length = lines[i].length();
          if (angle < 0){
            double x, y;
            x = lines[i].x1;
            lines[i].x1 = lines[i].x2;
            lines[i].x2 = x;
            y = lines[i].y1;
            lines[i].y1 = lines[i].y2;
            lines[i].y2 = y;
            angle = angle + 180;
          }
          if (((mmax < angle && angle < mmin) || (angle > pmin && angle < pmax)) && length > 1) {
            lines2.push_back(lines[i]);
          }
        }
          
	printf("%d lines have correct angle\n", lines2.size());
        bool match[lines2.size()] = {false};
        double tx = 0;
        double ty = 0;
        int count = 0;
        for (int i = 0; i < lines2.size(); i++) {
          bool foundmatch = false;
          double angle1 = lines2[i].angle();
          double length1 = lines2[i].length();
          for (int j = i + 1; j < lines2.size(); j++) {
            double angle2 = lines2[j].angle();
            double length2 = lines2[j].length();
            int min = 15;
            int max = 35;

#define NEWCODE
#if defined(NEWCODE)
            // check that angles are on opposite sides of target
            if (((angle1 > 90) && (angle2 < 90)) ||
                ((angle1 < 90) && (angle2 > 90))) {
              foundmatch = true;
            }
#else
            // check that angles are on opposite sides of target
            if ((angle1 > mmax) && (angle1 < mmin) &&
              (angle2 > pmin) && (angle2 < pmax)) {
              foundmatch = true;
            }
#endif

#if defined(NEWCODE)
            // check that lengths are similar
			if ((length1 < 0.75 * length2) || (length2 < 0.75 * length1))
			  foundmatch = false;
#else
            // check that lengths are similar
			if ((length1 < 0.75 * length2) || (length1 > 1.25 * length2))
			  foundmatch = false;
#endif

#if defined(NEWCODE)
			// (0, 0) is top left of image
            int line1xbot = lines2[i].x2;
            int line1ybot = lines2[i].y2;
            int line1xtop = lines2[i].x1;
            int line1ytop = lines2[i].y1;
            int line2xbot = lines2[j].x2;
            int line2ybot = lines2[j].y2;
            int line2xtop = lines2[j].x1;
            int line2ytop = lines2[j].y1;
#else
            int line1xtop = lines2[i].x2;
            int line1ytop = lines2[i].y2;
            int line1xbot = lines2[i].x1;
            int line1ybot = lines2[i].y1;
            int line2xtop = lines2[j].x2;
            int line2ytop = lines2[j].y2;
            int line2xbot = lines2[j].x1;
            int line2ybot = lines2[j].y1;
#endif
			if (foundmatch) {
              double distance;
              double x1, y1, x2, y2;
	      printf("Angles and lengths matched. Checking spacing.\n");
              x1 = lines2[i].x2;
              y1 = lines2[i].y2;
              x2 = lines2[j].x2;
              y2 = lines2[j].y2;
              distance = sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );

#if defined(NEWCODE)
              // check distance between them
			  double average_length = (length1 + length2) / 2.0;
              if ((distance < average_length * 0.5) || (distance > average_length * 1.5))
                foundmatch = false;
              // check that lines don't cross each other
              // The angles look to be the wrong way around, but it works this way around.
              // Maybe Grip angles are positive clockwise
              if (((angle2 > 90) && (line2xbot < line1xbot)) ||
				  ((angle1 > 90) && (line1xbot < line2xbot)))
                foundmatch = false;
#else
              // check distance between them
              if ((distance < length2 * 0.8) || (distance > 1.25 * length2))
                foundmatch = false;
              // check that lines don't cross each other
              if (line2xbot < line1xbot)
                foundmatch = false;
#endif
            }
            if (foundmatch)
              printf("Found (%d, %d), (%d, %d), (%d, %d), (%d, %d)\n", line1xtop, line1ytop, line1xbot, line1ybot, line2xtop, line2ytop, line2xbot, line2ybot);

// Example printout
// (144, 170), (136, 148), (164, 174), (173, 155)
// The angle of the two lines: 70.016893 115.346176
// (144, 170), (136, 148), (166, 177), (177, 156)
// (142, 173), (133, 149), (164, 174), (173, 155)
// The angle of the two lines: 69.443955 115.346176
// (142, 173), (133, 149), (166, 177), (177, 156)
// The angle of the two lines: 69.443955 117.645975

#if defined(NEWCODE)
//              if (((angle1 > 90) && (line2xbot < line1xbot)) ||
//				  ((angle2 > 90) && (line1xbot < line2xbot)))
//                foundmatch = false;
//              }
#if 1
              if ((line1ytop < (line2ytop - length2 * 0.3)) || (line1ytop > (line2ytop + length2 * 0.3))) {
                  foundmatch = false;
              }
              if ((line2ytop < (line1ytop - length1 * 0.13)) || (line2ytop > (line1ytop + length1 * 0.3))) {
                  foundmatch = false;
              }
//              if (line1xtop > line2xbot) {
//                  foundmatch = false;
//              }
              if ((line2xbot < (line1xbot - 1.3 * length1)) || (line2xbot > (line1xbot + 1.3 * length1))) {
                  foundmatch = false;
              }
              if ((line1xbot < (line2xbot - 1.3 * length2)) || (line1xbot > (line2xbot + 1.3 * length2))) {
                  foundmatch = false;
              }
              if (line1xtop < 50 ||line1xtop > 270) {
                  foundmatch = false;
              }
              if (line2xtop < 50 ||line2xtop > 270) {
                  foundmatch = false;
              }
#endif
#else
              if ((line1ybot < (line2ybot - length2 * 0.3)) || (line1ybot > (line2ybot + length2 * 0.3))) {
                  foundmatch = false;
              }
              if (line1xbot > line2xtop)
                  foundmatch = false;

              if ((line2ybot < (line1ybot - length1 * 0.3)) || (line2ybot > (line1ybot + length1 * 0.3))) {
                  foundmatch = false;
              }
              if ((line2xtop < (line1xtop + 0.3 * length1)) || (line2xtop > (line1xtop + 2.0 * length1))) {
                  foundmatch = false;
              }
#endif

              if (foundmatch) {
                double x1, y1, x2, y2;
                x1 = lines2[i].x2;
                y1 = lines2[i].y2;
                x2 = lines2[j].x2;
                y2 = lines2[j].y2;
                tx = tx + (x1 + x2)/2;
                ty = ty + (y1 + y2)/2;
                count++;
                match[i] = true;
                match[j] = true;
                printf("The angle of the two lines: %lf %lf\n",angle1, angle2);
              }
            }
          }
          printf(".\n");
          double pi = 3.14159265358979323846;
          double midx = 0;
          double midy = 0;
          double degreetx = 0;
          double distancety = 0;
          double cameraheight = 32.0; // 37.25;
          if (count > 0) {
            midx = tx / count;
            midy = ty / count;
            printf("Mid = (%lf,%lf)\n", midx, midy);
#if 0
            // For raw data collection and calculations for measuring distance
            // see KoalafiedShare google drive under software/2020/Data collection...
            degreetx = (midx-cx) * (38.0/160.0);
            double theta = ( 18 - ((cy - 120) + midy - 25) /4.2) * (pi/180.0);
            distancety = (83.25 - cameraheight) / (tan (theta) );
#else
            // New approach because old one was not working (probably old robot):
            // View top is 79cm above view centre when distance is 150cm, so slope = 79/150
            // Centre is 120 pixels down from top, so we scale by height_in_pixels/120
            // So:
            //   slope = ((cy - midy)/120) * (79/150)
            // If the top was the target height (h) then h/distancety = 79/150
            degreetx = (midx-cx) * (38.0/160.0);
            double slope = ((cy - midy)/120.0) * (79.0/150.0);
            distancety = (83.25 - cameraheight) / slope;
#endif
            printf("The degree from the goal is %lf\n",degreetx);
            printf("The distance when the degree is 0 is %lf\n\n",distancety);
            dashboard->PutNumber("tx", degreetx);
            dashboard->PutNumber("ty", distancety);
            dashboard->PutNumber("tv", 1.0);
	    printf("found target\n");
          } else {
            dashboard->PutNumber("tv", 0.0);
	    printf("lost target\n");
	        }
        
          //std::cout << GetMilliSpan(start) << "ms, ";
        
          // Skip frames (when counter is not 0) to reduce bandwidth
          counter = (counter + 1) % kFrameRateDivider;
          if (!counter) {
  	        // select the view to display
	          int view = dashboard->GetNumber("view", 0.0);
            switch (view) {
              case 0:
              default:
                // Draw the calibration crosshair on the front camera image
                line(frontMat, cv::Point(cx, 0), cv::Point(cx, kHeight), cv::Scalar(128, 128, 128), 1);
                line(frontMat, cv::Point(0, cy), cv::Point(kWidth, cy), cv::Scalar(128, 128, 128), 1);

                if (count > 0) {
                  // Only show the targeting crosshair if we found the target
                  if (abs(degreetx) < 0.5) {
                    // Draw a red targeting crosshair if we are aligned
                    line(frontMat, cv::Point(midx-30, midy), cv::Point(midx-10, midy), cv::Scalar(0, 0, 255), 3);
                    line(frontMat, cv::Point(midx+10, midy), cv::Point(midx+30, midy), cv::Scalar(0, 0, 255), 3);
                    line(frontMat, cv::Point(midx, midy-30), cv::Point(midx, midy-10), cv::Scalar(0, 0, 255), 3);
                    line(frontMat, cv::Point(midx, midy+10), cv::Point(midx, midy+30), cv::Scalar(0, 0, 255), 3);
                  } else {
                    // Draw a green targeting crosshair if not aligned
                    line(frontMat, cv::Point(midx-30, midy), cv::Point(midx-10, midy), cv::Scalar(0, 255, 0), 3);
                    line(frontMat, cv::Point(midx+10, midy), cv::Point(midx+30, midy), cv::Scalar(0, 255, 0), 3);
                    line(frontMat, cv::Point(midx, midy-30), cv::Point(midx, midy-10), cv::Scalar(0, 255, 0), 3);
                    line(frontMat, cv::Point(midx, midy+10), cv::Point(midx, midy+30), cv::Scalar(0, 255, 0), 3);
                  }
                }
                FrontSvr.PutFrame(frontMat);
//                break;
//              case 1:
                // Show a black image with the detected lines drawn on it
                cv::inRange(frontMat, cv::Scalar(1, 1, 1), cv::Scalar(0, 0, 0), frontView);
                for (int i = 0; i < lines2.size(); i++){
                  if (match[i]){
                    double x1, y1, x2, y2;
                    x1 = lines2[i].x1;
                    y1 = lines2[i].y1;
                    x2 = lines2[i].x2;
                    y2 = lines2[i].y2;
                    line(frontView, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 1);
                  }
                }
                if (count > 0)
                  line(frontView, cv::Point(midx, midy+20), cv::Point(midx, midy-20), cv::Scalar(255, 255, 255), 1);
                DebugSvr.PutFrame(frontView);
                break;
	          }
            // Stream a zoomed-in part of the image
            frontSmall = frontMat(cv::Rect(kWidth/4, 0, kWidth/2, kHeight/2)); 
            FrontSmall.PutFrame(frontSmall);
          }

          //std::cout << GetMilliSpan(start) << "ms\n";

      }
    }).detach();
  }

  // Wait so that ports are allocated in the same order
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // start separate image processing threads for each camera if present
  if (cameras.size() >= 2) {

    // c.name = config.at("name").get<std::string>();
    // auto str = cameras.at("name").get<std::string>();
    // wpi::StringRef s(str);
    // if (s.equals_lower("back")) {

    std::thread([&] {
      // Control bandwidth by defining output resolution and camera frame rate divider
      const double kWidth = 320.0;
      const double kHeight = 240.0;
      const int kFrameRateDivider = 2;

      // Back facing drive camera. We just want to draw cross hairs on this.
      cs::CvSink BackCam = frc::CameraServer::GetInstance()->GetVideo(cameras[1]);
      // Setup a CvSource. This will send images back to the Dashboard
      cs::CvSource BackSvr =
        frc::CameraServer::GetInstance()->PutVideo("BackCam", kWidth, kHeight);
      // BackSvr.SetFPS(10); // This does not seem to work

      // Create mats to hold images
      cv::Mat backMat;
      cv::Mat backView;
      int counter = 0;

      while (true) {
        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (BackCam.GrabFrame(backMat) == 0) {
          // Send error to the output
          BackSvr.NotifyError(BackCam.GetError());
          // skip the rest of the current iteration
          continue;
        }

        // Skip frames (when counter is not 0) to reduce bandwidth
        counter = (counter + 1) % kFrameRateDivider;
        if (!counter) {
          // Scale the image (if needed) to reduce bandwidth
          cv::resize(backMat, backView, cv::Size(kWidth, kHeight), 0.0, 0.0, cv::INTER_AREA);
          // Give the output stream a new image to display
          BackSvr.PutFrame(backView);
        }
      }
    }).detach();
  }

  // loop forever
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(10));
}
