/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Constants.h"
#include "PolarizationCameraUtils.h"

#include <cmath>
#include <cstdint>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <csignal> // For signal handling

#include <opencv2/core.hpp>
#include<opencv2/core/core.hpp>

#include<System.h>
#include <string>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

// Global flag to indicate if Ctrl+C was pressed
volatile sig_atomic_t g_signal_received = 0;

void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "\nCtrl+C detected! Shutting down gracefully..." << std::endl;
        g_signal_received = 1; // Set the flag
    }
}

struct PoseStamped
{
    double timestamp;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

void SaveTrajectoryForAllFramesClayder(const string &filename, std::vector<PoseStamped> trajectory)
{
    cout << endl << "Saving Clayder frames trajectory to " << filename << " ..." << endl;

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f(filename.c_str());
    f.imbue(std::locale::classic());

    // CSV header
    // f << "ts (ns),tx (m),ty (m),tz (m),qx,qy,qz,qw\n";

    for (const PoseStamped& pose : trajectory)
    {
        double t = pose.timestamp;
        double x = pose.x;
        double y = pose.y;
        double z = pose.z;

        double qx = pose.qx;
        double qy = pose.qy;
        double qz = pose.qz;
        double qw = pose.qw;

        long long ts_ns = static_cast<long long>(std::round(t * 1e9));

        f << std::fixed << std::setprecision(9) << ts_ns << ','
          << std::scientific << std::setprecision(7)
          << static_cast<double>(x) << ','
          << static_cast<double>(y) << ','
          << static_cast<double>(z) << ','
          << static_cast<double>(qx) << ','
          << static_cast<double>(qy) << ','
          << static_cast<double>(qz) << ','
          << static_cast<double>(qw) << '\n';
    }

    f.close();
}

void SaveKeypointsForAllFramesClayder(const string &filename, std::vector<std::pair<double, uint16_t>> matchedKeypointsPerFrame, std::vector<std::pair<double, uint8_t>> isNewKeyFrameVector)
{
    cout << endl << "Saving Clayder keypoints trajectory to " << filename << " ..." << endl;

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f(filename.c_str());
    f.imbue(std::locale::classic());

    // CSV header
    // f << "ts (ns),tx (m),ty (m),tz (m),qx,qy,qz,qw\n";
    //
    std::cout<<matchedKeypointsPerFrame.size()<<' '<<isNewKeyFrameVector.size()<<std::endl;

    for (uint32_t i = 0; i < matchedKeypointsPerFrame.size(); i++)
    {
        double t = matchedKeypointsPerFrame[i].first;
        double t2 = isNewKeyFrameVector[i+1].first;

        // std::cout<<t<<' '<<t2<<std::endl;

        uint16_t numKeypointsPerFrame = matchedKeypointsPerFrame[i].second;
        uint8_t isNewKeyFrame = isNewKeyFrameVector[i+1].second;
        long long ts_ns = static_cast<long long>(std::round(t * 1e9));
        // long long ts_ns2 = static_cast<long long>(std::round(t2 * 1e9));

        // std::cout<<ts_ns<<' '<<ts_ns2<<std::endl;

        // if(ts_ns==ts_ns2) {

        f << std::fixed << std::setprecision(9) << ts_ns << ','
        << numKeypointsPerFrame
        << ',' << (uint16_t)(isNewKeyFrame) << '\n';
        // }
    }

    f.close();
}

// double computeWeight(double DoLP_mean, double DoLP_min = 0.26, double DoLP_max = 0.38) {
// double computeWeight(double DoLP_mean, double DoLP_min = 0.25, double DoLP_max = 0.385) {
// double computeWeight(double DoLP_mean, double DoLP_min = 0.22, double DoLP_max = 0.4) { //this works on 0835
double computeWeight(double DoLP_mean, double DoLP_min = 0.25, double DoLP_max = 0.42) {
// double computeWeight(double DoLP_mean, double DoLP_min = 0.27, double DoLP_max = 0.4) {
    double t = (DoLP_mean - DoLP_min) / (DoLP_max - DoLP_min);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t;
}

inline double clampVal(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// double f(double AoLP_mean, double AoLP_3stddev, double AoLP_max = M_PI / 2.0) {
//     double v = clampVal(std::abs(AoLP_mean) / AoLP_max, 0.0, 1.0);
//     double g = 1.0 - v;
//     return (1.0 - g) * AoLP_mean + g * AoLP_3stddev;
// }
//
// double f(double AoLP_mean, double AoLP_3stddev, double p, double AoLP_max = 60.0*M_PI/180.0) {
double f(double AoLP_mean, double AoLP_3stddev, double p, double AoLP_max) {
// double f(double AoLP_mean, double AoLP_3stddev, double p, double AoLP_max = M_PI / 2.0) {
// double f(double AoLP_mean, double AoLP_3stddev, double p, double AoLP_max = M_PI) {
    double v = clampVal(std::abs(AoLP_mean) / AoLP_max, 0.0, 1.0);

    // std::cout << "AoLP_mean: " << AoLP_mean*180.0/M_PI << " v: " << v << std::endl;

    double v_p = std::pow(v, p);
    // double v_p = v*p;
    double g = 1.0 - v_p;
    // double g = 1.0 - v_p;
    // return (1.0 - g) * AoLP_mean + g * AoLP_3stddev;
    return v_p * AoLP_mean + g * AoLP_3stddev;
}

double computeSmoothAngleTransition(double theta_curr, double theta_target, double max_theta_diff) {


    double theta_target0 = theta_target - M_PI; //180

    double diff0 = theta_target0 - theta_curr;
    double diff1 = theta_target - theta_curr;

    double diff = std::abs(diff0)<std::abs(diff1)?diff0:diff1;

    // double diff = theta_target - theta_curr;

    if (std::abs(diff)>=max_theta_diff) //saturate in 10
        return (diff<0)?theta_curr-max_theta_diff:theta_curr+max_theta_diff;
    else
        return theta_target;
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    //if(argc != 4)
    // if(argc != 6)
    // if(argc != 7)
    // {
    //     cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_imgs_cam0 path_to_imgs_cam1 path_to_keyframe_trajectory" << endl;
    //     return 1;
    // }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";


    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, ORB_SLAM3::Constants::POLCAM01, true);
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, ORB_SLAM3::Constants::POLCAM0, true);
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, ORB_SLAM3::Constants::POLCAM0, false);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_resize = 0.f;
    double t_track = 0.f;

    // Main loop
    cv::Mat imCam0;
    cv::Mat imCam1;
    cv::Mat polcam_img;
    cv::Mat Itheta0;
    cv::Mat Itheta1;

    cv::Mat imAux; //= cv::Mat::zeros(480, 640, CV_8U);
    int mState;

    std::vector<PoseStamped> trajectory;

    uint16_t imCounter = 0;

    double aolp_mean_curr = 0;
    double aolp_mean_prev = -999;
    double aolp_std = 0;
    double dolp_mean = 0;

    // int theta0_curr = 155;
    double theta0_curr = 900 * M_PI / 180.0;
    double theta0_prev = theta0_curr;
    double theta0_target = 0;
    // int theta1_curr = 98;
    double theta1_curr = 900 * M_PI / 180.0;

    double theta_3std_left;
    double theta_3std_right;

    double r = std::stof(argv[4]);
    double phi_circmax = std::stof(argv[5]) * M_PI/180.0;

    const uint FPS = 20;
    // const uint T_STEP = (uint)(1.0) * FPS ;
    const uint T_STEP = 0.5 * FPS ;
    // const uint8_t THETA_SHIFT_MIN = 5; //degrees
    const double THETA_SHIFT_MIN = 1.0*M_PI/180.0; //degrees
    // const double THETA_SHIFT_MIN = 3*M_PI/180.0; //degrees
    // const double DOLP_MIN = 0.21;
    // const double DOLP_MIN = 0.22; works for 0835
    // const double DOLP_MIN = 0.22;

    double DOLP_MIN = std::stof(argv[6]);
    // const double DOLP_MIN = 0.25; works for 0821
    // const double DOLP_MIN = 0.27;
    const double THETA_MAX_DIFF = 20.0*M_PI/180.0;
    const double I_ANGLE_VALUE = 60.0*M_PI/180.0;


    double theta0_curr_aux = 0;
    double theta1_curr_aux = 0;

    bool returnI = false;

    for(int ni=0; ni<nImages && !g_signal_received; ni++)
    // for(int ni=19; ni<nImages && !g_signal_received; ni++)
    // for(int ni=23; ni<nImages && !g_signal_received; ni++)
    //for(int ni=0; ni<1410 && !g_signal_received; ni++)
    {
        // Read image from file
        polcam_img = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        std::array<double, 3> aolp_dolp_stats = PolarizationCameraUtils::demosaicPolImageAndComputeStats(polcam_img, theta0_curr, theta1_curr, Itheta0, Itheta1, returnI);
        // std::array<double, 3> aolp_dolp_stats = PolarizationCameraUtils::demosaicPolImageAndComputeStats(polcam_img, 45*M_PI/180.0, 155*M_PI/180.0, Itheta0, Itheta1);

        aolp_mean_curr = aolp_dolp_stats[0];
        aolp_std = aolp_dolp_stats[1];
        dolp_mean = aolp_dolp_stats[2];

        if(ni%T_STEP==0) {
            if(dolp_mean>DOLP_MIN) { //each T_STEP seconds

                returnI = false;

                theta_3std_left = aolp_mean_curr-3.0*aolp_std;
                theta_3std_right = aolp_mean_curr+3.0*aolp_std;


                double aolp_mean_diff = aolp_mean_curr-aolp_mean_prev;

                if(std::abs(aolp_mean_diff)>=THETA_SHIFT_MIN) {
                    // theta0_curr_aux = f(aolp_mean_curr, theta_3std_right, 0.25); //sequences on 0803 were working with this value and pi/2 max in f()
                    // theta0_curr_aux = f(aolp_mean_curr, theta_3std_right, 0.3);
                    // theta0_curr_aux = f(aolp_mean_curr, theta_3std_right, 0.4);
                    // theta0_curr_aux = f(aolp_mean_curr, theta_3std_right, 0.7);
                    theta0_curr_aux = f(aolp_mean_curr, theta_3std_right, r, phi_circmax);
                    // theta0_curr_aux = f(aolp_mean_curr, theta_3std_right, 0.5); //all the sequences was working with this value and the exponential function
                    // theta0_curr_aux = f(aolp_mean_curr, theta_3std_right, 1.0);
                    // theta0_curr_aux = f(aolp_mean_curr, theta_3std_right, 1.5);
                    // theta0_curr_aux = 90.0*M_PI/180.0;

                    theta1_curr_aux = aolp_mean_curr + (M_PI/2.0);

                }
            }
            else {
                // theta0_curr_aux = 60*M_PI/180.0;
                theta0_curr_aux = I_ANGLE_VALUE;
                theta1_curr_aux = aolp_mean_curr + (M_PI/2.0);
                returnI = true;
                // std::cout<<"returnI=true"<<std::endl;
            }

            // if(ni!=0 && (std::abs(theta0_curr_aux-theta0_prev)>20.0*M_PI/180.0 && std::abs((theta0_curr_aux+M_PI)-theta0_prev)>20.0*M_PI/180.0) ) {
            if( ni!=0 && ( std::abs(theta0_curr_aux-theta0_prev)>THETA_MAX_DIFF && std::abs((theta0_curr_aux+M_PI)-theta0_prev)>THETA_MAX_DIFF ) ) {
                theta0_curr_aux = computeSmoothAngleTransition(theta0_prev, theta0_curr_aux, THETA_MAX_DIFF);
                // std::cout<<"Smooth transition"<<std::endl;
            }

            std::cout<<"dolp_mean: "<<dolp_mean<<" aolp_mean: "<<aolp_mean_curr * 180.0 / M_PI<<" theta_3std_right: "<<theta_3std_right * 180.0 / M_PI<<" theta0_prev: "<<theta0_prev * 180.0 / M_PI<<" theta0: "<<theta0_curr_aux * 180.0 / M_PI<<" theta1: "<<theta1_curr_aux * 180.0 / M_PI<<std::endl;

            theta0_curr = theta0_curr_aux;
            theta1_curr = theta1_curr_aux;
            aolp_mean_prev = aolp_mean_curr;
            theta0_prev = theta0_curr_aux;
        }

        imCam0 = Itheta0;
        imCam1 = Itheta1;

        double tframe = vTimestamps[ni];

        if(imCam0.empty() || imCam1.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
          //std::cout<<"HEY"<<std::endl;
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            //int width = im.cols * imageScale;
            //int height = im.rows * imageScale;
            //cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            //t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            //SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        Sophus::SE3f Tcw = SLAM.TrackMonocularPolcam(imCam0,imCam1,tframe);
        // Sophus::SE3f Tcw = SLAM.TrackMonocular(imCam0,tframe);
        Sophus::SE3f Twc = Tcw.inverse();

        // std::cout<<tframe<<" "<<Tcw.translation()<<std::endl;

        double tx = Twc.translation().x();
        double ty = Twc.translation().y();
        double tz = Twc.translation().z();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        trajectory.push_back({
            tframe,
            tx, ty, tz,
            q.x(), q.y(), q.z(), q.w()
        });


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];




        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    //cv::destroyAllWindows();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveTrajectoryTUM("KeyFrameTrajectory.txt");


    SLAM.SaveKeyFrameTrajectoryTUM(string(argv[7]));
    SaveTrajectoryForAllFramesClayder(string(argv[8]), trajectory);
    SaveKeypointsForAllFramesClayder(string(argv[9]), SLAM.mpTracker->matchedKeypointsPerFrame, SLAM.mpTracker->isNewKeyFrameVector);

    // SLAM.SaveKeyFrameTrajectoryTUM(string(argv[8]));
    // SaveTrajectoryForAllFramesClayder(string(argv[9]), trajectory);
    // SaveKeypointsForAllFramesClayder(string(argv[10]), SLAM.mpTracker->matchedKeypointsPerFrame, SLAM.mpTracker->isNewKeyFrameVector);
    //
    // SLAM.SaveKeyFrameTrajectoryTUM(string(argv[5]));
    // SaveTrajectoryForAllFramesClayder(string(argv[6]), trajectory);
    // SaveKeypointsForAllFramesClayder(string(argv[7]), SLAM.mpTracker->matchedKeypointsPerFrame, SLAM.mpTracker->isNewKeyFrameVector);

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
