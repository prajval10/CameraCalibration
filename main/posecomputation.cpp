#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpIoTools.h>
#include <fstream>
#include <opencv2/calib3d/calib3d.hpp>
#include <vvs.h>
#include <grid_tracker.h>
#include <perspective_camera.h>
#include <distortion_camera.h>
#include <cb_tracker.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"  //-- Contains structures and classes for holding and manipulating images
#include "opencv2/highgui/highgui.hpp" //-- Contains functions for displaying images on screen
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::stringstream;
using cv::waitKey;
using namespace covis;

int main(int argc, char** argv)
{
    vpHomogeneousMatrix M_;
    cv::Mat frame;

    if(argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " <video_file> " << std::endl;
        return -1;
    }

    //grab first frame from video for initialisation
    const std::string filename(argv[1]);
    cv::VideoCapture capture(filename);
    capture.grab();
    if(!capture.retrieve(frame,0))
    {
        std::cout << "No image grabbed. Terminating capture." << std::endl;
        return -1;
    }
    // Use Intrinsic paramters found in calibration.cpp as initialisation of cam.
    PerspectiveCamera cam (1247.604396, 1241.197965, 319.211406, 245.5879185);

    // init empty vector of detected patterns
    Pattern pat;
    pat.im =  frame.clone();

    //We give distance and size of pattern to VVS function
    VVS vvs(cam, 0.03, 6, 6);

    GridTracker tracker;          // this tracker detects a 6x6 grid of points
    //CBTracker tracker(8,6);     // this one is to be given the chessboard dimension (8x6)

    // initiate virtual visual servoing with inter-point distance and pattern dimensions
    tracker.detect(pat.im,pat.point);
    vvs.computePose(pat,M_,true );
    // read images while the corresponding file exists
    // images are displayed to ensure the detection was performed
    while(true)
    {
        //Capture frames from video one-by-one
        cv::Mat frame1;
        capture.grab();
        if(!capture.retrieve(frame1,0))
        {
            std::cout << "No image grabbed. Terminating capture." << std::endl;
            break;
        }
        pat.im =frame1.clone();

        //We use track instead of detect method here since the changes between two frames in the video is small
        tracker.track(pat.im, pat.point);

        //call ComputePose method in vvs.cpp to compute the extrinsic params M_
        vvs.computePose(pat,M_ ,false );

        // draw extraction results
        drawSeq(pat.window, pat.im, pat.point);

        std::cout<<"pose:\n"<<M_<<std::endl;
        std::cout<<"-------------------------------------------------------"<<std::endl;

        if(cv::waitKey(33) >= 0) break;
    }

    // this will wait for a key pressed to stop the program
    waitKey(0);
}
