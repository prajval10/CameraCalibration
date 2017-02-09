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

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::stringstream;
using cv::waitKey;
using namespace covis;

int main()
{   /*----------------------------------------------------------------------------------------
    / load calibration images from hard drive
    / The file location here needs to be modified
    /*----------------------------------------------------------------------------------------*/

    const string base = "/home/prajval10/Desktop/LABS/covis/Lab2/images/";
    const string prefix = "img";

    // init empty vector of detected patterns
    vector<Pattern> patterns;

    /*-------------------------------------------------------------------------------
     *  The program loads all 11 images and tries to find all 36 dots (dots in 6 by 6 pattern
     *  in each frame. Now we have the 2D coordinates (x and y) of 396 dots,
     *  which corresponds to 792 scalars.
     * --------------------------------------------------------------------------*/
    patterns.clear();
    patterns.reserve(36);

    GridTracker tracker;          // this tracker detects a 6x6 grid of points
    //CBTracker tracker(8,6);     // this one is to be given the chessboard dimension (8x6)

    // read images while the corresponding file exists
    // images are displayed to ensure the detection was performed
    while(true)
    {
        stringstream ss;
        ss << prefix << patterns.size() << ".jpg";
        std::ifstream testfile(base + ss.str());
        if(testfile.good())
        {
            testfile.close();
            Pattern pat;
            pat.im =  cv::imread(base + ss.str());

            //Detect points from the images
            tracker.detect(pat.im, pat.point);
            // Display the results
            pat.window = ss.str();
            // draw extraction results
            drawSeq(pat.window, pat.im, pat.point);
            //store the pat in vector of <Pattern> patterns
            patterns.push_back(pat);
            waitKey(0);
        }
        else
            break;
    }
    cout << "Found " << patterns.size() << " images" << endl;

    /* create a camera model (Perspective or Distortion)
    * default parameters should be guessed from image dimensions
    * PerspectiveCamera cam(1,1,1,1);   // not a very good guess
    * The calibration can end in a local but not global minimum, resulting in a totally wrong result. As can
    * be seen, the frame cannot be seen in the window, meaning it is either computed too big or too small.
    */

    // camera model with default parameters ( in calibration . cpp )
    const double pxy = 0.5*( patterns[0].im.rows + patterns[0].im.cols );
    PerspectiveCamera cam ( pxy , pxy , 0.5*patterns[0].im.cols , 0.5*patterns[0].im.rows);

    // initiate virtual visual servoing with inter-point distance and pattern dimensions
    VVS vvs(cam, 0.03, 6, 6);

    // calibrate from all images
    vvs.calibrate(patterns);

    // print results
    cout << "Final calibration: " << cam.xi_.t() << endl;

    /*----------------------------------------------------------------------------
     * The calibration result is computed and shown on each frame. The initial 11 figures shows the pattern
       detected by tracker.detect, and the final figure shows the calculated axises of the coordinate of the
       pattern (red for X positive, green for Y positive and blue for Z positive), and the red frame shows the
       range of the pattern.
     * --------------------------------------------------------------------------*/
    // this will wait for a key pressed to stop the program
    waitKey(0);
}
