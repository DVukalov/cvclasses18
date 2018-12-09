/* Demo application for Computer Vision Library.
* @file
* @date 2018-11-25
* @author Dmitrij Vukalov
*/

#include <cvlib.hpp>
#include <opencv2/opencv.hpp>

#include "utils.hpp"
struct img_features
{
    cv::Mat img;
    std::vector<cv::KeyPoint> corners;
    cv::Mat descriptors;
};


int demo_image_stitching(int argc, char* argv[])
{
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
        return -1;

    const auto main_wnd = "orig";
    const auto demo_wnd = "demo";

    cv::namedWindow(main_wnd);
    cv::namedWindow(demo_wnd);

    auto detector = cvlib::corner_detector_fast::create();
    auto descriptor = cv::ORB::create();
    auto matcher = cv::BFMatcher();
    matcher.create();

    auto stitcher = cvlib::Stitcher();

    //auto detector = cvlib::corner_detector_fast::create();
    //auto matcher = cvlib::descriptor_matcher();
    //auto stitcher = cvlib::Stitcher();

    int detector_threshold = 30;
    int ratio = 12;
    int max_distance = 99;


    detector->setVarThreshold(detector_threshold);
    cv::createTrackbar(
        "Threshold", main_wnd, &detector_threshold, 255,
        [](int threshold, void* detector) { ((cvlib::corner_detector_fast*)detector)->setVarThreshold(threshold); }, (void*)detector);
    
    //matcher.set_ratio(ratio / 10.0f);
    //cv::createTrackbar(
    //    "Ratio * 10", main_wnd, &ratio, 20, [](int ratio, void* matcher) { ((cvlib::descriptor_matcher*)matcher)->set_ratio(ratio / 10.0f); },
    //    (void*)&matcher);

    //cv::createTrackbar("ratio SSD", demo_wnd, &ratio, 50, on_ratio_changed, (void*)&matcher);

    cv::createTrackbar("MaxDist", main_wnd, &max_distance, 500);

    img_features frame1, frame2;

   /* cv::Mat refImg, testImg;
    std::vector<cv::KeyPoint> refCorners, testCorners;
    cv::Mat refDescriptors, testDescriptors;*/
    std::vector<std::vector<cv::DMatch>> pairs;

    cv::Mat main_frame;
    cv::Mat demo_frame;
    cv::Mat test_frame;
    utils::fps_counter fps;
    int pressed_key = 0;
    bool needToInit = true;

    while (pressed_key != 27) // ESC
    {
        cap >> frame1.img;
        frame1.img.copyTo(main_frame);

        //cv::drawKeypoints(frame1.img, frame1.corners, main_frame, cv::Scalar(0, 0, 255));
        //frame1.img.copyTo(main_frame);
        utils::put_fps_text(main_frame, fps);
        cv::imshow(main_wnd, main_frame);

        pressed_key = cv::waitKey(30);
        if (pressed_key == ' ') // space
        {
            if (needToInit)
            {
                frame2.img = frame1.img.clone();
                //frame1.img.copyTo(frame2.img);
                detector->detect(frame2.img, frame2.corners);
                descriptor->compute(frame2.img, frame2.corners, frame2.descriptors);
                needToInit = false;
            }
            else
            {
                //stitcher.stitch(frame1.corners, frame2.corners, frame1.descriptors, frame2.descriptors, frame2.img);
            }
        }

        if (frame2.corners.empty())
        {
            continue;
        }
        /*if (refCorners.empty())
            continue;*/

        detector->detect(frame1.img, frame1.corners);

        descriptor->compute(frame1.img, frame1.corners, frame1.descriptors);
        
        matcher.radiusMatch(frame1.descriptors, frame2.descriptors, pairs, (float)max_distance + 1.0f);
        cv::drawMatches(frame1.img, frame1.corners, frame2.img, frame2.corners, pairs, demo_frame);

        //stitcher.stichedImage(frame1.img, frame1.corners, frame2.img, frame2.corners, pairs, test_frame);

        cv::imshow(demo_wnd, demo_frame);
    }

    cv::destroyWindow(main_wnd);
    cv::destroyWindow(demo_wnd);

    return 0;
}
