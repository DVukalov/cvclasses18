/* Image stitcher algorithm implementation.
 * @file
 * @date 2018-12-05
 * @author Anonymous
 */

#include "cvlib.hpp"
#include <iostream>

namespace cvlib
{
void Stitcher::stichedImage(const cv::Mat& frame1, const std::vector<cv::KeyPoint>& frame1Corners, const cv::Mat& frame2,
                            const std::vector<cv::KeyPoint>& frame2Corners, const std::vector<std::vector<cv::DMatch>>& pairs, cv::Mat& stitchedImg)
{
    m_isStitched = false;
    std::vector<cv::Point2f> point1, point2;

    for (auto i = 0; i < pairs.size(); i++)
    {
        if (pairs[i].size() > 0)
        {
            point1.push_back(frame1Corners[pairs[i][0].queryIdx].pt);
            point2.push_back(frame2Corners[pairs[i][0].trainIdx].pt);
        }
    }

    H = cv::findHomography(point2, point1, cv::RANSAC);

    std::cout << "H:\n";
    for (int i = 0; i < H.size().width; i++)
    {
        for (int j = 0; j < H.size().height; j++)
        {
            std::cout << (double)H.at<double>(i, j) << "\t";
        }
        std::cout << "\n";
    }

    if (!H.empty())
    {
        // смещение по осям
        c = H.at<double>(0, 2);
        f = H.at<double>(1, 2);

        if (c < 0 && f < 0)
             H = H.inv();

        // // найдем прямоугольник
        // cv::Rect frameRect = cv::boundingRect(point1);
        //
        // // выберем 4 точки, которые являются вершинами данного прямоугольника
        // std::vector<cv::Point2f> rectCorners(4);
        // rectCorners.push_back(cv::Point2f(frameRect.x, frameRect.y));
        // rectCorners.push_back(cv::Point2f(frameRect.x + frameRect.width, frameRect.y));
        // rectCorners.push_back(cv::Point2f(frameRect.x + frameRect.width, frameRect.y + frameRect.height));
        // rectCorners.push_back(cv::Point2f(frameRect.x, frameRect.y + frameRect.height));

        // // вычисляем гомографию 4x точек
        // std::vector<cv::Point2f> homographyCorners(4);
        // cv::perspectiveTransform(rectCorners, homographyCorners, H);

        // получаем размеры исходного изображения

       cv::Size size = frame2.size();
        if (c >= 0 && f >= 0)
        {
            size.width += c;
            size.height += f;

            cv::warpPerspective(frame2, m_stitching_image, H, size);
            frame1.copyTo(m_stitching_image.rowRange(0, frame1.rows).colRange(0, frame1.cols));

            //cv::warpPerspective(frame1, m_stitching_image, H, size);
            //frame2.copyTo(m_stitching_image.rowRange(0, frame2.rows).colRange(0, frame2.cols));
            m_isStitched = true;
        }

        if (c < 0 && f < 0)
        {
            size.width -= c;
            size.height -= f;
            cv::warpPerspective(frame1, m_stitching_image, H, size);
            //cv::vconcat(frame2Descriptors, frame1Descriptors, frame2Descriptors);
            frame2.copyTo(m_stitching_image.rowRange(0, frame2.rows).colRange(0, frame2.cols));
            m_isStitched = true;
        }
        // stitchedImg.copyTo(stitchedImg);

        if (m_isStitched)
        {
            m_stitching_image.copyTo(stitchedImg);
            // stitch(frame1Corners, frame2Corners, frame1Descriptors, frame2Descriptors, frame2.im);
        }

        else
            frame2.copyTo(stitchedImg);
        // вычисляем гомографию
    }

    // stitchedImg = frame1.clone();
}

void Stitcher::stitch(std::vector<cv::KeyPoint> frame1Corners, std::vector<cv::KeyPoint>& frame2Corners, const cv::Mat& frame1Descriptors,
                      cv::Mat& frame2Descriptors, cv::Mat& frame2)
{
    if (m_isStitched)
    {
        std::vector<cv::KeyPoint> transformCorners;
        if (c >= 0 && f >= 0)
        {
            transformKeyPoints(frame2Corners);
        }
        else if (c < 0 && f < 0)
        {
            transformKeyPoints(frame1Corners);
        }

        frame2Corners.insert(frame2Corners.end(), frame1Corners.begin(), frame1Corners.end());
        cv::vconcat(frame2Descriptors, frame1Descriptors, frame2Descriptors);
        m_stitching_image.copyTo(frame2);
    }
}

void Stitcher::transformKeyPoints(std::vector<cv::KeyPoint>& keypoints)
{
    cv::Mat_<double> P(3, 1);
    for (int i = 0; i < keypoints.size(); i++)
    {
        P(0, 0) = keypoints[i].pt.x;
        P(1, 0) = keypoints[i].pt.y;
        P(2, 0) = 1.0;

        P = H * P;
        // cv::perspectiveTransform(P, P, H);
        // m_homography * P;

        keypoints[i].pt.x = P(0, 0) / P(2, 0);
        keypoints[i].pt.y = P(1, 0) / P(2, 0);
    }
}
/*
void Stitcher::apply(const cv::Mat& src, cv::Mat& dst)
{
    std::cout << "BEGIN\n";

    if (src.empty()) return;
    if (mPanoram.empty())
    {
        mPanoram = src.clone();
        dst = mPanoram.clone();
        return;
    }
    

















































    std::cout << "GOT PANORAM\n";

    auto detector = cvlib::corner_detector_fast::create();
    auto extractor = cv::ORB::create();
    cv::BFMatcher matcher(cv::NORM_L2, false);

    std::vector<cv::KeyPoint> cornersSrc;
    std::vector<cv::KeyPoint> cornersPanoram;
    cv::Mat descriptorsSrc;
    cv::Mat descriptorsPanoram;
    std::vector<cv::DMatch> matches;
    

















































    std::cout << "INIT\n";

    detector->detect(src, cornersSrc);
    detector->detect(mPanoram, cornersPanoram);
    

















































    std::cout << "DETECT\n";
    

















































    extractor->compute(src, cornersSrc, descriptorsSrc);
    extractor->compute(mPanoram, cornersPanoram, descriptorsPanoram);
    

















































    std::cout << "COMPUTE\n";

    matcher.match(descriptorsSrc, descriptorsPanoram, matches);

    std::cout << "MATCH: " << matches.size() <<"\n";
    

















































    double maxDist = 0; double minDist = 100;
    for (int i = 0; i < matches.size(); i++)
    {
        double dist = matches[i].distance;
        if (dist < minDist) minDist = dist;
        if (dist > maxDist) maxDist = dist;
    }
    

















































    if (minDist < 1) minDist = 1;

    std::cout << "MIN DIST: " << minDist <<"\n";
    std::cout << "MAX DIST: " << maxDist <<"\n";

    std::vector<cv::DMatch> goodMatches;
    for(int i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance <= 3*minDist)
            goodMatches.push_back(matches[i]);
    }

    std::cout << "GOOD MATCHES: " << goodMatches.size() << std::endl;
    

















































    std::vector<cv::Point2f> frame;
    std::vector<cv::Point2f> scene;

    for( int i = 0; i < goodMatches.size(); i++ )
    {
        frame.push_back(cornersSrc[goodMatches[i].queryIdx].pt);
        scene.push_back(cornersPanoram[goodMatches[i].trainIdx].pt);
    }

    cv::Rect frameRect = cv::boundingRect(frame);
    cv::Mat H = cv::findHomography(frame, scene, cv::RANSAC);
    

















































    std::cout << "H:\n";
    for (int i = 0; i < H.size().width; i++)
    {
        for (int j = 0; j < H.size().height; j++)
        {
            std::cout << (double)H.at<double>(i,j) << "\t";
        }
        std::cout << "\n";
    }

    std::vector<cv::Point2f> frameCorners(4);
    

















































    frameCorners[0] = cv::Point2f(frameRect.x, frameRect.y);
    frameCorners[1] = cv::Point2f(frameRect.x + frameRect.width, frameRect.y);
    frameCorners[2] = cv::Point2f(frameRect.x + frameRect.width, frameRect.y + frameRect.height);
    frameCorners[3] = cv::Point2f(frameRect.x, frameRect.y + frameRect.height);
    

















































    std::vector<cv::Point2f> sceneCorners(4);
    cv::perspectiveTransform(frameCorners, sceneCorners, H);
    

















































    std::cout << "OBJECT:\n";
    std::cout << frameCorners[0].x << " " << frameCorners[0].y << "\n";
    std::cout << frameCorners[1].x << " " << frameCorners[1].y << "\n";
    std::cout << frameCorners[2].x << " " << frameCorners[2].y << "\n";
    std::cout << frameCorners[3].x << " " << frameCorners[3].y << "\n";
    std::cout << "SCENE:\n";
    std::cout << sceneCorners[0].x << " " << sceneCorners[0].y << "\n";
    std::cout << sceneCorners[1].x << " " << sceneCorners[1].y << "\n";
    std::cout << sceneCorners[2].x << " " << sceneCorners[2].y << "\n";
    std::cout << sceneCorners[3].x << " " << sceneCorners[3].y << "\n";
    

















































    int dw = -(int)H.at<double>(0,2);
    int dh = -(int)H.at<double>(1,2);
    

















































    cv::Rect sceneRect(sceneCorners[0], sceneCorners[2]);
    int saW = 0; // shared area Width
    int saH = 0; // shared area Height

    if (dw>0) // scene to the right
        saW = sceneRect.x + sceneRect.width/2 + src.cols - (frameRect.x + frameRect.width/2);
    else if (dw<0)// scene to the left
        saW = frameRect.x + frameRect.width/2 + mPanoram.cols - (sceneRect.x + sceneRect.width/2);
    else
        saW = std::min(mPanoram.cols, src.cols);

    if (dh>0) // scene to the down
        saH = sceneRect.y + sceneRect.height/2 + src.rows - (frameRect.y + frameRect.height/2);
    else if (dh<0)// scene to the up
        saH = frameRect.y + frameRect.height/2 + mPanoram.rows - (sceneRect.y + sceneRect.height/2);
    else
        saH = std::min(mPanoram.rows, src.rows);

    int width = mPanoram.cols + src.cols - saW;
    int height = mPanoram.rows + src.rows - saH;

    cv::Mat result(height, width, mPanoram.type(), cv::Scalar::all(0));

    cv::Rect sceneRoi( (dw>0)*(result.cols - mPanoram.cols), (dh>0)*(result.rows - mPanoram.rows), mPanoram.cols, mPanoram.rows);
    mPanoram.copyTo(result(sceneRoi));

    cv::Rect frameRoi( (dw<0)*(result.cols - src.cols), (dh<0)*(result.rows - src.rows), src.cols, src.rows);
    src.copyTo(result(frameRoi));
    

















































    int sharedRoiX = (dw>0) ? sceneRoi.x : frameRoi.x;
    int sharedRoiY = (dh>0) ? sceneRoi.y : frameRoi.y;
    cv::Rect sharedRoi(sharedRoiX, sharedRoiY, saW, saH);
    

















































    int sharedRoiSceneX = (dw<0) ? mPanoram.cols - saW : 0;
    int sharedRoiSceneY = (dh<0) ? mPanoram.rows - saH : 0;
    cv::Rect sharedRoiScene(sharedRoiSceneX, sharedRoiSceneY, saW, saH);
    

















































    int sharedRoiFrameX = (dw>0) ? src.cols - saW : 0;
    int sharedRoiFrameY = (dh>0) ? src.rows - saH : 0;
    cv::Rect sharedRoiFrame(sharedRoiFrameX, sharedRoiFrameY, saW, saH);
    

















































    cv::Mat sharedArea = mPanoram(sharedRoiScene).clone()/2;
    sharedArea += src(sharedRoiFrame).clone()/2;
    sharedArea.copyTo(result(sharedRoi));

    cv::resize(result, result, cv::Size(1280, 720));
    cv::imshow("Panoram", result);
    

















































    // DEMO
    cv::Mat imgMatches;
    cv::drawMatches( src, cornersSrc, mPanoram, cornersPanoram,
               goodMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::line( imgMatches, frameCorners[0], frameCorners[1], cv::Scalar( 255, 255, 0), 4 );
    cv::line( imgMatches, frameCorners[1], frameCorners[2], cv::Scalar( 255, 255, 0), 4 );
    cv::line( imgMatches, frameCorners[2], frameCorners[3], cv::Scalar( 255, 255, 0), 4 );
    cv::line( imgMatches, frameCorners[3], frameCorners[0], cv::Scalar( 255, 255, 0), 4 );
    

















































    cv::line( imgMatches, sceneCorners[0] + cv::Point2f( src.cols, 0), sceneCorners[1] + cv::Point2f( src.cols, 0), cv::Scalar( 0, 255, 0), 4 );
    cv::line( imgMatches, sceneCorners[1] + cv::Point2f( src.cols, 0), sceneCorners[2] + cv::Point2f( src.cols, 0), cv::Scalar( 0, 255, 0), 4 );
    cv::line( imgMatches, sceneCorners[2] + cv::Point2f( src.cols, 0), sceneCorners[3] + cv::Point2f( src.cols, 0), cv::Scalar( 0, 255, 0), 4 );
    cv::line( imgMatches, sceneCorners[3] + cv::Point2f( src.cols, 0), sceneCorners[0] + cv::Point2f( src.cols, 0), cv::Scalar( 0, 255, 0), 4 );

    cv::resize(imgMatches, imgMatches, cv::Size(1280, 720));
    cv::imshow( "Good Matches & Object detection", imgMatches );
}
*/
} // namespace cvlib
