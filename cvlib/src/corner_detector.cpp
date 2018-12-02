
/* FAST corner detector algorithm implementation.
 * @file
 * @date 2018-10-16
 * @author Dmitrij Vukalov
 */

#include "cvlib.hpp"
#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>

namespace cvlib
{
// static
cv::Ptr<corner_detector_fast> corner_detector_fast::create()
{
    return cv::makePtr<corner_detector_fast>();
}

void corner_detector_fast::detect(cv::InputArray image, CV_OUT std::vector<cv::KeyPoint>& keypoints, cv::InputArray /*mask = cv::noArray()*/)
{
    keypoints.clear();
    m_num_point = 0;
    cv::Size image_size = image.size();
    cv::Mat imag = image.getMat();

    if (imag.channels() == 3)
        cv::cvtColor(imag, imag, cv::COLOR_BGR2GRAY);

    std::vector<int> etalon_1(12);
    std::vector<int> etalon_2(12);
    std::fill(etalon_1.begin(), etalon_1.end(), 1);
    std::fill(etalon_2.begin(), etalon_2.end(), 2);

    int i_c, i_plus_thresh, i_minus_thresh;

    for (auto i = 3; i < image_size.height - 3; i++)
        for (auto j = 3; j < image_size.width - 3; j++)
        {
            i_c = imag.at<unsigned char>(i, j);
            i_plus_thresh = i_c + m_threshold;
            i_minus_thresh = i_c - m_threshold;

            for (auto k = 0; k < 16; k++)
                circle_points.push_back(imag.at<unsigned char>(i + offset_i[k], j + offset_j[k]));

            int light = 0, dark = 0; // проверяем точки 1, 5, 9, 13
            for (auto l = circle_points.begin(); l != circle_points.end(); l += 4)
            {
                light = *l > i_plus_thresh ? ++light : light;
                dark = *l < i_minus_thresh ? ++dark : dark;
            }

            if (light > 2 || dark > 2)
            {
                copyVector();
                std::transform(cyclic_buffer.begin(), cyclic_buffer.end(), cyclic_buffer.begin(),
                               [=](int n) { return (n < i_minus_thresh) + 2 * (n > i_plus_thresh); });

                auto it_dark = std::search(cyclic_buffer.begin(), cyclic_buffer.end(), etalon_1.begin(), etalon_1.end());
                auto it_ligth = std::search(cyclic_buffer.begin(), cyclic_buffer.end(), etalon_2.begin(), etalon_2.end());

                if (it_dark != cyclic_buffer.end() || it_ligth != cyclic_buffer.end())
                {
                    m_num_point++;
                    keypoints.push_back(cv::KeyPoint(cv::Point2f(double(j), double(i)), 10, float(0)));
                }
                cyclic_buffer.clear();
            }
            circle_points.clear();
        }
}
void corner_detector_fast::compute(cv::InputArray image, std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors)
{
    const int desc_length = 15 + 3;
    descriptors.create(static_cast<int>(keypoints.size()), desc_length, CV_32S);

    auto desc_mat = descriptors.getMat();
    desc_mat.setTo(0);

    cv::Mat image_mat = image.getMat();
    if (image_mat.channels() == 3)
        cv::cvtColor(image_mat, image_mat, cv::COLOR_BGR2GRAY);

    int* ptr = reinterpret_cast<int*>(desc_mat.ptr());

    for (const auto& pt : keypoints)
    {
        int x = (int)pt.pt.x;
        int y = (int)pt.pt.y;

        int directions[3];
        for (auto j = 1; j < 4; j++)
        {
            int radius_area = j;
            cv::Mat region = image_mat(cv::Range(y - radius_area, y + radius_area + 1), cv::Range(x - radius_area, x + radius_area + 1));

            double min, max;
            cv::minMaxLoc(region, &min, &max);

            cv::Mat mmean, mstd;
            cv::meanStdDev(region, mmean, mstd);
            float mean = (float)mmean.at<double>(0);
            float std = (float)mstd.at<double>(0);

            // берем точки из основных направлений
            std::vector<int> square;
            square.push_back((int)region.at<unsigned char>(0, 0));
            square.push_back((int)region.at<unsigned char>(0, radius_area));
            square.push_back((int)region.at<unsigned char>(0, 2 * radius_area));
            square.push_back((int)region.at<unsigned char>(radius_area, 2 * radius_area));
            square.push_back((int)region.at<unsigned char>(2 * radius_area, 2 * radius_area));
            square.push_back((int)region.at<unsigned char>(2 * radius_area, radius_area));
            square.push_back((int)region.at<unsigned char>(2 * radius_area, 0));
            square.push_back((int)region.at<unsigned char>(radius_area, 0));

            // находим основное направление
            int main_diff = 0;
            int center = (int)region.at<unsigned char>(radius_area, radius_area);

            for (auto i = 0; i < 8; i++)
            {
                int diff = std::abs(center - square.at(i));
                if (main_diff < diff)
                {
                    main_diff = diff;
                    directions[j - 1] = 1000 * i;
                }
            }

            *ptr = (directions[j - 1]);
            ptr++;

            *ptr = int(mean / max * 1e4);
            ptr++;
            *ptr = int(std / max * 1e4);
            ptr++;
            *ptr = (int)max;
            ptr++;
            *ptr = (int)min;
            ptr++;

            // находим основные моменты и HuMoments
            cv::Moments mom = cv::moments(region);
            double hu[7];
            cv::HuMoments(mom, hu);
            *ptr = int(hu[0] * 100000);
            ptr++;
            square.clear();
        }
    }
}

void corner_detector_fast::detectAndCompute(cv::InputArray image, cv::InputArray, std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors,
                                            bool /*= false*/)
{
    setVarThreshold(20);
    detect(image, keypoints);
    compute(image, keypoints, descriptors);
}
} // namespace cvlib
