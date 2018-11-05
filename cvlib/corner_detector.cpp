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
    //                  1    2   3   4  5  6  7  8  9  10 11 12 13 14  15  16
    int offset_i[16] = {-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3};
    int offset_j[16] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};

    std::vector<int> circle_points(16);
    std::vector<int> etalon(12);

    std::vector<int> cyclic_buffer_dark(27);
    std::vector<int> cyclic_buffer_ligth(27);

    std::fill(etalon.begin(), etalon.end(), 1);

    int i_c, i_plus_thresh, i_minus_thresh;

    for (auto i = 3; i < image_size.height - 3; i++)
    {
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
                std::copy(circle_points.begin(), circle_points.begin() + 16, cyclic_buffer_dark.begin());
                std::copy(circle_points.begin(), circle_points.begin() + 11, cyclic_buffer_dark.begin() + 16);

                //std::copy(circle_points.begin(), circle_points.begin() + 16, cyclic_buffer_dark.begin() + 16);
                std::copy(cyclic_buffer_dark.begin(), cyclic_buffer_dark.begin()+27, cyclic_buffer_ligth.begin());

                /*std::copy(cyclic_buffer_dark.begin(), cyclic_buffer_dark.begin()+27,
                    std::ostream_iterator<int>(std::cout, " "));
                std::cout << std::endl;
*/
                //std::copy(cyclic_buffer_ligth.begin(), cyclic_buffer_ligth.end(),
                //    std::ostream_iterator<int>(std::cout, " "));
                //std::cout << std::endl;
                //std::cout << "i_minus_thresh" << i_minus_thresh << std::endl;

                std::transform(cyclic_buffer_dark.begin(), cyclic_buffer_dark.end(), cyclic_buffer_dark.begin(), [=](int n) { return (n < i_minus_thresh); });
                std::transform(cyclic_buffer_ligth.begin(), cyclic_buffer_ligth.end(), cyclic_buffer_ligth.begin(), [=](int n) { return (n > i_plus_thresh); });

                //std::copy(cyclic_buffer_dark.begin(), cyclic_buffer_dark.end()-1,
                //    std::ostream_iterator<int>(std::cout, " "));
                //std::cout << std::endl;
                //std::cout << std::endl;

                auto it_dark = std::search(cyclic_buffer_dark.begin(), cyclic_buffer_dark.end(), etalon.begin(), etalon.end());
                auto it_ligth = std::search(cyclic_buffer_ligth.begin(), cyclic_buffer_ligth.end(), etalon.begin(), etalon.end());
                if (it_dark != cyclic_buffer_dark.begin()+27 || it_dark != cyclic_buffer_ligth.begin() + 27)
                {
                    m_num_point++;
                    keypoints.push_back(cv::KeyPoint(cv::Point2f(double(j), double(i)), 10, float(0)));
                }
                    //std::cout << "needle2 found at position " << std::endl;
                //std::copy(cyclic_buffer_ligth.begin(), cyclic_buffer_ligth.end(),
                //    std::ostream_iterator<int>(std::cout, " "));
                //std::cout << std::endl;


                cyclic_buffer_ligth.clear();
                cyclic_buffer_dark.clear();
                //auto first_element_not_dark = std::find_if(circle_points.begin(), circle_points.end(), [=](int n) { return (n >= i_minus_thresh); });
                //auto first_element_not_ligth = std::find_if(circle_points.begin(), circle_points.end(), [=](int n) { return (n <= i_plus_thresh); });

                //std::reverse(circle_points.begin(), circle_points.end());

                //auto last_element_not_dark = std::find_if(circle_points.begin(), circle_points.end(), [=](int n) { return (n >= i_minus_thresh); });
                //auto last_element_not_ligth = std::find_if(circle_points.begin(), circle_points.end(), [=](int n) { return (n <= i_plus_thresh); });

                ////std::cout << "abs" << abs(first_element_not_dark - last_element_not_dark) << std::endl;
                //std::reverse(circle_points.begin(), circle_points.end());
                //std::copy(circle_points.begin(), circle_points.end(),
                //    std::ostream_iterator<int>(std::cout, " "));
                //std::cout << std::endl;

                //int a = first_element_not_dark - circle_points.begin();
                //int b = last_element_not_dark - circle_points.begin();
                //std::cout << "i_minus_thresh" << i_minus_thresh << std::endl;
                //std::cout << "First" << first_element_not_dark - circle_points.begin() << std::endl;
                //std::cout << "Last" << last_element_not_dark  - circle_points.begin() << std::endl;
                //std::cout << "abs" << abs(a- b) << std::endl;

                // ищем первую не подходящуюю, если она меньше 12, то стартуем с нее иначе уже нашли
                //if (abs(first_element_not_dark - 16 + last_element_not_dark)) < 3 ||
                //    (abs(first_element_not_ligth - 16 + last_element_not_ligth) < 3))
                //{
                //    m_num_point++;
                //    keypoints.push_back(cv::KeyPoint(cv::Point2f(double(j), double(i)), 10, float(0)));
                //}
            }
                /*std::copy(circle_points.begin(), circle_points.end(),
                std::ostream_iterator<int>(std::cout, " "));
                std::cout << std::endl;

                std::cout << "i_minus_thresh" << i_minus_thresh << std::endl;
                std::cout << "First" << first_element_not_dark - circle_points.begin() << std::endl;
                std::cout << "Last" << last_element_not_dark - circle_points.begin() << std::endl;
                */    //std::find_if(circle_points.begin(), circle_points.begin(), [=](int) { return n == i_minus_thresh; });

              /*  std::vector<int> cyclic_buffer(27);
                std::copy(circle_points.begin(), circle_points.end(), cyclic_buffer.begin());
                std::copy(circle_points.begin(), circle_points.begin() + 11, cyclic_buffer.begin() + 16);

                for (auto h = cyclic_buffer.begin(); h != cyclic_buffer.begin() + 15; h++)
                {
                    if (std::count_if(h, h + 12, [=](int n) { return (n < i_minus_thresh); }) == 12 ||
                        std::count_if(h, h + 12, [=](int n) { return n > i_plus_thresh; }) == 12)
                    {
                        m_num_point++;
                        keypoints.push_back(cv::KeyPoint(cv::Point2f(double(j), double(i)), 10, float(0)));
                    }
                }
                cyclic_buffer.clear();*/

            /*  for (auto k = 0; k < 15; k++)
              {
                  if (std::count_if(cyclic_buffer.begin() + k, cyclic_buffer.begin() + 12 + k, [=](int n) { return (n < i_minus_thresh); }) > 11 ||
                      std::count_if(cyclic_buffer.begin() + k, cyclic_buffer.begin() + 12 + k, [=](int n) { return n > i_plus_thresh; }) > 11)
                  {
                      m_num_point++;
                      keypoints.push_back(cv::KeyPoint(cv::Point2f(double(j), double(i)), 10, float(0)));
                  }

              }*/
              /* std::copy(cyclic_buffer.begin(), cyclic_buffer.end(),
                   std::ostream_iterator<int>(std::cout, " "));
               std::cout << std::endl;*/

               /*
                            if (std::count_if(circle_points.begin(), circle_points.end(), [=](int n) { return (n < i_minus_thresh); }) > 11 ||
                                std::count_if(circle_points.begin(), circle_points.end(), [=](int n) { return n > i_plus_thresh; }) > 11)
                            {
                                std::vector<int> cic_points(16);
                                m_num_point++;
                                keypoints.push_back(cv::KeyPoint(cv::Point2f(double(j), double(i)), 10, float(0)));

                            }*/

          /*  int num_dark_points = std::count_if(circle_points.begin(), circle_points.end(), [=](int n) { return (n < i_minus_thresh); });
            if (num_dark_points > 11)
            {
                m_num_point++;
                keypoints.push_back(cv::KeyPoint(cv::Point2f(double(j), double(i)), 10, float(0)));
            }
            else
            {
                int num_ligth_points = std::count_if(circle_points.begin(), circle_points.end(), [=](int n) { return n > i_plus_thresh; });
                if (num_ligth_points > 11)
                {
                    m_num_point++;
                    keypoints.push_back(cv::KeyPoint(cv::Point2f(double(j), double(i)), 10, float(0)));
                }
            }
*/
            //}

            circle_points.clear();
        }
    }
}
} // namespace cvlib
