/* Descriptor matcher algorithm implementation.
 * @file
 * @date 2018-11-25
 * @author Anonymous
 */

#include "cvlib.hpp"

namespace cvlib
{
void descriptor_matcher::knnMatchImpl(cv::InputArray queryDescriptors, std::vector<std::vector<cv::DMatch>>& matches, int k /*unhandled*/,
                                      cv::InputArrayOfArrays masks /*unhandled*/, bool compactResult /*unhandled*/)
{
    if (trainDescCollection.empty())
        return;

    auto q_desc = queryDescriptors.getMat();
    auto& t_desc = trainDescCollection[0];

    matches.resize(q_desc.rows);

    cv::RNG rnd;
    for (int i = 0; i < q_desc.rows; ++i)
    {
        for (j = 0; j < t_desc.rows)
        // \todo implement Ratio of SSD check.
        // matches[i].emplace_back(i, rnd.uniform(0, t_desc.rows), FLT_MAX);
    }
}

void descriptor_matcher::radiusMatchImpl(cv::InputArray queryDescriptors, std::vector<std::vector<cv::DMatch>>& matches, float maxDistance,
                                         cv::InputArrayOfArrays masks /*unhandled*/, bool compactResult /*unhandled*/)
{
    // \todo implement matching with "maxDistance"

    if (trainDescCollection.empty())
        return;

    auto q_desc = queryDescriptors.getMat();
    auto& t_desc = trainDescCollection[0];

    // matches.resize(q_desc.rows);

    // cv::RNG rnd;
    cv::Mat true_desc;
    for (int i = 0; i < q_desc.rows; ++i)
    {
        for (int j = 0; j < t_desc.rows; ++j)
        {
            if (cv::norm(q_desc.row(i) - t_desc.row(j), cv::NORM_L2) <= maxDistance)
            {
                if (true_desc.empty())
                    true_desc = q_desc.row(i).clone();
                else
                    cv::vconcat(true_desc, q_desc.row(i), true_desc);
            }
        }
    }

    // knnMatchImpl(true_desc, matches, 1, masks, compactResult);
}
} // namespace cvlib
