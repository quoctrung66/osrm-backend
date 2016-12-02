#ifndef OSRM_GUIDANCE_TOOLKIT_HPP_
#define OSRM_GUIDANCE_TOOLKIT_HPP_

#include "util/attributes.hpp"
#include "util/bearing.hpp"
#include "util/coordinate.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/guidance/toolkit.hpp"
#include "util/guidance/turn_lanes.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include "extractor/compressed_edge_container.hpp"
#include "extractor/query_node.hpp"

#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/road_classification.hpp"
#include "extractor/guidance/turn_instruction.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace osrm
{
namespace extractor
{
namespace guidance
{

inline bool obviousByRoadClass(const RoadClassification in_classification,
                               const RoadClassification obvious_candidate,
                               const RoadClassification compare_candidate)
{
    // lower numbers are of higher priority
    const bool has_high_priority = PRIORITY_DISTINCTION_FACTOR * obvious_candidate.GetPriority() <
                                   compare_candidate.GetPriority();

    const bool continues_on_same_class = in_classification == obvious_candidate;
    return (has_high_priority && continues_on_same_class) ||
           (!obvious_candidate.IsLowPriorityRoadClass() &&
            !in_classification.IsLowPriorityRoadClass() &&
            compare_candidate.IsLowPriorityRoadClass());
}

/* We use the sum of least squares to calculate a linear regression through our
* coordinates.
* This regression gives a good idea of how the road can be perceived and corrects for
* initial and final corrections
*/
inline std::pair<util::Coordinate, util::Coordinate>
leastSquareRegression(const std::vector<util::Coordinate> &coordinates)
{
    BOOST_ASSERT(coordinates.size() >= 2);
    double sum_lon = 0, sum_lat = 0, sum_lon_lat = 0, sum_lon_lon = 0;
    double min_lon = static_cast<double>(toFloating(coordinates.front().lon));
    double max_lon = static_cast<double>(toFloating(coordinates.front().lon));
    for (const auto coord : coordinates)
    {
        min_lon = std::min(min_lon, static_cast<double>(toFloating(coord.lon)));
        max_lon = std::max(max_lon, static_cast<double>(toFloating(coord.lon)));
        sum_lon += static_cast<double>(toFloating(coord.lon));
        sum_lon_lon +=
            static_cast<double>(toFloating(coord.lon)) * static_cast<double>(toFloating(coord.lon));
        sum_lat += static_cast<double>(toFloating(coord.lat));
        sum_lon_lat +=
            static_cast<double>(toFloating(coord.lon)) * static_cast<double>(toFloating(coord.lat));
    }

    const auto dividend = coordinates.size() * sum_lon_lat - sum_lon * sum_lat;
    const auto divisor = coordinates.size() * sum_lon_lon - sum_lon * sum_lon;
    if (std::abs(divisor) < std::numeric_limits<double>::epsilon())
        return std::make_pair(coordinates.front(), coordinates.back());

    // slope of the regression line
    const auto slope = dividend / divisor;
    const auto intercept = (sum_lat - slope * sum_lon) / coordinates.size();

    const auto GetLatAtLon = [intercept,
                              slope](const util::FloatLongitude longitude) -> util::FloatLatitude {
        return {intercept + slope * static_cast<double>((longitude))};
    };

    const util::Coordinate regression_first = {
        toFixed(util::FloatLongitude{min_lon - 1}),
        toFixed(util::FloatLatitude(GetLatAtLon(util::FloatLongitude{min_lon - 1})))};
    const util::Coordinate regression_end = {
        toFixed(util::FloatLongitude{max_lon + 1}),
        toFixed(util::FloatLatitude(GetLatAtLon(util::FloatLongitude{max_lon + 1})))};

    return {regression_first, regression_end};
}

inline std::uint8_t getLaneCountAtIntersection(const NodeID intersection_node,
                                               const util::NodeBasedDynamicGraph &node_based_graph)
{
    std::uint8_t lanes = 0;
    for (const EdgeID onto_edge : node_based_graph.GetAdjacentEdgeRange(intersection_node))
        lanes = std::max(
            lanes, node_based_graph.GetEdgeData(onto_edge).road_classification.GetNumberOfLanes());
    return lanes;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif // OSRM_GUIDANCE_TOOLKIT_HPP_
