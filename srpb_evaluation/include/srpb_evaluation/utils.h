#pragma once

#include <exception>
#include <fstream>
#include <functional>
#include <string>
#include <tuple>
#include <vector>

#include <people_msgs_utils/utils.h>

namespace srpb {
namespace evaluation {

/**
 * @brief Does similar to what @ref people_msgs_utils::fillGroupsWithMembers does, but considers timestamps of data
 *
 * @return Modified @ref timed_groups vector
 */
std::vector<std::pair<double, people_msgs_utils::Group>> fillGroupsWithMembers(
    const std::vector<std::pair<double, people_msgs_utils::Group>>& timed_groups,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& timed_people
);

/**
 * @defgroup templates Template functions
 * @{
 */
// string to pair (first = timestamp, second = logged state)
template <typename T>
static std::vector<std::pair<double, T>> parseFile(const std::string& filepath, std::function<T(const std::string&)> from_string_fun) {
    std::ifstream file(filepath);
    if (file.bad()) {
        throw std::runtime_error("Could not open the log file");
    }

    std::vector<std::pair<double, T>> data_log;
    std::string line;
    // parse lines 1 by 1
    while (std::getline(file, line) && !file.bad()) {
        if (line.empty()) {
            std::cout << "File contains an empty line, it may be damaged!" << std::endl;
            continue;
        }

        // extract timestamp
        auto ts_start = line.find_first_not_of(' ');
        if (ts_start == std::string::npos) {
            continue;
        }
        std::string timestamp_str = line.substr(ts_start);
        line = line.substr(ts_start);
        auto ts_end = line.find_first_of(' ');
        if (ts_end == std::string::npos) {
            continue;
        }
        timestamp_str = timestamp_str.substr(0, ts_end);

        // extract logged state
        data_log.push_back(
            std::make_pair(
            std::stod(timestamp_str),
            from_string_fun(line.substr(ts_end + 1))
            )
        );
    }

    file.close();
    return data_log;
}
/// @}

} // namespace evaluation
} // namespace srpb
