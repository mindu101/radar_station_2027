#include "fusion/types.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fusion {

namespace {

std::string trim(const std::string& value)
{
    const auto begin = std::find_if_not(value.begin(), value.end(),
                                        [](unsigned char c) { return std::isspace(c) != 0; });
    const auto end = std::find_if_not(value.rbegin(), value.rend(),
                                      [](unsigned char c) { return std::isspace(c) != 0; })
                         .base();
    if (begin >= end)
        return {};
    return std::string(begin, end);
}

// 从 JSON 文本里取出 key 后面的 n 个数字。calibrate_field.py 生成的是
// 固定格式，没必要引入 JSON 库。
bool extract_numbers(const std::string& text, const std::string& key,
                     std::size_t count, double* output)
{
    const std::size_t key_pos = text.find('"' + key + '"');
    if (key_pos == std::string::npos)
        return false;
    const std::size_t array_begin = text.find('[', key_pos);
    if (array_begin == std::string::npos)
        return false;
    const char* cursor = text.c_str() + array_begin + 1;
    for (std::size_t i = 0; i < count; ++i)
    {
        // 跳过逗号、空白等分隔符。
        while (*cursor != '\0' &&
               (std::isspace(static_cast<unsigned char>(*cursor)) != 0 ||
                *cursor == ','))
            ++cursor;
        char* end = nullptr;
        const double value = std::strtod(cursor, &end);
        if (end == cursor || !std::isfinite(value))
            return false;
        output[i] = value;
        cursor = end;
    }
    return true;
}

bool parse_bool(const std::string& value, bool& output)
{
    if (value == "1" || value == "true" || value == "yes" || value == "on")
    {
        output = true;
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off")
    {
        output = false;
        return true;
    }
    return false;
}

bool parse_double(const std::string& value, double& output)
{
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed))
        return false;
    output = parsed;
    return true;
}

} // namespace

Vec3 FieldTransform::apply(const Vec3& lidar_point) const
{
    Vec3 field;
    field.x = rotation[0] * lidar_point.x + rotation[1] * lidar_point.y +
              rotation[2] * lidar_point.z + translation[0];
    field.y = rotation[3] * lidar_point.x + rotation[4] * lidar_point.y +
              rotation[5] * lidar_point.z + translation[1];
    field.z = rotation[6] * lidar_point.x + rotation[7] * lidar_point.y +
              rotation[8] * lidar_point.z + translation[2];
    return field;
}

Vec3 FieldTransform::apply_rotation(const Vec3& lidar_vector) const
{
    Vec3 field;
    field.x = rotation[0] * lidar_vector.x + rotation[1] * lidar_vector.y +
              rotation[2] * lidar_vector.z;
    field.y = rotation[3] * lidar_vector.x + rotation[4] * lidar_vector.y +
              rotation[5] * lidar_vector.z;
    field.z = rotation[6] * lidar_vector.x + rotation[7] * lidar_vector.y +
              rotation[8] * lidar_vector.z;
    return field;
}

void FieldTransform::set_identity()
{
    const double identity[9] = {1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0};
    std::copy(identity, identity + 9, rotation);
    translation[0] = translation[1] = translation[2] = 0.0;
}

bool FieldTransform::load_json(const std::string& path, std::string* error)
{
    std::ifstream input(path);
    if (!input)
    {
        if (error)
            *error = "cannot open " + path;
        return false;
    }
    const std::string text((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
    if (!extract_numbers(text, "rotation", 9, rotation))
    {
        if (error)
            *error = "missing or invalid rotation in " + path;
        return false;
    }
    if (!extract_numbers(text, "translation", 3, translation))
    {
        if (error)
            *error = "missing or invalid translation in " + path;
        return false;
    }
    return true;
}

bool FusionConfig::load(const std::string& path, std::string* error)
{
    std::ifstream input(path);
    if (!input)
    {
        if (error)
            *error = "cannot open " + path;
        return false;
    }
    std::string line;
    int line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos)
            line.erase(comment);
        line = trim(line);
        if (line.empty())
            continue;
        std::size_t separator = line.find('=');
        if (separator == std::string::npos)
            separator = line.find(':');
        if (separator == std::string::npos)
        {
            if (error)
                *error = path + ":" + std::to_string(line_number) + ": expected key=value";
            return false;
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        bool ok = true;
        double number = 0.0;
        if (key == "gate")
            ok = parse_double(value, gate);
        else if (key == "switch_penalty")
            ok = parse_double(value, switch_penalty);
        else if (key == "camera_ttl")
            ok = parse_double(value, camera_ttl);
        else if (key == "camera_hold")
            ok = parse_double(value, camera_hold);
        else if (key == "identity_ttl")
            ok = parse_double(value, identity_ttl);
        else if (key == "lidar_ttl")
            ok = parse_double(value, lidar_ttl);
        else if (key == "lidar_weight")
            ok = parse_double(value, lidar_weight);
        else if (key == "camera_velocity_alpha")
            ok = parse_double(value, camera_velocity_alpha);
        else if (key == "lidar_velocity_alpha")
            ok = parse_double(value, lidar_velocity_alpha);
        else if (key == "radio_ttl")
            ok = parse_double(value, radio_ttl);
        else if (key == "radio_gate")
            ok = parse_double(value, radio_gate);
        else if (key == "use_radio")
            ok = parse_bool(value, use_radio);
        else if (key == "include_unlabeled")
            ok = parse_bool(value, include_unlabeled);
        else if (key == "min_camera_confidence")
        {
            ok = parse_double(value, number);
            if (ok)
                min_camera_confidence = static_cast<float>(number);
        }
        else
        {
            if (error)
                *error = path + ":" + std::to_string(line_number) + ": unknown key " + key;
            return false;
        }
        if (!ok)
        {
            if (error)
                *error = path + ":" + std::to_string(line_number) + ": bad value for " + key;
            return false;
        }
    }
    return true;
}

double now_seconds()
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

const char* source_name(Source source)
{
    switch (source)
    {
    case Source::Camera:
        return "camera";
    case Source::Lidar:
        return "lidar";
    case Source::LidarPredicted:
        return "lidar-pred";
    case Source::Radio:
        return "radio";
    case Source::None:
    default:
        return "none";
    }
}

} // namespace fusion
