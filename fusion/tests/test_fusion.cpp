#include "fusion/fusion.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace {

int failures = 0;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,         \
                         __LINE__, #condition);                                 \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                   \
    CHECK(std::fabs((a) - (b)) <= (eps))

const fusion::FusedTarget* find_label(const std::vector<fusion::FusedTarget>& targets, int label)
{
    for (const fusion::FusedTarget& target : targets)
        if (target.label == label)
            return &target;
    return nullptr;
}

const fusion::FusedTarget* find_slot(const std::vector<fusion::FusedTarget>& targets, int slot)
{
    for (const fusion::FusedTarget& target : targets)
        if (target.slot == slot)
            return &target;
    return nullptr;
}

std::string write_temp_file(const char* name, const std::string& content)
{
    const std::string path = std::string("/tmp/") + name;
    std::ofstream output(path);
    output << content;
    output.close();
    return path;
}

void test_transform_json()
{
    const std::string path = write_temp_file(
        "fusion_test_calibration.json",
        "{\n"
        "  \"rotation\": [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],\n"
        "  \"translation\": [1.0, 2.0, 3.0],\n"
        "  \"field_length\": 28.0\n"
        "}\n");
    fusion::FieldTransform transform;
    std::string error;
    CHECK(transform.load_json(path, &error));
    const fusion::Vec3 origin = transform.apply({0.0, 0.0, 0.0});
    CHECK_NEAR(origin.x, 1.0, 1e-9);
    CHECK_NEAR(origin.y, 2.0, 1e-9);
    CHECK_NEAR(origin.z, 3.0, 1e-9);
    const fusion::Vec3 unit_x = transform.apply({1.0, 0.0, 0.0});
    CHECK_NEAR(unit_x.x, 1.0, 1e-9);
    CHECK_NEAR(unit_x.y, 3.0, 1e-9);
    CHECK_NEAR(unit_x.z, 3.0, 1e-9);
    const fusion::Vec3 velocity = transform.apply_rotation({1.0, 0.0, 0.0});
    CHECK_NEAR(velocity.x, 0.0, 1e-9);
    CHECK_NEAR(velocity.y, 1.0, 1e-9);
}

void test_config_load()
{
    const std::string path = write_temp_file(
        "fusion_test_config.conf",
        "# comment\n"
        "gate = 1.5\n"
        "camera_ttl: 0.4\n"
        "use_radio = true\n"
        "include_unlabeled = false\n"
        "min_camera_confidence = 0.2\n");
    fusion::FusionConfig config;
    std::string error;
    CHECK(config.load(path, &error));
    CHECK_NEAR(config.gate, 1.5, 1e-9);
    CHECK_NEAR(config.camera_ttl, 0.4, 1e-9);
    CHECK(config.use_radio);
    CHECK(!config.include_unlabeled);
    CHECK_NEAR(config.min_camera_confidence, 0.2, 1e-6);
}

void test_camera_lidar_association()
{
    fusion::FusionConfig config;
    config.lidar_weight = 1.0;
    config.include_unlabeled = false;
    fusion::Fusion fusion(config);
    fusion.update_lidar({{1, {5.0, 5.0, 0.0}, {0.0, 0.0, 0.0}, false}}, 10.0);
    fusion.update_camera({{7, {5.2, 5.1}, 1.0F}}, 10.0);
    const auto targets = fusion.fuse(10.0);
    const fusion::FusedTarget* target = find_label(targets, 7);
    CHECK(target != nullptr);
    if (target)
    {
        CHECK(target->lidar_id == 1);
        CHECK(target->source == fusion::Source::Lidar);
        CHECK_NEAR(target->position.x, 5.0, 0.2);
        CHECK_NEAR(target->position.y, 5.0, 0.2);
    }
}

void test_camera_only()
{
    fusion::FusionConfig config;
    config.include_unlabeled = false;
    fusion::Fusion fusion(config);
    fusion.update_camera({{7, {3.0, 4.0}, 0.9F}}, 10.0);
    const auto targets = fusion.fuse(10.0);
    const fusion::FusedTarget* target = find_label(targets, 7);
    CHECK(target != nullptr);
    if (target)
    {
        CHECK(target->source == fusion::Source::Camera);
        CHECK_NEAR(target->position.x, 3.0, 1e-9);
        CHECK_NEAR(target->position.y, 4.0, 1e-9);
        CHECK_NEAR(target->confidence, 0.9, 1e-6);
    }
}

void test_identity_persistence()
{
    fusion::FusionConfig config;
    config.include_unlabeled = false;
    fusion::Fusion fusion(config);
    fusion.update_lidar({{3, {10.0, 10.0, 0.0}, {1.0, 0.0, 0.0}, false}}, 10.0);
    fusion.update_camera({{9, {10.1, 10.0}, 1.0F}}, 10.0);
    // 先建立绑定。
    auto targets = fusion.fuse(10.0);
    CHECK(find_label(targets, 9) != nullptr);
    // 相机丢失 0.5s，激光轨迹仍有效：靠外推继续输出身份。
    targets = fusion.fuse(10.5);
    const fusion::FusedTarget* target = find_label(targets, 9);
    CHECK(target != nullptr);
    if (target)
    {
        CHECK(target->source == fusion::Source::LidarPredicted);
        CHECK(target->predicted);
        CHECK_NEAR(target->position.x, 10.5, 0.05);
    }
    // 超过 identity_ttl 后身份必须消失。
    targets = fusion.fuse(12.5);
    CHECK(find_label(targets, 9) == nullptr);
}

void test_gate_rejection()
{
    fusion::FusionConfig config;
    config.include_unlabeled = false;
    fusion::Fusion fusion(config);
    fusion.update_lidar({{1, {5.0, 5.0, 0.0}, {0.0, 0.0, 0.0}, false}}, 10.0);
    fusion.update_camera({{7, {0.0, 0.0}, 1.0F}}, 10.0);
    const auto targets = fusion.fuse(10.0);
    const fusion::FusedTarget* target = find_label(targets, 7);
    CHECK(target != nullptr);
    if (target)
    {
        CHECK(target->source == fusion::Source::Camera);
        CHECK(target->lidar_id == -1);
    }
}

void test_unlabeled_lidar()
{
    fusion::Fusion fusion;
    fusion.update_lidar({{4, {8.0, 2.0, 0.0}, {0.0, 0.0, 0.0}, false}}, 10.0);
    const auto targets = fusion.fuse(10.0);
    const fusion::FusedTarget* target = find_label(targets, -1);
    CHECK(target != nullptr);
    if (target)
        CHECK(target->lidar_id == 4);
}

void test_radio_fallback()
{
    fusion::FusionConfig config;
    config.use_radio = true;
    config.include_unlabeled = false;
    fusion::Fusion fusion(config);
    fusion.update_radio({{2, {6.0, 6.0}, true}}, 10.0);
    const auto targets = fusion.fuse(10.0);
    const fusion::FusedTarget* target = find_slot(targets, 2);
    CHECK(target != nullptr);
    if (target)
        CHECK(target->source == fusion::Source::Radio);
}

} // namespace

int main()
{
    test_transform_json();
    test_config_load();
    test_camera_lidar_association();
    test_camera_only();
    test_identity_persistence();
    test_gate_rejection();
    test_unlabeled_lidar();
    test_radio_fallback();

    if (failures == 0)
    {
        std::printf("fusion tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "fusion tests failed: %d\n", failures);
    return 1;
}
