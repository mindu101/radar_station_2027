#include "fusion/fusion.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>
#include <vector>

namespace fusion {

namespace {

double distance(const Vec2& a, const Vec2& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

Vec2 blend(const Vec2& a, const Vec2& b, double b_weight)
{
    return {a.x + (b.x - a.x) * b_weight, a.y + (b.y - a.y) * b_weight};
}

bool finite(const Vec2& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y);
}

bool finite(const Vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

} // namespace

void Fusion::update_lidar(const std::vector<LidarTrack>& tracks, double now)
{
    for (const LidarTrack& track : tracks)
    {
        if (track.id < 0 || !finite(track.position) || !finite(track.velocity))
            continue;
        const Vec3 position = transform_.apply(track.position);
        const Vec3 velocity = transform_.apply_rotation(track.velocity);
        LidarState& state = lidar_[track.id];
        if (state.has_sample && config_.lidar_velocity_alpha < 1.0)
        {
            const double alpha = std::clamp(config_.lidar_velocity_alpha, 0.0, 1.0);
            state.velocity.x += (velocity.x - state.velocity.x) * alpha;
            state.velocity.y += (velocity.y - state.velocity.y) * alpha;
            state.velocity.z += (velocity.z - state.velocity.z) * alpha;
        }
        else
        {
            state.velocity = velocity;
        }
        state.position = position;
        state.stamp = now;
        state.predicted = track.predicted;
        state.has_sample = true;
    }
}

void Fusion::update_camera(const std::vector<CameraTarget>& targets, double now)
{
    for (const CameraTarget& target : targets)
    {
        if (target.label < 0 || !finite(target.position))
            continue;
        if (!(target.confidence >= config_.min_camera_confidence))
            continue;
        IdentityState& identity = identities_[target.label];
        if (identity.has_camera_sample)
        {
            const double dt = now - identity.camera_stamp;
            if (dt > 1e-4)
            {
                const Vec2 instantaneous{
                    (target.position.x - identity.camera_position.x) / dt,
                    (target.position.y - identity.camera_position.y) / dt};
                identity.camera_velocity = blend(
                    identity.camera_velocity, instantaneous,
                    std::clamp(config_.camera_velocity_alpha, 0.0, 1.0));
            }
        }
        identity.camera_position = target.position;
        identity.camera_stamp = now;
        identity.confidence = target.confidence;
        identity.has_camera_sample = true;
    }
}

void Fusion::update_radio(const std::vector<RadioTarget>& targets, double now)
{
    for (const RadioTarget& target : targets)
    {
        if (target.slot < 0)
            continue;
        RadioState& state = radio_[target.slot];
        state.position = target.position;
        state.stamp = now;
        state.valid = target.valid;
    }
}

void Fusion::prune(double now)
{
    for (auto it = lidar_.begin(); it != lidar_.end();)
    {
        if (now - it->second.stamp > config_.lidar_ttl)
        {
            const int id = it->first;
            for (auto& entry : identities_)
                if (entry.second.lidar_id == id)
                    entry.second.lidar_id = -1;
            it = lidar_.erase(it);
        }
        else
        {
            ++it;
        }
    }
    for (auto it = identities_.begin(); it != identities_.end();)
    {
        if (now - it->second.camera_stamp > config_.identity_ttl)
            it = identities_.erase(it);
        else
            ++it;
    }
    if (config_.use_radio)
    {
        for (auto it = radio_.begin(); it != radio_.end();)
        {
            if (!it->second.valid || now - it->second.stamp > config_.radio_ttl)
                it = radio_.erase(it);
            else
                ++it;
        }
    }
}

std::vector<FusedTarget> Fusion::fuse(double now)
{
    prune(now);

    struct View
    {
        int id = -1;
        Vec2 position;
        Vec2 velocity;
        double stamp = 0.0;
        bool predicted = false;
    };

    std::vector<View> views;
    views.reserve(lidar_.size());
    for (const auto& entry : lidar_)
    {
        const LidarState& state = entry.second;
        const double dt = std::max(0.0, now - state.stamp);
        View view;
        view.id = entry.first;
        view.position = {state.position.x + state.velocity.x * dt,
                         state.position.y + state.velocity.y * dt};
        view.velocity = {state.velocity.x, state.velocity.y};
        view.stamp = state.stamp;
        view.predicted = state.predicted;
        views.push_back(view);
    }
    std::unordered_map<int, std::size_t> view_by_id;
    for (std::size_t i = 0; i < views.size(); ++i)
        view_by_id[views[i].id] = i;

    // 1. 相机检测与激光轨迹的关联：相机给身份，激光给连续位置。
    std::vector<int> fresh_labels;
    fresh_labels.reserve(identities_.size());
    for (const auto& entry : identities_)
        if (now - entry.second.camera_stamp <= config_.camera_ttl)
            fresh_labels.push_back(entry.first);

    struct Pair
    {
        double cost = 0.0;
        int label = -1;
        std::size_t view = 0;
    };
    std::vector<Pair> pairs;
    for (const int label : fresh_labels)
    {
        const IdentityState& identity = identities_.at(label);
        for (std::size_t k = 0; k < views.size(); ++k)
        {
            const double d = distance(identity.camera_position, views[k].position);
            if (d > config_.gate)
                continue;
            const double cost =
                d - (identity.lidar_id == views[k].id ? config_.switch_penalty : 0.0);
            pairs.push_back({cost, label, k});
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) { return a.cost < b.cost; });

    std::set<int> used_labels;
    std::set<std::size_t> used_views;
    std::unordered_map<std::size_t, int> view_owner;
    for (const Pair& pair : pairs)
    {
        if (used_labels.count(pair.label) != 0 || used_views.count(pair.view) != 0)
            continue;
        used_labels.insert(pair.label);
        used_views.insert(pair.view);
        view_owner[pair.view] = pair.label;
        identities_[pair.label].lidar_id = views[pair.view].id;
    }
    // 相机看到了但附近没有激光的类别，解除绑定，按纯相机输出。
    for (const int label : fresh_labels)
        if (used_labels.count(label) == 0)
            identities_[label].lidar_id = -1;
    // 某个激光轨迹被重新分配给别的类别时，旧类别不能再沿用它。
    for (auto& entry : identities_)
    {
        IdentityState& identity = entry.second;
        if (identity.lidar_id < 0)
            continue;
        const auto view = view_by_id.find(identity.lidar_id);
        if (view == view_by_id.end())
        {
            identity.lidar_id = -1;
            continue;
        }
        const auto owner = view_owner.find(view->second);
        if (owner != view_owner.end() && owner->second != entry.first)
            identity.lidar_id = -1;
    }

    // 2. 生成融合结果。
    std::vector<FusedTarget> output;
    output.reserve(identities_.size() + views.size() + radio_.size());
    std::set<int> used_lidar_ids;

    for (const auto& entry : identities_)
    {
        const int label = entry.first;
        const IdentityState& identity = entry.second;
        const double camera_age = now - identity.camera_stamp;
        const View* lidar_view = nullptr;
        if (identity.lidar_id >= 0)
        {
            const auto found = view_by_id.find(identity.lidar_id);
            if (found != view_by_id.end())
                lidar_view = &views[found->second];
        }

        FusedTarget target;
        target.label = label;
        target.age = camera_age;

        if (camera_age <= config_.camera_ttl)
        {
            if (lidar_view != nullptr)
            {
                target.position = blend(identity.camera_position, lidar_view->position,
                                        std::clamp(config_.lidar_weight, 0.0, 1.0));
                target.velocity = lidar_view->velocity;
                target.lidar_id = lidar_view->id;
                target.source = Source::Lidar;
                target.confidence = std::max(identity.confidence, 0.75F);
                target.predicted = lidar_view->predicted;
                used_lidar_ids.insert(lidar_view->id);
            }
            else
            {
                target.position = identity.camera_position;
                target.velocity = identity.camera_velocity;
                target.source = Source::Camera;
                target.confidence = identity.confidence;
            }
            output.push_back(target);
            continue;
        }

        // 相机暂时丢失：优先靠绑定的激光轨迹延续身份。
        if (lidar_view != nullptr && camera_age <= config_.identity_ttl)
        {
            target.position = lidar_view->position;
            target.velocity = lidar_view->velocity;
            target.lidar_id = lidar_view->id;
            target.source = Source::LidarPredicted;
            target.predicted = true;
            const double span = std::max(1e-3, config_.identity_ttl - config_.camera_ttl);
            const double fade = std::clamp((camera_age - config_.camera_ttl) / span, 0.0, 1.0);
            target.confidence = static_cast<float>(0.9 - 0.65 * fade);
            used_lidar_ids.insert(lidar_view->id);
            output.push_back(target);
            continue;
        }

        // 没有激光，只能靠相机速度短时外推。
        if (camera_age <= config_.camera_hold)
        {
            target.position = {
                identity.camera_position.x + identity.camera_velocity.x * camera_age,
                identity.camera_position.y + identity.camera_velocity.y * camera_age};
            target.velocity = identity.camera_velocity;
            target.source = Source::Camera;
            target.predicted = true;
            const double fade = std::clamp(camera_age / std::max(1e-3, config_.camera_hold), 0.0, 1.0);
            target.confidence = identity.confidence * static_cast<float>(1.0 - fade);
            output.push_back(target);
        }
    }

    // 3. 没有身份归属的激光轨迹。
    if (config_.include_unlabeled)
    {
        for (const View& view : views)
        {
            if (used_lidar_ids.count(view.id) != 0)
                continue;
            FusedTarget target;
            target.label = -1;
            target.lidar_id = view.id;
            target.position = view.position;
            target.velocity = view.velocity;
            target.source = Source::Lidar;
            target.predicted = view.predicted;
            target.confidence = view.predicted ? 0.3F : 0.5F;
            target.age = now - view.stamp;
            output.push_back(target);
        }
    }

    // 4. 无线电兜底：附近没有任何相机/激光结果时才输出。
    if (config_.use_radio)
    {
        for (const auto& entry : radio_)
        {
            const RadioState& state = entry.second;
            if (!state.valid || now - state.stamp > config_.radio_ttl)
                continue;
            bool covered = false;
            for (const FusedTarget& target : output)
                if (distance(target.position, state.position) <= config_.radio_gate)
                {
                    covered = true;
                    break;
                }
            if (covered)
                continue;
            FusedTarget target;
            target.label = -1;
            target.slot = entry.first;
            target.position = state.position;
            target.source = Source::Radio;
            target.confidence = 0.4F;
            target.age = now - state.stamp;
            output.push_back(target);
        }
    }

    std::stable_sort(output.begin(), output.end(),
                     [](const FusedTarget& a, const FusedTarget& b)
                     {
                         const bool a_labeled = a.label >= 0;
                         const bool b_labeled = b.label >= 0;
                         if (a_labeled != b_labeled)
                             return a_labeled;
                         if (a.label != b.label)
                             return a.label < b.label;
                         return a.slot < b.slot;
                     });
    return output;
}

void Fusion::reset()
{
    lidar_.clear();
    identities_.clear();
    radio_.clear();
}

} // namespace fusion
