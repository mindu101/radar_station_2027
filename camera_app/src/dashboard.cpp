#include "dashboard.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

DashboardPublisher::DashboardPublisher(std::string output_directory, std::string source_name, float width, float height)
    : directory(std::move(output_directory)), source(std::move(source_name)),
      field_width(width), field_height(height)
{
    std::filesystem::create_directories(directory);
}

void DashboardPublisher::publish(const cv::Mat& frame, const std::vector<DashboardTarget>& targets,
                                 unsigned int latency_ms, unsigned int result_age_ms, bool calibrated)
{
    if(frame.empty()) return;
    const auto now=std::chrono::steady_clock::now();
    if(last_publish.time_since_epoch().count() && now-last_publish<std::chrono::milliseconds(80)) return;
    last_publish=now;

    try
    {
        const auto next_sequence=sequence+1;
        const auto frame_name="frame-"+std::to_string(next_sequence)+".jpg";
        const auto frame_tmp=directory/(frame_name+".tmp");
        const auto state_tmp=directory/"state.json.tmp";
        std::vector<unsigned char> jpeg;
        cv::Mat preview;
        if(frame.cols>1600)
            cv::resize(frame,preview,cv::Size(1600,static_cast<int>(frame.rows*1600.0/frame.cols)));
        else preview=frame;
        if(!cv::imencode(".jpg",preview,jpeg,{cv::IMWRITE_JPEG_QUALITY,82})) return;
        {
            std::ofstream file(frame_tmp,std::ios::binary);
            file.write(reinterpret_cast<const char*>(jpeg.data()),jpeg.size());
            if(!file) throw std::runtime_error("cannot write dashboard frame");
        }
        std::filesystem::rename(frame_tmp,directory/frame_name);

        const auto timestamp=std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::ostringstream json;
        json<<std::fixed<<std::setprecision(2);
        json<<"{\"sequence\":"<<next_sequence<<",\"timestamp_ms\":"<<timestamp
            <<",\"source\":\""<<source<<"\",\"width\":"<<frame.cols
            <<",\"height\":"<<frame.rows<<",\"field_width\":"<<field_width
            <<",\"field_height\":"<<field_height<<",\"latency_ms\":"<<latency_ms
            <<",\"result_age_ms\":"<<result_age_ms
            <<",\"calibrated\":"<<(calibrated?"true":"false")<<",\"targets\":[";
        bool first=true;
        for(const auto& target:targets)
        {
            if(!first) json<<',';
            first=false;
            json<<"{\"track_id\":"<<target.track_id<<",\"robot_id\":"<<target.robot_id
                <<",\"confidence\":"<<target.confidence
                <<",\"robot_confidence\":"<<target.robot_confidence
                <<",\"bbox\":["<<target.box.x<<','<<target.box.y<<','<<target.box.width<<','<<target.box.height<<']'
                <<",\"pixel\":["<<target.pixel.x<<','<<target.pixel.y<<']'
                <<",\"position\":";
            if(target.has_field_position && std::isfinite(target.field.x) && std::isfinite(target.field.y))
                json<<'['<<target.field.x<<','<<target.field.y<<']';
            else json<<"null";
            json<<'}';
        }
        json<<"]}";
        {
            std::ofstream file(state_tmp,std::ios::binary);
            file<<json.str();
            if(!file) throw std::runtime_error("cannot write dashboard state");
        }
        std::filesystem::rename(state_tmp,directory/"state.json");
        sequence=next_sequence;
        if(sequence>24)
        {
            std::error_code ignored;
            std::filesystem::remove(directory/("frame-"+std::to_string(sequence-24)+".jpg"),ignored);
        }
        warned=false;
    }
    catch(const std::exception& error)
    {
        if(!warned) std::cerr<<"Dashboard output failed: "<<error.what()<<std::endl;
        warned=true;
    }
}
