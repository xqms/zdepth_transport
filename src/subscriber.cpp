// Subscriber
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <image_transport/simple_subscriber_plugin.h>
#include <sensor_msgs/CompressedImage.h>
#include <pluginlib/class_list_macros.hpp>

#include <rosfmt/full.h>

#include <zdepth.hpp>

namespace zdepth_transport
{

class Subscriber : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
public:
    ~Subscriber() {}

    std::string getTransportName() const override
    {
        return "zDepth";
    }

protected:
    void internalCallback(const sensor_msgs::CompressedImageConstPtr& message, const Callback& user_cb) override
    {
        if(!m_compressor)
            m_compressor = std::make_unique<zdepth::DepthCompressor>();

        auto out = boost::make_shared<sensor_msgs::Image>();

        std::vector<uint16_t> outData;
        int width;
        int height;

        auto ret = m_compressor->Decompress(message->data, width, height, outData);
        if(ret != zdepth::DepthResult::Success)
        {
            ROSFMT_ERROR("Could not decompress zdepth image: {}", ret);
            return;
        }

        out->data.resize(sizeof(uint16_t)*outData.size());
        std::memcpy(out->data.data(), outData.data(), out->data.size());

        out->width = width;
        out->height = height;
        out->step = sizeof(uint16_t)*width;
        out->encoding = "16UC1";
        out->header = message->header;

        user_cb(out);
    }

private:
    std::unique_ptr<zdepth::DepthCompressor> m_compressor;
};

}

PLUGINLIB_EXPORT_CLASS(zdepth_transport::Subscriber, image_transport::SubscriberPlugin)
