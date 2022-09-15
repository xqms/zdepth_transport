// Publisher
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <image_transport/simple_publisher_plugin.h>
#include <sensor_msgs/CompressedImage.h>

#include <pluginlib/class_list_macros.hpp>

#include <rosfmt/full.h>

#include <zdepth.hpp>

namespace zdepth_transport
{

class Publisher : public image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage>
{
public:
    virtual ~Publisher() {}

    virtual std::string getTransportName() const override
    {
        return "zDepth";
    }

private:
    void publish(const sensor_msgs::Image& message,
                        const PublishFn& publish_fn) const override;

    mutable std::unique_ptr<zdepth::DepthCompressor> m_compressor;
};

void Publisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
    if(message.encoding != "16UC1")
    {
        ROSFMT_ERROR("Invalid encoding for zdepth transport: {}", message.encoding);
        return;
    }

    if(!m_compressor)
        m_compressor = std::make_unique<zdepth::DepthCompressor>();

    auto data = reinterpret_cast<const uint16_t*>(message.data.data());

    auto out = boost::make_shared<sensor_msgs::CompressedImage>();
    out->header = message.header;
    out->format = "zdepth";

    auto ret = m_compressor->Compress(message.width, message.height, data, out->data, true);
    if(ret != zdepth::DepthResult::Success)
    {
        ROSFMT_WARN_THROTTLE(1.0, "Could not encode depth image");
        return;
    }

    publish_fn(*out);
}

}

PLUGINLIB_EXPORT_CLASS(zdepth_transport::Publisher, image_transport::PublisherPlugin)
