#include <cstdint>
#include <iostream>

#include "vision_api/converters/vision_api_buffer_converter_libimg_.hpp"
#include "vision_api/vision_api.hpp"
#include "libimg/libimg.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"


namespace sick_vision_ros2
{

class SickVision : public rclcpp::Node
{
  public:
    explicit SickVision(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("sick_vision", options),
      pub_cam_(image_transport::create_publisher(this, "/sick_cam"))
    {
      // initialize vision_api library
      vision_api::Library::Initialize();

      connectCamera();
    }

    ~SickVision() {
      // close library before exiting program
      vision_api::Library::Close();
    }
  
  private:
    void connectCamera() {
      // create a camera manager object
      auto &deviceManager = vision_api::DeviceManager::Instance();

      // define the CTI that will be used
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("sick_vision_ros2");
      std::string cti = package_share_directory + "/cti/sick_gevgentl.cti";

      // add CTI to deviceManager
      deviceManager.AddProducerLibrary(cti);

      // update the deviceManager
      deviceManager.Update(vision_api::DeviceManager::UpdatePolicy::DontScanEnvironmentForProducerLibraries);

      // wait for camera to be found
      if (deviceManager.Devices().empty())
      {
        RCLCPP_ERROR(get_logger(), "No camera found");
        rclcpp::shutdown();
      }
      else
      {
        std::string serialNumber = declare_parameter("serial_no", rclcpp::ParameterValue("")).get<rclcpp::PARAMETER_STRING>();
        // A ROS2 hack: until a better way is found to avoid auto convertion of strings containing only digits to integers:
        if (serialNumber.front() == '_') serialNumber = serialNumber.substr(1);    // remove '_' prefix
        float fps = declare_parameter("fps", 30.0);

        // list all available devices
        uint64_t i = 0;
        std::string info;
        int selectedDevice = 0;
        info = "Devices available: \n";
        for (const auto &deviceDescriptor: deviceManager.Devices())
        {
          info = info + std::to_string(i) + ": " + deviceDescriptor->ModelName() + " (" +
            deviceDescriptor->ParentInterface()->DisplayName() + "; " +
            deviceDescriptor->ParentInterface()->ParentSystem()->DisplayName() + " v." +
            deviceDescriptor->ParentInterface()->ParentSystem()->Version() + ")\n" +
            + "S/N: " + deviceDescriptor->SerialNumber() + "\n";
          if (deviceDescriptor->SerialNumber() == serialNumber) {
            selectedDevice = i;
          }
          ++i;
        }
        RCLCPP_INFO(get_logger(), info.c_str());
        RCLCPP_INFO(get_logger(), ("Publishing from Device: " + std::to_string(selectedDevice)).c_str());

        // open camera
        auto device = deviceManager.Devices().at(selectedDevice)->OpenDevice(vision_api::core::DeviceAccessType::Control);
        // get the remote device node map
        auto nodeMapRemoteDevice = device->RemoteDevice()->NodeMaps().at(selectedDevice);
        // open the data stream
        auto dataStream = device->DataStreams().at(selectedDevice)->OpenDataStream();

        try
        {
          // allocate and announce image buffers
          auto payloadSize = nodeMapRemoteDevice->FindNode<vision_api::core::nodes::IntegerNode>("PayloadSize")->Value();
          auto bufferCountMax = dataStream->NumBuffersAnnouncedMinRequired();
          for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
          {
            auto buffer = dataStream->AllocAndAnnounceBuffer(static_cast<size_t> (payloadSize), nullptr);
            dataStream->QueueBuffer(buffer);
          }

          // prepare for untriggered continuous image acquisition
          nodeMapRemoteDevice->FindNode<vision_api::core::nodes::EnumerationNode>("TriggerSelector")->SetCurrentEntry("ExposureStart");
          nodeMapRemoteDevice->FindNode<vision_api::core::nodes::EnumerationNode>("TriggerMode")->SetCurrentEntry("Off");
          auto nodeFrameRate = nodeMapRemoteDevice->FindNode<vision_api::core::nodes::FloatNode>("AcquisitionFrameRate");
          nodeFrameRate->SetValue(fps);

          // start acquisition
          dataStream->StartAcquisition();
          nodeMapRemoteDevice->FindNode<vision_api::core::nodes::CommandNode>("AcquisitionStart")->Execute();

          // process the acquired images
          auto imageWidth = static_cast<int> (nodeMapRemoteDevice->FindNode<vision_api::core::nodes::IntegerNode > ("Width")->Value());
          auto imageHeight = static_cast<int> (nodeMapRemoteDevice->FindNode<vision_api::core::nodes::IntegerNode > ("Height")->Value());

          cv::Mat cv_image(imageHeight, imageWidth, CV_8UC3);
          int byteCount = cv_image.total() * cv_image.elemSize();

          while (rclcpp::ok())
          {
            // get buffer from datastream and create SICK LibIMG image from it
            auto buffer = dataStream->WaitForFinishedBuffer(5000);

            // create SICK LibIMG image from buffer
            auto iplImage = vision_api::BufferTo<libimg::Image>(buffer).ConvertTo(libimg::PixelFormatName::BGR8, cv_image.data, static_cast<size_t> (byteCount));;

            // Queue buffer so that it can be used again
            dataStream->QueueBuffer(buffer);

            std::memcpy(cv_image.data, iplImage.Data(), byteCount);

            img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image)
                        .toImageMsg();
            pub_cam_.publish(*img_msg.get());
          }

          dataStream->StopAcquisition(vision_api::core::AcquisitionStopMode::Default);
          nodeMapRemoteDevice->FindNode<vision_api::core::nodes::CommandNode>("AcquisitionStop")->Execute();

          // flush and revoke all buffers
          dataStream->Flush(vision_api::core::DataStreamFlushMode::DiscardAll);
          for (const auto &buffer: dataStream->AnnouncedBuffers())
          {
            dataStream->RevokeBuffer(buffer);
          }
        }

        catch (vision_api::core::Exception& e)
        {
          RCLCPP_ERROR(get_logger(), e.what());
        }
      }
    };

  const image_transport::Publisher pub_cam_;
  sensor_msgs::msg::Image::SharedPtr img_msg;
};

}	// namespace sick_vision_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sick_vision_ros2::SickVision)
