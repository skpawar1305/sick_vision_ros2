#include <cstdint>
#include <iostream>

#include <vision_api/converters/vision_api_buffer_converter_libimg_.hpp>
#include <vision_api/vision_api.hpp>
#include <libimg/libimg.hpp>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"


int main()
{
    // initialize vision_api library
    vision_api::Library::Initialize();

    // create a camera manager object
    auto& deviceManager = vision_api::DeviceManager::Instance();

    try
    {
        // update the cameraManager
        deviceManager.Update();

        // exit program if no camera was found
        if (deviceManager.Devices().empty())
        {
            std::cout << "No camera found. Exiting program." << std::endl << std::endl;
            // close library before exiting program
            vision_api::Library::Close();
            return 0;
        }

        // list all available devices
        uint64_t i = 0;
        std::cout << "Devices available: " << std::endl;
        for (const auto& deviceDescriptor : deviceManager.Devices())
        {
            std::cout << i << ": " << deviceDescriptor->ModelName() << " ("
                      << deviceDescriptor->ParentInterface()->DisplayName() << "; "
                      << deviceDescriptor->ParentInterface()->ParentSystem()->DisplayName() << " v."
                      << deviceDescriptor->ParentInterface()->ParentSystem()->Version() << ")" << std::endl;
            ++i;
        }

        int selectedDevice = 0;
        // open camera
        auto device = deviceManager.Devices().at(selectedDevice)->OpenDevice(vision_api::core::DeviceAccessType::Control);
        // get the remote device node map
        auto nodeMapRemoteDevice = device->RemoteDevice()->NodeMaps().at(0);
        // open the data stream
        auto dataStream = device->DataStreams().at(0)->OpenDataStream();

        // allocate and announce image buffers
        auto payloadSize = nodeMapRemoteDevice->FindNode<vision_api::core::nodes::IntegerNode>("PayloadSize")->Value();
        auto bufferCountMax = dataStream->NumBuffersAnnouncedMinRequired();
        for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
        {
            auto buffer = dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
            dataStream->QueueBuffer(buffer);
        }

        // prepare for untriggered continuous image acquisition
        nodeMapRemoteDevice->FindNode<vision_api::core::nodes::EnumerationNode>("TriggerSelector")
            ->SetCurrentEntry("ExposureStart");
        nodeMapRemoteDevice->FindNode<vision_api::core::nodes::EnumerationNode>("TriggerMode")->SetCurrentEntry("Off");
        nodeMapRemoteDevice->FindNode<vision_api::core::nodes::FloatNode>("AcquisitionFrameRate")->SetValue(30);

        // start acquisition
        dataStream->StartAcquisition();
        nodeMapRemoteDevice->FindNode<vision_api::core::nodes::CommandNode>("AcquisitionStart")->Execute();
        nodeMapRemoteDevice->FindNode<vision_api::core::nodes::CommandNode>("AcquisitionStart")->WaitUntilDone();

        // process the acquired images
        auto imageWidth = static_cast<int>(
            nodeMapRemoteDevice->FindNode<vision_api::core::nodes::IntegerNode>("Width")->Value());
        auto imageHeight = static_cast<int>(
            nodeMapRemoteDevice->FindNode<vision_api::core::nodes::IntegerNode>("Height")->Value());

        cv::Mat cv_image(imageHeight, imageWidth, CV_8UC4);
        int byteCount = cv_image.total()*cv_image.elemSize();

        while (true)
        {
            // get buffer from datastream and create SICK LibIMG image from it
            auto buffer = dataStream->WaitForFinishedBuffer(5000);

            // create SICK LibIMG image from buffer
            auto iplImage = vision_api::BufferTo<libimg::Image>(buffer).ConvertTo(
                libimg::PixelFormatName::BGRa8, cv_image.data, static_cast<size_t>(byteCount));;

            // Queue buffer so that it can be used again
            dataStream->QueueBuffer(buffer);

            std::memcpy(cv_image.data, iplImage.Data(), byteCount);

            cv::imshow("h", cv_image);
            cv::waitKey(20);
        }

        // stop acquistion of camera
        try
        {
            dataStream->StopAcquisition(vision_api::core::AcquisitionStopMode::Default);
        }
        catch (const std::exception&)
        {
            // Some transport layers need no explicit acquisition stop of the datastream when starting its
            // acquisition with a finite number of images. Ignoring Errors due to that TL behavior.

            std::cout << "WARNING: Ignoring that TL failed to stop acquisition on datastream." << std::endl;
        }
        nodeMapRemoteDevice->FindNode<vision_api::core::nodes::CommandNode>("AcquisitionStop")->Execute();

        // flush and revoke all buffers
        dataStream->Flush(vision_api::core::DataStreamFlushMode::DiscardAll);
        for (const auto& buffer : dataStream->AnnouncedBuffers())
        {
            dataStream->RevokeBuffer(buffer);
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
    }

    // close library before exiting program
    vision_api::Library::Close();
    return 0;
}
