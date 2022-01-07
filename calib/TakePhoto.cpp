/**
 * 拍照程序
 */
#include "../include/camera.hpp"

int main(int argc, char **argv)
{
    //OpenDevice:

    // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
	libcaer::devices::davis davisHandle = libcaer::devices::davis(1);

	// Let's take a look at the information we have on the device.
	struct caer_davis_info davis_info = davisHandle.infoGet();

	// printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info.deviceString,
	// 	davis_info.deviceID, davis_info.deviceIsMaster, davis_info.dvsSizeX, davis_info.dvsSizeY,
	// 	davis_info.logicVersion);

	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	davisHandle.sendDefaultConfig();

	// Tweak some biases, to increase bandwidth in this case. 
	struct caer_bias_coarsefine coarseFineBias;//定义一个畸变，具体各个值的含义可以去头文件看

	coarseFineBias.coarseValue        = 2; 
	coarseFineBias.fineValue          = 116;
	coarseFineBias.enabled            = true;
	coarseFineBias.sexN               = false;
	coarseFineBias.typeNormal         = true;
	coarseFineBias.currentLevelNormal = true;

	davisHandle.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias));

	coarseFineBias.coarseValue        = 1;
	coarseFineBias.fineValue          = 33;
	coarseFineBias.enabled            = true;
	coarseFineBias.sexN               = false;
	coarseFineBias.typeNormal         = true;
	coarseFineBias.currentLevelNormal = true;

	davisHandle.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias));

	// Let's verify they really changed!
	// uint32_t prBias   = davisHandle.configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP);
	// uint32_t prsfBias = davisHandle.configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP);

	// printf("New bias values --- PR-coarse: %d, PR-fine: %d, PRSF-coarse: %d, PRSF-fine: %d.\n",
	// 	caerBiasCoarseFineParse(prBias).coarseValue, caerBiasCoarseFineParse(prBias).fineValue,
	// 	caerBiasCoarseFineParse(prsfBias).coarseValue, caerBiasCoarseFineParse(prsfBias).fineValue);

	// Now let's get start getting some data from the device. We just loop in blocking mode,
	// no notification needed regarding new events. The shutdown notification, for example if
	// the device is disconnected, should be listened to.
	davisHandle.dataStart(nullptr, nullptr, nullptr, nullptr, nullptr);

	// Let's turn on blocking data-get mode to avoid wasting resources.
	davisHandle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

	davisHandle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, false);
	davisHandle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, false);
	davisHandle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, 4200);

    //construct the windows:
	cv::namedWindow("PLOT_FRAME",
		cv::WindowFlags::WINDOW_AUTOSIZE | cv::WindowFlags::WINDOW_KEEPRATIO | cv::WindowFlags::WINDOW_GUI_EXPANDED);



    int count = 0;
    int limit = 30;
    int conti_flag = 0;
    std::cout << "输入要拍的照片张数: ";
    std::cin >> limit;
    std::cout << "请按s键拍照 " << limit << " 次\n";



	while (1) {
		std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = davisHandle.dataGet();
		if (packetContainer == nullptr) {
			continue; // Skip if nothing there.
		}

		// printf("\nGot event container with %d packets (allocated).\n", packetContainer->size());

		for (auto &packet : *packetContainer) {
			if (packet == nullptr) {
				// printf("Packet is empty (not present).\n");
				continue; // Skip if nothing there.
			}

			// printf("Packet of type %d -> %d events, %d capacity.\n", packet->getEventType(), packet->getEventNumber(),
				// packet->getEventCapacity());

			if (packet->getEventType() == FRAME_EVENT) {
				std::shared_ptr<const libcaer::events::FrameEventPacket> frame
					= std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);

				// Get full timestamp, and sum all pixels of first frame event.
				const libcaer::events::FrameEvent &firstEvent = (*frame)[0];

				int32_t ts   = firstEvent.getTimestamp();
				uint64_t sum = 0;

				for (int32_t y = 0; y < firstEvent.getLengthY(); y++) {
					for (int32_t x = 0; x < firstEvent.getLengthX(); x++) {
						sum += firstEvent.getPixel(x, y);
					}
				}

				// printf("First frame event - ts: %d, sum: %" PRIu64 ".\n", ts, sum);

				for (const auto &f : *frame) {
					if (f.getROIIdentifier() != 0) {
						continue;
					}

					cv::Mat cvFrame = f.getOpenCVMat(false);
					// Simple display, just use OpenCV GUI.
					cv::imshow("PLOT_FRAME", cvFrame);
                    // cvFrame.convertTo(cvFrame, CV_8UC1);

                    char c = cv::waitKey(1);
                    if (c == 'q' || c == 27)
                        break;
                    else if(c == 's')
                    {
                        cv::imwrite(std::to_string(count) + ".jpg", cvFrame / 255);
                        count++;
                        std::cout << "拍了 " << count << " 次\n";
                    }
                    if(count == limit)
                        return 0;
                    
				}
			}

		}
	}

	davisHandle.dataStop();
	// Close automatically done by destructor.

	cv::destroyWindow("PLOT_EVENTS");
	cv::destroyWindow("PLOT_FRAME");
    return 0;
}
