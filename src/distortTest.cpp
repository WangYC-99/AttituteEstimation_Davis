#include "../include/camera.hpp"

using namespace std;

int main()
{

	vector<cv::Point> points_selected(3);

	// Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
	libcaer::devices::davis davisHandle = libcaer::devices::davis(1);

	// Let's take a look at the information we have on the device.
	struct caer_davis_info davis_info = davisHandle.infoGet();

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

	davisHandle.dataStart(nullptr, nullptr, nullptr, nullptr, nullptr);

	// Let's turn on blocking data-get mode to avoid wasting resources.
	davisHandle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

	davisHandle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, false);
	davisHandle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, false);
	davisHandle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, 4200);

    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); // 摄像机内参数矩阵
    cv::Mat distCoeffs   = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); // 摄像机的5个畸变系数：k1,k2,p1,p2,k3

    // cameraMatrix.at<float>(0, 0) = 2.0051109271105071e+02;
    // cameraMatrix.at<float>(0, 3) = 1.0320481858056947e+02;
    // cameraMatrix.at<float>(1, 1) = 2.0051109271105071e+02;
    // cameraMatrix.at<float>(1, 2) = 9.5269574917876866e+01;
    // cameraMatrix.at<float>(2, 2) = 1.;
	
    cameraMatrix.at<float>(0, 0) = 1.9567466282961288e+02;
    cameraMatrix.at<float>(0, 2) = 1.0577070896553398e+02;
    cameraMatrix.at<float>(1, 1) = 1.9534670253508108e+02;
    cameraMatrix.at<float>(1, 2) = 9.6975360024586649e+01;
    cameraMatrix.at<float>(2, 2) = 1.;


    // distCoeffs.at<float>(0, 0) = -4.1493955882402356e-01;
    // distCoeffs.at<float>(0, 1) = 2.2858315476539787e-01;
    // distCoeffs.at<float>(0, 2) = 3.7861257763060283e-06;
    // distCoeffs.at<float>(0, 3) = 8.1589213734131001e-04;
    // distCoeffs.at<float>(0, 4) = -7.8462959681429245e-0; 


    distCoeffs.at<float>(0, 0) = -4.0544743015005236e-01;
    distCoeffs.at<float>(0, 1) = 1.9259010066228921e-01;
    distCoeffs.at<float>(0, 2) = 8.1365636792228959e-04;
    distCoeffs.at<float>(0, 3) = -4.1489788374319701e-04;
    distCoeffs.at<float>(0, 4) = -4.6558748509152897e-02; 

	//    data: [ -4.0544743015005236e-01, 1.9259010066228921e-01,
    //    8.1365636792228959e-04, -4.1489788374319701e-04,
    //    -4.6558748509152897e-02 ]
    
    cout << "camera:" << endl;
    for(int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cout << "row:" << i << " col:" << j << " value:" << cameraMatrix.at<float>(i, j) << endl;
        }
    }
    
    cout << "dis:" << endl;
    for(int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            cout << "row:" << i << " col:" << j << " value:" << distCoeffs.at<float>(i, j) << endl;
        }
    }
    
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
                    cv::Mat newFrame;
					cv::imshow("origin", cvFrame);
                    cv::undistort(cvFrame, newFrame, cameraMatrix, distCoeffs);
					cv::imshow("distorted", newFrame);
					// Simple display, just use OpenCV GUI
					cv::waitKey(1);
				}
			}

		}
	}

	davisHandle.dataStop();
	// Close automatically done by destructor.

	// printf("Shutdown successful.\n");
    return 0;
}

