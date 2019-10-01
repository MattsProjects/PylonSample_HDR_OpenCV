/*
// PylonSample_HDR_OpenCV_Advanced.cpp
//
// Demostrates using OpenCV's Exposure Fusion tools with Basler cameras and drivers to create an HDR image.
// This sample uses the camera's Sequencer feature to setup the camera to take multiple different images with one trigger.
//
// Copyright (c) 2019 Matthew Breit - matt.breit@baslerweb.com or matt.breit@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Uses OpenCV libraries. License information can be found here:
// https://opencv.org/license/
*/

// Include files to use OpenCV
// This Sample uses OpenCV 3.0
// The Visual Studio project uses static OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

// DEMO: Additional library for showing "progress bar" image.
#include "../include/StitchImage.h"

// STD libraries needed
#include <vector>

// This determines what kind of camera we will use
#define USE_USB

#if defined( USE_1394 )
// Settings for using Basler IEEE 1394 cameras.
#include <pylon/1394/Basler1394InstantCamera.h>
typedef Pylon::CBasler1394InstantCamera Camera_t;
typedef Pylon::CBasler1394GrabResultPtr GrabResultPtr_t;
using namespace Basler_IIDC1394CameraParams;
#elif defined ( USE_GIGE )
// Settings for using Basler GigE cameras.
#include <pylon/gige/BaslerGigEInstantCamera.h>
typedef Pylon::CBaslerGigEInstantCamera Camera_t;
typedef Pylon::CBaslerGigEGrabResultPtr GrabResultPtr_t;
using namespace Basler_GigECameraParams;
#elif defined ( USE_CAMERALINK )
// Settings for using Basler Camera Link cameras.
#include <pylon/cameralink/BaslerCameraLinkInstantCamera.h>
typedef Pylon::CBaslerCameraLinkInstantCamera Camera_t;
using namespace Basler_CLCameraParams;
#elif defined ( USE_USB )
// Settings for using Basler USB cameras.
#include <pylon/usb/BaslerUsbInstantCamera.h>
typedef Pylon::CBaslerUsbInstantCamera Camera_t;
typedef Pylon::CBaslerUsbGrabResultPtr GrabResultPtr_t;
using namespace Basler_UsbCameraParams;
#else
#error Camera type is not specified. For example, define USE_GIGE for using GigE cameras.
#endif

// DEMO: Number of individual images to be grabbed before shutting down.
static const uint32_t c_countOfImagesToGrab = 1000;
// Number of individual images per HDR image
static const uint32_t c_imagesPerHDR = 3;
// Lowest exposure time we will use for HDR (in microseconds)
static const double c_lowExposureTime = 100;
// Highest exposure time we will use for HDR (in microseconds)
static const double c_highExposureTime = 100000;

using namespace std;

// The function which will generate the "HDR" image from a set of images.
void CreateHDR(std::vector<Pylon::CPylonImage> &images, Pylon::CPylonImage &OutputImage)
{
	// we will use pylon's image format converter to convert the image to openCV format.
	Pylon::CImageFormatConverter myConverter;
	Pylon::PixelType openCVPixelType = Pylon::EPixelType::PixelType_BGR8packed;
	myConverter.OutputPixelFormat.SetValue(openCVPixelType);

	// Step 1: Convert all stored pylon images to opencv format
	std::vector<cv::Mat> cv_images;
	for (int i = 0; i < images.size(); i++)
	{
		Pylon::CPylonImage convertedImage;
		myConverter.Convert(convertedImage, images[i]);
		cv::Mat cv_image(convertedImage.GetHeight(), convertedImage.GetWidth(), CV_8UC3, (uint8_t*)convertedImage.GetBuffer());
		cv_images.push_back(cv_image.clone());
	}

	// merge_mertens will perform the exposure fusion to get the HDR image
	cv::Ptr<cv::MergeMertens> mergeMertens = cv::createMergeMertens();
	cv::Ptr<cv::AlignMTB> alignMTB = cv::createAlignMTB();

	// Step 2: align the images (in case the camera moved. But this decreases speed and modifies the final image size)
	// OPTIMIZATION: If speed is preferred over image quality, comment this out.
	// alignMTB->process(cv_images, cv_images);

	// Step 3: Create the HDR image
	cv::Mat fusion;
	mergeMertens->process(cv_images, fusion);
	cv::Mat hdrMat;
	fusion.convertTo(hdrMat, CV_8UC3, 255);

	// Step 4: recovert the HDR image to a pylon image and display it
	Pylon::CPylonImage hdrImage;
	hdrImage.AttachUserBuffer(hdrMat.data, (hdrMat.total() * hdrMat.elemSize()), openCVPixelType, hdrMat.cols, hdrMat.rows, 0);
	OutputImage.CopyImage(hdrImage);

	// Step 5: Clean up for the next HDR image
	cv_images.clear();
}

int main(int argc, char* argv[])
{
	// The exit code of the sample application.
	int exitCode = 0;

	// Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
	// is initialized during the lifetime of this object.
	Pylon::PylonAutoInitTerm autoInitTerm;

	try
	{
		// ********************************** BEGIN SETUP **********************************

		// Use a DeviceInfo object to open a specific camera.
		Pylon::CDeviceInfo info;
		info.SetSerialNumber("21792244");

		// Create an instant camera object with the given info.
		Camera_t camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice(info));

		// Print the model name of the camera.
		std::cout << "Using device " << camera.GetDeviceInfo().GetModelName() << std::endl;

		// Open the camera to gain access to parameters
		camera.Open();

		// calculate the exposure time increments for the subsequent images
		double c_exposureTimeIncrement = (c_highExposureTime - c_lowExposureTime) / c_imagesPerHDR;

		// ********************************** BEGIN SEQUENCER SETUP **********************************

		// check if camera supports the sequencer first
		if (GenApi::IsWritable(camera.SequencerMode.GetNode()) == false)
		{
			cout << "This camera does not support the Sequencer feature. Exiting..." << endl;
			return 1;
		}

		// Turn off the sequencer so we can configure it
		camera.SequencerMode.FromString("Off");

		// Put the sequencer into configuration mode so we can cofigure it
		camera.SequencerConfigurationMode.FromString("On");

		for (int i = 0; i < c_imagesPerHDR; i++)
		{
			// We will start by configuring sequencer set 0
			camera.SequencerSetSelector.SetValue(i);

			// Now change some camera settings settings
			// The first image will have the low exposure, the last the highest, and the ones in between have an increment
			if (i == 0)
				camera.ExposureTime.SetValue(c_lowExposureTime);
			else if (i == (c_imagesPerHDR - 1))
				camera.ExposureTime.SetValue(c_highExposureTime);
			else
				camera.ExposureTime.SetValue(camera.ExposureTime.GetValue() + c_exposureTimeIncrement);

			// We will advance to the next sequencer set when this one has acquired it's image.
			// and we will cycle back to the first sequence set after we've run through all of them.
			if (i == (c_imagesPerHDR - 1))
				camera.SequencerSetNext.SetValue(0);
			else
				camera.SequencerSetNext.SetValue(i + 1);

			camera.SequencerPathSelector.SetValue(1);
			
			// Save the sequence set
			camera.SequencerSetSave.Execute();
		}

		// Now point the sequencer to start at the first set
		camera.SequencerSetSelector.SetValue(0);

		// Take the sequencer out of configuration mode
		camera.SequencerConfigurationMode.FromString("Off");

		// Turn on the sequencer
		camera.SequencerMode.FromString("On");

		// ********************************** END SEQUENCER SETUP **********************************

		// Setup the trigger mechanism
		camera.TriggerSelector.SetValue(TriggerSelector_FrameBurstStart);
		camera.AcquisitionBurstFrameCount.SetValue(c_imagesPerHDR);
		camera.TriggerMode.SetValue(TriggerMode_On);
		camera.TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Software);
	
		Pylon::CPylonImage image; // Pylon image to hold an individual incoming image
		std::vector<Pylon::CPylonImage> images; // vector to store the incoming images we will process

		// DEMO: We can show the user a 'progress bar' of individual images stitched together.
		Pylon::CPylonImage stitchedImage;		

		// how we will keep track of the images
		int imageCounter = 0;

		// GRAB ENGINE: The Grab Engine will receive the incoming images into buffers and hold them for retrieval.
		// Access to the image is through a "Grab Result", which hold the image and other information.
		// MaxNumBuffer can be used to control how many buffers are used in the Grab Engine.
		// If you see statistics about "missed images" or "buffer underruns", increase this value (default is 10).
		camera.MaxNumBuffer = 10;

		// This smart pointer points to the "Grab Result" provided by the Grab Engine.
		GrabResultPtr_t ptrGrabResult;

		// ********************************** END SETUP **********************************

		// Start the Grab Engine (StopGrabbing() will be called automatically when c_countOfImagesToGrab have been grabbed).
		camera.StartGrabbing(c_countOfImagesToGrab);

		// Send a kickoff trigger. Subsequent triggers will be sent later in the Grab Loop.
		camera.TriggerSoftware.Execute();

		// GRAB LOOP: Here we will retrieve images from the Grab Engine and process them.
		while (camera.IsGrabbing())
		{
			// Retrieve a "Grab Result" from the Grab Engine. If nothing shows up by the timeout end, throw an exception.
			camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

			// Does the Grab Result actually contain an image?
			if (ptrGrabResult->GrabSucceeded())
			{
				imageCounter++;
				std::cout << "Image " << imageCounter << " Retrieved." << std::endl;

				// Store this image.
				image.CopyImage(ptrGrabResult);
				images.push_back(image);

				// DEMO: we can show the user a 'progress bar' by stitching images side by side
				std::string errorMessage = "";
				StitchImage::StitchToRight(stitchedImage, image, &stitchedImage, errorMessage);
				Pylon::DisplayImage(1, stitchedImage);
				if (imageCounter == c_imagesPerHDR)
					stitchedImage.Release();
			}
			else
			{
				// The grab result failed. Show the error message that came with it.
				std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
			}
					
			// Once we have all the images, do HDR processing	
			if (images.size() == c_imagesPerHDR)
			{
				// First, we can now send another trigger to the camera to get a head start on the next batch of images.
				cout << "Received all images. Sending trigger for next batch..." << endl;
				camera.TriggerSoftware.Execute();

				// Create and display the HDR Image.
				std::cout << "Generating HDR Image for current batch..." << std::endl;
				Pylon::CPylonImage hdrImage;
				CreateHDR(images, hdrImage);
				Pylon::DisplayImage(0, hdrImage);
				std::cout << "HDR Image Generated!" << std::endl;
				std::cout << std::endl;

				// Clean up for the next run
				imageCounter = 0;
				images.clear();
			}
		}
	}
	catch (GenICam::GenericException &e)
	{
		// Error handling.
		std::cerr << "An exception occurred." << std::endl
			<< e.GetDescription() << std::endl;
		exitCode = 1;
	}

	// Comment the following two lines to disable waiting on exit.
	std::cerr << std::endl << "Press Enter to exit." << std::endl;
	while (std::cin.get() != '\n');

	return exitCode;
}
