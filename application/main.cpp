/*
 * Copyright (c) 2015  Juan M. Falquez,
 *                     University of Colorado - Boulder
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif
#include <opencv2/opencv.hpp>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <HAL/Utils/GetPot>
#include <HAL/Camera/CameraDevice.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <gSLIC/FastImgSeg.h>

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  GetPot cl_args(argc, argv);

  ///----- Initialize Camera.
  if (!cl_args.search("-cam")) {
    std::cerr << "Camera arguments missing!" << std::endl;
    exit(EXIT_FAILURE);
  }
  hal::Camera camera(cl_args.follow("", "-cam"));

  const int image_width = camera.Width();
  const int image_height = camera.Height();
  std::cout << "- Image Dimensions: " << image_width <<
               "x" << image_height << std::endl;

  ///----- Initialize gSLIC.
  const double  weight = 10.0;
  const int     number_segments = 1000;

  FastImgSeg segmenter;

  segmenter.initializeFastSeg(image_width, image_height, number_segments);

  // Image holder.
  std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();

  /////////////////////////////////////////////////////////////////////////////
  ///---- MAIN LOOP
  ///
  int key = 0;
  while (key != 27) {

    // Capture new image.
    camera.Capture(*images);
    cv::Mat image = images->at(0)->Mat().clone();

    // Convert to 4 channels, since code expects (in future versions)
    // depth on last channel.
    if (image.type() == CV_8UC1) {
      cv::cvtColor(image, image, CV_GRAY2RGBA);
    } else if (image.type() == CV_8UC3) {
      cv::cvtColor(image, image, CV_RGB2RGBA);
    } else {
      std::cerr << "Unknown image format. Aborting..." << std::endl;;
      exit(EXIT_FAILURE);
    }

    // Load image.
    segmenter.LoadImg(image.data);

    // Segment.
    segmenter.DoSegmentation(RGB_SLIC, weight);

    // Get marked image.
    segmenter.Tool_GetMarkedImg();
    cv::Mat marked_image(image_height, image_width, CV_8UC4, segmenter.markedImg);

    // Show images.
    cv::imshow("Image", image);
    cv::imshow("Superpixels", marked_image);

    // Render. If "escape" key pressed, exit.
    key = cv::waitKey(1e3);
  }

  return 0;
}
