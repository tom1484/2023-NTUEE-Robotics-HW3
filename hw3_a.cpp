#include <cstdio>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
  // Check number of arguments.
  // If no argument input, exit the program.
  if (argc != 2) {
    exit(1);
  }

  cv::Mat SrcImage, WorkImage, ComImage;

  // Load a gray scale picture.
  SrcImage = cv::imread(argv[1], 0);
  if (!SrcImage.data) {
    exit(1);
  }

  // Create windows for debug.
  cv::namedWindow("SrcImage", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("WorkImage", cv::WINDOW_AUTOSIZE);

  // Show the source image.
  cv::imshow("SrcImage", SrcImage);
  cv::waitKey();

  // Duplicate the source iamge.
  WorkImage = SrcImage.clone();

  // Extract the contour of
  /* If you're familiar with OpenCV, findContours() will be a better way.*/
  cv::GaussianBlur(WorkImage, WorkImage, cv::Size(3, 3), 0, 0);
  cv::threshold(WorkImage, WorkImage, 128, 255, cv::THRESH_BINARY);

  // Opening
  cv::erode(WorkImage, WorkImage, cv::Mat());
  cv::dilate(WorkImage, WorkImage, cv::Mat());

  // Duplicate the working iamge.
  ComImage = WorkImage.clone();

  // Show the working image after preprocessing.
  cv::imshow("WorkImage", WorkImage);
  cv::waitKey();

  return 0;
}
