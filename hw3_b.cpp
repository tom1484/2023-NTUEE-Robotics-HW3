#include <opencv2/core/hal/interface.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv) {
  // Check number of arguments.
  // If no argument input, exit the program.
  if (argc != 2) {
    exit(1);
  }

  cv::Mat srcImage, workImage, comImage;

  // Load a gray scale picture.
  srcImage = cv::imread(argv[1], cv::IMREAD_COLOR);
  if (!srcImage.data) {
    exit(1);
  }

  // Duplicate the source iamge.
  workImage = cv::Mat(cv::Size(0, 0), CV_8U);
  cv::cvtColor(srcImage, workImage, cv::COLOR_BGR2GRAY);

  // Extract the contour of
  /* If you're familiar with OpenCV, findContours() will be a better way.*/
  cv::GaussianBlur(workImage, workImage, cv::Size(3, 3), 0, 0);
  cv::threshold(workImage, workImage, 128, 255, cv::THRESH_BINARY);

  // Opening
  cv::erode(workImage, workImage, cv::Mat());
  cv::dilate(workImage, workImage, cv::Mat());

  // Duplicate the working iamge.
  comImage = workImage.clone();

  // Connected Component Analysis
  cv::Mat labelImage(cv::Size(0, 0), CV_32S);
  cv::Mat stats(cv::Size(0, 0), CV_32S);
  cv::Mat centroids(cv::Size(0, 0), CV_32S);

  cv::connectedComponentsWithStats(comImage, labelImage, stats, centroids);
  labelImage.convertTo(labelImage, CV_8U);

  // Filter out non-object components
  int areaThreshold = 2000;
  vector<bool> isObject(stats.rows, false);
  vector<cv::Point_<double>> objectCentroids(stats.rows);

  for (int i = 1; i < stats.rows; i++) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) > areaThreshold) {
      isObject[i] = true;
      objectCentroids[i] = cv::Point_<double>(centroids.at<double>(i, 0),
                                              centroids.at<double>(i, 1));
    }
  }

  // Calculate pricipal angles
  vector<double> angles(stats.rows, 0.0);

  for (int i = 1; i < stats.rows; i++) {
    if (!isObject[i]) {
      continue;
    }

    cv::Moments moment = cv::moments(labelImage == i, true);
    angles[i] = 0.5 * atan2(2 * moment.mu11, moment.mu20 - moment.mu02);
  }

  // Draw principal axes
  cv::Mat resultImage = srcImage.clone();
  for (int i = 1; i < stats.rows; i++) {
    if (!isObject[i]) {
      continue;
    }

    cv::Point_<double> center = objectCentroids[i];
    cv::Point_<double> end1(center.x + 10000 * cos(angles[i]),
                            center.y + 10000 * sin(angles[i]));
    cv::Point_<double> end2(center.x - 10000 * cos(angles[i]),
                            center.y - 10000 * sin(angles[i]));

    cv::line(resultImage, end1, end2, cv::Scalar(0, 255, 0), 2, cv::FILLED);

    cv::circle(resultImage, objectCentroids[i], 5, cv::Scalar(0, 0, 255),
               cv::FILLED);
  }

  cv::imshow("ComImage", resultImage);

  cv::waitKey();

  return 0;
}
