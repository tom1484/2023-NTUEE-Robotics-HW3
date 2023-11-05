#include <cstdio>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
namespace fs = std::__fs::filesystem;

void calibrateCamera(string path, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
  vector<vector<cv::Point3f>> objectPoints;
  vector<vector<cv::Point2f>> imagePoints;

  vector<cv::Point3f> obj;
  vector<cv::Point2f> corners;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 8; j++) {
      obj.push_back(cv::Point3f(j, i, 0));
    }
  }

  for (const auto &entry : fs::directory_iterator(path)) {
    string filePath = entry.path().string();
    string filename = fs::path(filePath).filename();
    string yaml = fs::path(filename).extension();

    if ((yaml != ".png" && yaml != ".jpg") || filename.find("calibrate_") != 0) {
      continue;
    }
    cout << filePath << ": ";

    cv::Mat gray = cv::imread(filePath, cv::IMREAD_GRAYSCALE);
    bool ret = cv::findChessboardCorners(gray, cv::Size(8, 6), corners,
                                         cv::CALIB_CB_ADAPTIVE_THRESH +
                                             cv::CALIB_CB_NORMALIZE_IMAGE +
                                             cv::CALIB_CB_FAST_CHECK);

    cout << ret << endl;
    if (ret) {
      objectPoints.push_back(obj);
      imagePoints.push_back(corners);

      // cv::cornerSubPix(
      //     gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
      //     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
      //     30,
      //                      0.1));
      //
      // cv::Mat draw(cv::Size(0, 0), CV_8UC3);
      // cv::cvtColor(gray, draw, cv::COLOR_GRAY2BGR);
      //
      // cv::drawChessboardCorners(draw, cv::Size(8, 6), corners, ret);
      // cv::imshow("image", draw);
      // cv::waitKey();
    }
  }

  vector<cv::Mat> rvecs, tvecs;
  cv::calibrateCamera(objectPoints, imagePoints, cv::Size(8, 6), cameraMatrix,
                      distCoeffs, rvecs, tvecs);
}

void undistortImages(string path, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
  string undistortedDir = (fs::path(path) / fs::path("undistorted")).string();
  if (!fs::exists(undistortedDir)) {
    fs::create_directory(undistortedDir);
  }

  for (const auto &entry : fs::directory_iterator(path)) {
    string filePath = entry.path().string();
    string filename = fs::path(filePath).filename();
    string yaml = fs::path(filename).extension();

    if (yaml != ".png" && yaml != ".jpg") {
      continue;
    }
    string exportPath =
        (fs::path(undistortedDir) / fs::path(filename)).string();
    cout << filePath << " -> " << exportPath << endl;

    cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);
    cv::Mat undistorted;
    cv::undistort(image, undistorted, cameraMatrix, distCoeffs);

    cv::imwrite(exportPath, undistorted);

    // cv::imshow(filePath, image);
    // cv::imshow("undistorted", undistorted);
    // cv::waitKey();
  }
}

int main(int argc, char **argv) {
  // Check number of arguments.
  // If no argument input, exit the program.
  if (argc != 3) {
    exit(1);
  }

  string arg1Ext = fs::path(argv[1]).extension();
  string arg2Ext = fs::path(argv[2]).extension();

  if (arg1Ext == ".yaml" && arg2Ext == "") {
    cv::FileStorage file(argv[1], cv::FileStorage::READ);
    cv::Mat cameraMatrix, distCoeffs;

    file["cameraMatrix"] >> cameraMatrix;
    file["distCoeffs"] >> distCoeffs;
    file.release();

    undistortImages(argv[2], cameraMatrix, distCoeffs);
  } else if (arg1Ext == "" && arg2Ext == ".yaml") {
    cv::Mat cameraMatrix, distCoeffs;
    calibrateCamera(argv[1], cameraMatrix, distCoeffs);

    cv::FileStorage file(argv[2], cv::FileStorage::WRITE);

    file << "cameraMatrix" << cameraMatrix;
    file << "distCoeffs" << distCoeffs;
    file.release();
  } else {
    exit(1);
  }

  return 0;
}
