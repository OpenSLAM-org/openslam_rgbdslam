/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifdef USE_SIFT_GPU
#include "sift_gpu_feature_detector.h"
#include <GL/gl.h>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>

using namespace cv;

SiftGPUFeatureDetector* SiftGPUFeatureDetector::instance = NULL;

SiftGPUFeatureDetector::SiftGPUFeatureDetector() {
	error = false;
	siftgpu = new SiftGPU();
	char * argv[] = {"-cuda"};
	siftgpu->ParseParam(1, argv);

	if (siftgpu->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
		ROS_ERROR("Can't create OpenGL context! SiftGPU cannot be used.");
		error = true;
	}

	data = (unsigned char*) malloc(imageWidth * imageHeight);
}

SiftGPUFeatureDetector::~SiftGPUFeatureDetector() {
	delete siftgpu;
	instance = NULL;
	if (data != NULL) {
		free(data);
		data = NULL;
	}
}

/*
 * For singleton
 */
SiftGPUFeatureDetector* SiftGPUFeatureDetector::GetInstance() {
	if (instance == NULL) {
		ROS_INFO("Create Instance");
		instance = new SiftGPUFeatureDetector();
	}
	return instance;
}

/*
 * Detect the features and storem them into keypoints.
 * The descriptors are computed and saved in descriptors.
 */
float* SiftGPUFeatureDetector::detect(const cv::Mat& image, cv::vector<cv::KeyPoint>& keypoints, const Mat& mask) const {
	if (error) {
		keypoints.clear();
		ROS_ERROR("SiftGPU cannot be used. Detection of keypoints failed");
		return 0;
	}

	//get image
	CVMatToSiftGPU(image, data);

	int num_features = 0;
	SiftGPU::SiftKeypoint* keys = 0;

	float* descriptors = 0;
	ROS_INFO("SIFTGPU: cols: %d, rows: %d", image.cols, image.rows);
	if (siftgpu->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE,
			GL_UNSIGNED_BYTE)) {
		num_features = siftgpu->GetFeatureNum();
		keys = new SiftGPU::SiftKeypoint[num_features];
		descriptors = new float[128 * num_features];
		siftgpu->GetFeatureVector(&keys[0], &descriptors[0]);
	} else {
		ROS_WARN("SIFTGPU->RunSIFT() failed!");
	}

	//copy to opencv structure
	keypoints.clear();
	for (int i = 0; i < num_features; ++i) {
		KeyPoint key(keys[i].x, keys[i].y, keys[i].s, keys[i].o);
		keypoints.push_back(key);
	}

//	FILE *fp = fopen("bla.pgm", "w");
//	WritePGM(fp, data, image.cols, image.rows);
//	fclose(fp);

	return descriptors;
}

/*
 * Converts a cv matrix into an OpenGL texture array
 */
void SiftGPUFeatureDetector::CVMatToSiftGPU(const Mat& image,
		unsigned char* siftImage) const {
	Mat tmp;
	image.convertTo(tmp, CV_8U);
	for (int y = 0; y < tmp.rows; ++y) {
		for (int x = 0; x < tmp.cols; ++x) {
			siftImage[y * tmp.cols + x] = tmp.at<unsigned char> (y, x);
		}
	}
}

/*
 * For debug
 */
void SiftGPUFeatureDetector::WritePGM(FILE *fp, unsigned char* data, int width, int height)
{
    int val;
    fprintf(fp, "P5\n%d %d\n255\n", width, height);

    for (int y = 0; y < height; y++){
		for (int x = 0; x < width; x++) {
			val = (int) (/*255.0 */ data[y * width + x]);
			if (x == 0) val = 255;
			if (y == 0) val = 255;
			fputc(MAX(0, MIN(255, val)), fp);
		}
	}
}
#endif
