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


#ifndef SIFT_GPU_FEATURE_DETECTOR_H
#define SIFT_GPU_FEATURE_DETECTOR_H

#ifdef USE_SIFT_GPU
#include <opencv2/features2d/features2d.hpp>
#include <opencv/cv.h>
#include <SiftGPU/SiftGPU.h>

class SiftGPUFeatureDetector {
public:
	virtual ~SiftGPUFeatureDetector();
	float* detect( const cv::Mat& image, cv::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = cv::Mat()) const;
	static SiftGPUFeatureDetector* GetInstance();

private:
	SiftGPUFeatureDetector();
	void CVMatToSiftGPU(const cv::Mat& image, unsigned char* siftImage) const;

	void WritePGM(FILE *fp, unsigned char* data, int width, int height);

	static const int imageWidth = 640;
	static const int imageHeight = 480;

	static SiftGPUFeatureDetector* instance;

	bool error;
	SiftGPU* siftgpu;
	unsigned char* data;	//image as texture
};

#endif
#endif
