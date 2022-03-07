/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * sobel_cv_stage.cpp - Sobel filter implementation, using OpenCV
 */

#include <libcamera/stream.h>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;

using Stream = libcamera::Stream;

class MyPostProcessStage : public PostProcessingStage
{
public:
	MyPostProcessStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	int ksize_ = 3;
};

#define NAME "my_post_process"

char const *MyPostProcessStage::Name() const
{
	return NAME;
}

void MyPostProcessStage::Read(boost::property_tree::ptree const &params)
{
	ksize_ = params.get<int16_t>("ksize", 3);
}

void MyPostProcessStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("MyPostProcessStage: only YUV420 format supported");
}

// Gaussian
// Sobel
// Scharr

struct Image
{
  int Rows;
  int Cols;
  uint8_t* Data;
  uint8_t Type;
};

// pitch = size of row (including num pixels, channels)
// bpp = bytes per pixel
void Convolve(struct Image &In, struct Image &Mask, struct Image &Out, int pitch, int bpp)
{
  long i,j,m,n,idx,jdx;
  int ms,im,val;
  // uint32_t* tmp;
  for (i = 0; i < In.Rows;++i)
  {
    for (j = 0; j < In.Cols;++j)
    {
      val = 0;

			// performing convolution
      for (m=0;m<Mask.Rows;++m)
      {
        for (n=0;n<Mask.Cols;++n)
        {
					// get mask factor
          ms = (int8_t)*(Mask.Data + m*Mask.Rows+n);

					// get current mask index into image
          idx = i - m;
          jdx = j - n;
          if (idx >= 0 && jdx >= 0)
          {
            im = *(In.Data + idx*pitch + jdx*bpp);
            val += ms*im;
          }
        }
      }

			val = (uint32_t)((float)(val) / (float)16);
      if (val > 255) val = 255;
      if (val < 0) val = 0;
      // tmp = (uint32_t*)(Out.Data + i*pitch + j*bpp);
      // *tmp = (uint32_t)((val << 16) | (val << 8) | (val));
    }
  }
}

void my_post_process(cv::Mat& src)
{
	uint8_t *myData = src.data;
	Image in, out, mask;
	in.Data = myData;
	in.Rows = src.rows;
	in.Cols = src.cols;
	std::cout << "Rows: " << in.Rows << std::endl;
	std::cout << "Cols: " << in.Cols << std::endl;
	out.Data = (uint8_t*)malloc(sizeof(uint8_t)*in.Rows*in.Cols);
	out.Rows = in.Rows;
	out.Cols = in.Cols;
	std::vector<uint8_t> filter = 
	{
		1,2,1,
		2,4,2,
		1,2,1
	};
	mask.Data = filter.data();
	mask.Rows = 3;
	mask.Cols = 3;
	Convolve(in, mask, out, 1, out.Cols);

	// std::swap(src.data,out.Data);
	// uint8_t *myData = src.data;
	// int x,y;
	// float noise, theta;
	// for (int i = 0; i < src.rows; ++i)
	// {
	// 	for (int j = 0; j < src.cols; ++j)
	// 	{
	// 		noise = sqrt(-2)
	// 		// v[0] = 0;
	// 		// v[1] = 255;
	// 		// v[2] = 0;
	// 	}
	// }
}

bool MyPostProcessStage::Process(CompletedRequestPtr &completed_request)
{
	StreamInfo info = app_->GetStreamInfo(stream_);
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
	uint8_t *ptr = (uint8_t *)buffer.data();

	//Everything beyond this point is image processing...

	uint8_t value = 128;
	int num = (info.stride * info.height) / 2;
	Mat src = Mat(info.height, info.width, CV_8U, ptr, info.stride);
	// int scale = 1;
	// int delta = 0;
	// int ddepth = CV_16S;

	memset(ptr + info.stride * info.height, value, num);

	my_post_process(src);
	// Remove noise by blurring with a Gaussian filter ( kernal size = 3 )
	// GaussianBlur(src, src, Size(111, 111), 0, 0, BORDER_DEFAULT);

	// Mat grad_x, grad_y;

	// // //Scharr(src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT);
	// // Sobel(src, grad_x, ddepth, 1, 0, ksize_, scale, delta, BORDER_DEFAULT);
	// // //Scharr(src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT);
	// // Sobel(src, grad_y, ddepth, 0, 1, ksize_, scale, delta, BORDER_DEFAULT);

	// // converting back to CV_8U
	// convertScaleAbs(grad_x, grad_x);
	// convertScaleAbs(grad_y, grad_y);

	// //weight the x and y gradients and add their magnitudes
	// addWeighted(grad_x, 0.5, grad_y, 0.5, 0, src);

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new MyPostProcessStage(app);
}

static RegisterStage reg(NAME, &Create);