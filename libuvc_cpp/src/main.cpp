#include "libuvc/libuvc.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  frame->frame_format = UVC_FRAME_FORMAT_YUYV;
  std::cout << "Frame format" << std::endl;
  std::cout << frame->frame_format << std::endl; 
  uvc_frame_t *greyLeft;
  uvc_frame_t *greyRight;
  uvc_error_t retLeft;
  uvc_error_t retRight;

  /* We'll convert the image from YUV/JPEG to gray8, so allocate space */
  greyLeft = uvc_allocate_frame(frame->width * frame->height);
  greyRight = uvc_allocate_frame(frame->width * frame->height);
  if (!greyLeft) {
    printf("unable to allocate grey left frame!");
    return;
  }
  if (!greyRight) {
    printf("unable to allocate grey right frame!");
    return;
  }

  /* Do the BGR conversion */
  retLeft = uvc_yuyv2y(frame, greyLeft);
  retRight = uvc_yuyv2uv(frame, greyRight);
  if (retLeft) {
    uvc_perror(retLeft, "uvc_yuyv2y");
    uvc_free_frame(greyLeft);
    printf("Error with yuyv to y conversion");
    return;
  }
  if (retRight) {
    uvc_perror(retRight, "uvc_yuyv2uv");
    uvc_free_frame(greyRight);
    printf("Error with yuyv to uv conversion");
    return;
  }

  /* Call a user function:
   *
   * my_type *my_obj = (*my_type) ptr;
   * my_user_function(ptr, bgr);
   * my_other_function(ptr, bgr->data, bgr->width, bgr->height);
   */

  /* Call a C++ method:
   *
   * my_type *my_obj = (*my_type) ptr;
   * my_obj->my_func(bgr);
   */

  /* Use opencv.highgui to display the image:
   * 
   * cvImg = cvCreateImageHeader(
   *     cvSize(bgr->width, bgr->height),
   *     IPL_DEPTH_8U,
   *     3);
   *
   * cvSetData(cvImg, bgr->data, bgr->width * 3); 
   *
   * cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
   * cvShowImage("Test", cvImg);
   * cvWaitKey(10);
   *
   * cvReleaseImageHeader(&cvImg);
   */

  IplImage *cvImg, *cvImg2;
  cvImg = cvCreateImage(cvSize(greyLeft->width, greyLeft->height),
		  IPL_DEPTH_8U, 1);
  cvImg2 = cvCreateImage(cvSize(greyRight->width, greyRight->height),
		  IPL_DEPTH_8U, 1);
  cvImg->imageData = (char*) greyLeft->data; 
  cvImg2->imageData = (char*) greyRight->data;

  cv::Mat left (greyLeft->height, greyLeft->width, CV_8UC1, greyLeft->data);
  cv::Mat right (greyRight->height, greyRight->width, CV_8UC1, greyRight->data);
  //cv::Mat right (cv::cvarrToMat(cvImg2));

  cv::imshow("Test", left);
  cv::imshow("Test2", right);
  cvWaitKey(10);

  cvReleaseImage(&cvImg);
  cvReleaseImage(&cvImg2);

  uvc_free_frame(greyLeft);
  uvc_free_frame(greyRight);
}

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;
  uvc_device_t **list;

  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx, NULL);


  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }
  puts("UVC initialized");
  res = uvc_get_device_list(ctx, &list);
  if (res < 0) {
    uvc_perror(res, "uvc_get_device_list");
    return res;
  }

/*  for(int i = 0; i < 1; i++) {
	 res = uvc_open(list[i], &devh);
	 uvc_print_diag(devh, stderr);
	 uvc_close(devh);
  }
  uchar x;
  uvc_free_device_list(list, x);
*/
  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(
      ctx, &dev,
      0, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
  } else {
    puts("Device found");

    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
      printf("Unable to open device");
    } else {
      puts("Device opened");

      /* Print out a message containing all the information that libuvc
       * knows about the device */
      uvc_print_diag(devh, stderr);

      /* Try to negotiate a 640x480 30 fps YUYV stream profile */
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, /* result stored in ctrl */
          UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
          640, 480, 30 /* width, height, fps */
      );

      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
      } else {
        /* Start the video stream. The library will call user function cb:
         *   cb(frame, (void*) 12345)
         */
        res = uvc_start_streaming(devh, &ctrl, cb, (void*)12345, 0);

        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          puts("Streaming...");

          uvc_set_ae_mode(devh, 0); /* e.g., turn on auto exposure */

          sleep(10); /* stream for 10 seconds */

          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
          puts("Done streaming.");
        }
      }

      /* Release our handle on the device */
      uvc_close(devh);
      puts("Device closed");
    }

    /* Release the device descriptor */
    uvc_unref_device(dev);
  }

  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
  puts("UVC exited");

  return 0;
}
