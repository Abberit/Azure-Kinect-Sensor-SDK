// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#define _CRT_SECURE_NO_WARNINGS

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <k4arecord/playback.h>
#include <string>
#include <fstream>
#include "transformation_helpers.h"
#include "turbojpeg.h"
#include <opencv2/highgui.hpp>
#include <iostream>

/*
static bool point_cloud_color_to_depth(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string file_name)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                               depth_image,
                                                                               color_image,
                                                                               transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              depth_image,
                                                                              K4A_CALIBRATION_TYPE_DEPTH,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, transformed_color_image, file_name.c_str());

    k4a_image_release(transformed_color_image);
    k4a_image_release(point_cloud_image);

    return true;
}
*/

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string file_name)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              transformed_depth_image,
                                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}

long WriteToFile(const char *fileName, void *buffer, size_t bufferSize)
{
    printf("Saving file '%s' of the size '%zu'\n", fileName, bufferSize);
    // std::cout << bufferSize << std::endl;
    // assert(buffer != NULL);

    std::ofstream hFile;
    hFile.open(fileName, std::ios::out | std::ios::trunc | std::ios::binary);
    if (hFile.is_open())
    {
        hFile.write((char *)buffer, static_cast<std::streamsize>(bufferSize));
        hFile.close();
    }
    // std::cout << "[Streaming Service] Color frame is stored in " << fileName << std::endl;

    return 0;
}

uint16_t getDepthInMM(int x, int y, size_t rowSizeInBytes, uint8_t *depthBuffer)
{
    uint16_t *depthBufferInMM = (uint16_t *)depthBuffer;
    size_t rowSizeInUInt16 = rowSizeInBytes / 2;
    depthBufferInMM += (uint8_t)y * rowSizeInUInt16 + (uint8_t)x;

    return *depthBufferInMM;
}

void locator(int event, int x, int y, int flags, void *userdata)
{
    std::ignore = flags;
    k4a_image_t transformed_depth_image = *((k4a_image_t *)userdata);

    size_t rowSizeInBytes = k4a_image_get_stride_bytes(transformed_depth_image);
    uint8_t *depthBuffer = k4a_image_get_buffer(transformed_depth_image);

    uint16_t depthInMM = getDepthInMM(x, y, rowSizeInBytes, depthBuffer);

    if (event == cv::EVENT_MOUSEMOVE)
    {
        printf("At (%d, %d) the depth is: %d\n", x, y, depthInMM);
    }
}

static bool point_cloud_depth_to_color2(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string output_dir)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    cv::Mat cv_image_no_alpha;
    cv::Mat cv_image_with_alpha;
    std::string file_name;
    cv::String windowTitle = "AzureKinectDK";

    bool result = false;

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        goto Cleanup;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        goto Cleanup;
    }

    cv_image_with_alpha = cv::Mat(color_image_height_pixels,
                                  color_image_width_pixels,
                                  CV_8UC4,
                                  (void *)k4a_image_get_buffer(color_image));
    cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);

    cv::namedWindow(windowTitle);
    cv::setMouseCallback(windowTitle, locator, &transformed_depth_image);
    cv::imshow(windowTitle, cv_image_no_alpha);
    cv::waitKey(0);

    printf("Saving screenshot to '%s'...\n", output_dir.c_str());

    file_name = output_dir + "/image.720p.bin";
    WriteToFile(file_name.c_str(), k4a_image_get_buffer(color_image), k4a_image_get_size(color_image));

    file_name = output_dir + "/depth.1024x1024.raw.bin";
    WriteToFile(file_name.c_str(), k4a_image_get_buffer(depth_image), k4a_image_get_size(depth_image));

    file_name = output_dir + "/depth.720p.bin";
    WriteToFile(file_name.c_str(),
                k4a_image_get_buffer(transformed_depth_image),
                k4a_image_get_size(transformed_depth_image));

    result = true;

Cleanup:
    k4a_image_release(transformed_depth_image);
    return result;
}

static int capture(std::string output_dir, uint8_t deviceId = K4A_DEVICE_DEFAULT)
{
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 10000;
    k4a_transformation_t transformation = NULL;
    k4a_transformation_t transformation_color_downscaled = NULL;
    k4a_capture_t capture = NULL;
    std::string file_name = "";
    uint32_t device_count = 0;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    //k4a_image_t color_image_downscaled = NULL;

    device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceId, &device))
    {
        printf("Failed to open device\n");
        goto Exit;
    }

    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras\n");
        goto Exit;
    }

    // Get a capture
    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        goto Exit;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        goto Exit;
    }

    // Get a depth image
    depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
    {
        printf("Failed to get depth image from capture\n");
        goto Exit;
    }

    // Get a color image
    color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        goto Exit;
    }

    /*
    // Compute color point cloud by warping color image into depth camera geometry
#ifdef _WIN32
    file_name = output_dir + "\\color_to_depth.ply";
#else
    file_name = output_dir + "/color_to_depth.ply";
#endif

    if (point_cloud_color_to_depth(transformation, depth_image, color_image, file_name.c_str()) == false)
    {
        goto Exit;
    }
    */

    // Compute color point cloud by warping depth image into color camera geometry
    
    if (point_cloud_depth_to_color2(transformation, depth_image, color_image, output_dir.c_str()) == false)
    {
        goto Exit;
    }

    /*
    // Compute color point cloud by warping depth image into color camera geometry
#ifdef _WIN32
    file_name = output_dir + "\\depth_to_color.ply";
#else
    file_name = output_dir + "/depth_to_color.ply";
#endif
    if (point_cloud_depth_to_color(transformation, depth_image, color_image, file_name.c_str()) == false)
    {
        goto Exit;
    }
    */

    /*
    // Compute color point cloud by warping depth image into color camera geometry with downscaled color image and
    // downscaled calibration. This example's goal is to show how to configure the calibration and use the
    // transformation API as it is when the user does not need a point cloud from high resolution transformed depth
    // image. The downscaling method here is naively to average binning 2x2 pixels, user should choose their own
    // appropriate downscale method on the color image, this example is only demonstrating the idea. However, no matter
    // what scale you choose to downscale the color image, please keep the aspect ratio unchanged (to ensure the
    // distortion parameters from original calibration can still be used for the downscaled image).
    k4a_calibration_t calibration_color_downscaled;
    memcpy(&calibration_color_downscaled, &calibration, sizeof(k4a_calibration_t));
    calibration_color_downscaled.color_camera_calibration.resolution_width /= 2;
    calibration_color_downscaled.color_camera_calibration.resolution_height /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cx /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cy /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fx /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fy /= 2;
    transformation_color_downscaled = k4a_transformation_create(&calibration_color_downscaled);
    color_image_downscaled = downscale_image_2x2_binning(color_image);
    if (color_image_downscaled == 0)
    {
        printf("Failed to downscaled color image\n");
        goto Exit;
    }

#ifdef _WIN32
    file_name = output_dir + "\\depth_to_color_downscaled.ply";
#else
    file_name = output_dir + "/depth_to_color_downscaled.ply";
#endif
    if (point_cloud_depth_to_color(transformation_color_downscaled,
                                   depth_image,
                                   color_image_downscaled,
                                   file_name.c_str()) == false)
    {
        goto Exit;
    }
    */

    returnCode = 0;

Exit:
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    if (transformation_color_downscaled != NULL)
    {
        k4a_transformation_destroy(transformation_color_downscaled);
    }
    if (device != NULL)
    {
        k4a_device_close(device);
    }
    return returnCode;
}

// Timestamp in milliseconds. Defaults to 1 sec as the first couple frames don't contain color
static int playback(char *input_path, int timestamp = 1000, std::string output_filename = "output.ply")
{
    int returnCode = 1;
    k4a_playback_t playback = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t uncompressed_color_image = NULL;

    k4a_result_t result;
    k4a_stream_result_t stream_result;

    // Open recording
    result = k4a_playback_open(input_path, &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("Failed to open recording %s\n", input_path);
        goto Exit;
    }

    result = k4a_playback_seek_timestamp(playback, timestamp * 1000, K4A_PLAYBACK_SEEK_BEGIN);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to seek timestamp %d\n", timestamp);
        goto Exit;
    }
    printf("Seeking to timestamp: %d/%d (ms)\n",
           timestamp,
           (int)(k4a_playback_get_recording_length_usec(playback) / 1000));

    stream_result = k4a_playback_get_next_capture(playback, &capture);
    if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL)
    {
        printf("Failed to fetch frame\n");
        goto Exit;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

    // Fetch frame
    depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
    {
        printf("Failed to get depth image from capture\n");
        goto Exit;
    }

    color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        goto Exit;
    }

    ///////////////////////////////
    // Convert color frame from mjpeg to bgra
    k4a_image_format_t format;
    format = k4a_image_get_format(color_image);
    if (format != K4A_IMAGE_FORMAT_COLOR_MJPG)
    {
        printf("Color format not supported. Please use MJPEG\n");
        goto Exit;
    }

    int color_width, color_height;
    color_width = k4a_image_get_width_pixels(color_image);
    color_height = k4a_image_get_height_pixels(color_image);

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 color_width,
                                                 color_height,
                                                 color_width * 4 * (int)sizeof(uint8_t),
                                                 &uncompressed_color_image))
    {
        printf("Failed to create image buffer\n");
        goto Exit;
    }

    tjhandle tjHandle;
    tjHandle = tjInitDecompress();
    if (tjDecompress2(tjHandle,
                      k4a_image_get_buffer(color_image),
                      static_cast<unsigned long>(k4a_image_get_size(color_image)),
                      k4a_image_get_buffer(uncompressed_color_image),
                      color_width,
                      0, // pitch
                      color_height,
                      TJPF_BGRA,
                      TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
    {
        printf("Failed to decompress color frame\n");
        if (tjDestroy(tjHandle))
        {
            printf("Failed to destroy turboJPEG handle\n");
        }
        goto Exit;
    }
    if (tjDestroy(tjHandle))
    {
        printf("Failed to destroy turboJPEG handle\n");
    }
    ///////////////////////////////

    // Compute color point cloud by warping depth image into color camera geometry
    if (point_cloud_depth_to_color(transformation, depth_image, uncompressed_color_image, output_filename) == false)
    {
        printf("Failed to transform depth to color\n");
        goto Exit;
    }

    returnCode = 0;

Exit:
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (uncompressed_color_image != NULL)
    {
        k4a_image_release(uncompressed_color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    return returnCode;
}

static void print_usage()
{
    printf("Usage: transformation_example capture <output_directory> [device_id]\n");
    printf("Usage: transformation_example playback <filename.mkv> [timestamp (ms)] [output_file]\n");
}

int main(int argc, char **argv)
{
    int returnCode = 0;

    /*
    * This is test case of reading depth information from serialized file.
 
    FILE *f = fopen("E:\\camera_output\\sample\\depth.bin", "rb+");
    if (f)
    {
        fseek(f, 0L, SEEK_END);
        long filesize = ftell(f);          // get file size
        fseek(f, 0L, SEEK_SET);            // go back to the beginning
        uint8_t *buffer = new uint8_t[filesize]; // allocate the read buf
        fread(buffer, 1, filesize, f);
        fclose(f);

        // Do what you want with file data
        int x = 0;
        int y = 0;

        uint16_t depth = getDepthInMM(x, y, 1280 * 2, buffer);
        printf("Detected depth for (%d, %d): %d\n", x, y, depth);

        delete[] buffer;
    }
    */

    if (argc < 2)
    {
        char c;
        bool exit = false;
        do
        {
            returnCode = capture("E:/camera_output");
            printf("Screenshot is taken. Press ESC to exit, any other key to continue with another screenshot.\n");
            if (!std::cin.get(c) || c == 27)
            {
                exit = true;
            }
        } while (!exit);
        
        //print_usage();
    }
    else
    {
        std::string mode = std::string(argv[1]);
        if (mode == "capture")
        {
            if (argc == 3)
            {
                returnCode = capture(argv[2]);
            }
            else if (argc == 4)
            {
                returnCode = capture(argv[2], (uint8_t)atoi(argv[3]));
            }
            else
            {
                print_usage();
            }
        }
        else if (mode == "playback")
        {
            if (argc == 3)
            {
                returnCode = playback(argv[2]);
            }
            else if (argc == 4)
            {
                returnCode = playback(argv[2], atoi(argv[3]));
            }
            else if (argc == 5)
            {
                returnCode = playback(argv[2], atoi(argv[3]), argv[4]);
            }
            else
            {
                print_usage();
            }
        }
        else
        {
            print_usage();
        }
    }

    cv::destroyAllWindows();

    return returnCode;
}
