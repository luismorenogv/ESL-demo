./your_program "test:/path/to/video.mp4"

g++ main_example.cpp img_proc_test.cpp -std=c++11 -o green_tracker     `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0` -lstdc++

ffmpeg -i input.avi -pix_fmt yuyv422 -vcodec rawvideo -an output_yuy2.avi