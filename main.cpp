#include "vision.hpp"

bool ctrl_c_pressed = false;
void ctrlc_handler(int) { ctrl_c_pressed = true; }

int main(void)
{
    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.169 port=8001 sync=false";
    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);

    string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    //VideoCapture source(src, CAP_GSTREAMER);

    VideoCapture source("./vid/lanefollow_100rpm_cw.mp4");

    Mat frame;

    Dxl mx;
    struct timeval start, end1;
    double time1;
    int vel1 = 0, vel2 = 0;
    signal(SIGINT, ctrlc_handler);
    if (!mx.open()) { cout << "dynamixel open error" << endl; return -1; }

    static int prev_error_l = 0;
    static int error_l = 0;
    static int prev_error_r = 0;
    static int error_r = 0;
    int error = 0;

    bool allow_to_start = false;

    int def_speed = 100;
    double k = 0.25;

    while (true)
    {
        gettimeofday(&start, NULL);

        if (mx.kbhit()) //키보드입력 체크
        {
           char c = mx.getch(); //키입력 받기
           if (c == 's') allow_to_start = true;
        }

        source >> frame;
        if (frame.empty()) { source.set(CAP_PROP_POS_FRAMES, 0); source >> frame; /*cerr << "frame empty!" << endl; break;*/ }
        writer1 << frame;

        Mat cut_gray = preprocess(frame);

        error_l = -1 * calc_err(cut_gray(Rect(0,0,320,90)), prev_error_l, true);
        error_r = -1 * calc_err(cut_gray(Rect(320,0,320,90)), prev_error_r, false);

        error = (error_l+error_r)/2;

        if (allow_to_start){
            vel1 = def_speed + (error*k);
            vel2 = -1 * (def_speed - (error*k));
        }

        if(!mx.setVelocity(vel1,vel2)){ cout << "setVelocity error"<<endl; return -1;}

        if (ctrl_c_pressed) break; //Ctrl+c입력시 탈출
        usleep(20 * 1000);
        gettimeofday(&end1, NULL);
        time1 = end1.tv_sec - start.tv_sec + (end1.tv_usec - start.tv_usec) / 1000000.0;
        cout << "vel1:" << vel1 << ',' << "vel2:" << vel2 << ",time:" << time1 << ", error: " << error << endl;
        prev_error_l = error_l;
        prev_error_r = error_r;
    }
    mx.close(); // 장치닫기
    return 0;
}
