#include "vision.hpp"

Mat preprocess(Mat input){
    //static string dst3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.169 port=8003 sync=false";
    //static VideoWriter writer3(dst3, 0, (double)30, Size(640, 90), false);

    Mat cut_gray;
    cvtColor(input(Rect(0, 270, 640, 90)), cut_gray, COLOR_BGR2GRAY);
    cut_gray = cut_gray + (Scalar(160) - mean(cut_gray));
    GaussianBlur(cut_gray, cut_gray, Size(9, 9), 3, 3);
    //writer3 << cut_gray;

    threshold(cut_gray, cut_gray, 190, 256, THRESH_BINARY);
    morphologyEx(cut_gray, cut_gray, MORPH_CLOSE, getStructuringElement(0, Size(7, 7)));
    
    return cut_gray;
}


int calc_err(Mat gray_img, int prev_error, bool isLeft) {
    static string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.169 port=8002 sync=false";
    static VideoWriter writer2(dst2, 0, (double)30, Size(320, 90), true);

    static string dst3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.169 port=8003 sync=false";
    static VideoWriter writer3(dst3, 0, (double)30, Size(320, 90), true);

    Mat labels, stats, centroids;
    int cnt = connectedComponentsWithStats(gray_img, labels, stats, centroids);

    Mat dst;
    cvtColor(gray_img, dst, COLOR_GRAY2BGR);
    static Point center_variable_l(160, 45);
    static Point center_variable_r(160, 45);
    Rect fit;

    static int error = 0;
    int selected;
    int sel_cnt=0;

    for (int i = 1; i < cnt; i++) {
        int* p = stats.ptr<int>(i);
        if ((p[4] < 200) || (p[4]) > 20000) continue;
        Rect r(p[0], p[1], p[2], p[3]);
        
        //putText(dst, format("%d", i), Point(p[0] + 25, p[1] + 25), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 2);

        Point r_center = (r.br() + r.tl()) * 0.5;
        drawMarker(dst, r_center, Scalar(0, 0, 255), 0, 10, 2, 8);
        
        if (p[1]+p[3] > 85) {
            if(prev_error == 0) prev_error = 160 - r_center.x;
            int diff = (-1*prev_error) - (160 - r_center.x);
            //cout << "diff " << diff << endl;
            if (isLeft && diff < 100 && diff > -100 && norm(center_variable_l-r_center) < 70) {
                error = 160 - r_center.x;
                rectangle(dst, r, Scalar(0, 255, 255), 2);
                //drawMarker(dst, findCentroid(cut_gray, r), Scalar(255, 128, 0), 1, 10, 2, 8);
                center_variable_l = r_center;
                selected = i;
                sel_cnt++;
            }
            else if (!isLeft && diff < 100 && diff > -100 && norm(center_variable_r-r_center) < 70) {
                error = 160 - r_center.x;
                rectangle(dst, r, Scalar(0, 255, 255), 2);
                //drawMarker(dst, findCentroid(cut_gray, r), Scalar(255, 128, 0), 1, 10, 2, 8);
                center_variable_r = r_center;
                selected = i;
                sel_cnt++;
            }
            else {
                rectangle(dst, r, Scalar(255, 255, 0), 2);
            }
        }
        else {
            rectangle(dst, r, Scalar(255, 255, 0), 2);
        }
    }
    if(sel_cnt >= 2) cout << "[WARN] MORE THAN TWO CANDIDATE! lastsel=" << selected << "/cnt=" << sel_cnt << endl;
    
    if(isLeft) drawMarker(dst, center_variable_l, Scalar(255, 128, 255), 3, 10, 2, 8);
    else drawMarker(dst, center_variable_r, Scalar(255, 128, 255), 3, 10, 2, 8);

    if (isLeft) writer2 << dst;
    else writer3 << dst;
    return error;
}