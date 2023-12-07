#include "vision.hpp"

Mat preprocess(Mat input){
    static string dst3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.169 port=8003 sync=false";
    static VideoWriter writer3(dst3, 0, (double)30, Size(640, 90), false);

    Mat cut_gray;
    cvtColor(input(Rect(0, 270, 640, 90)), cut_gray, COLOR_BGR2GRAY);
    cut_gray = cut_gray + (Scalar(190) - mean(cut_gray));
    GaussianBlur(cut_gray, cut_gray, Size(9, 9), 3, 3);
    writer3 << cut_gray;

    threshold(cut_gray, cut_gray, 235, 256, THRESH_BINARY);
    morphologyEx(cut_gray, cut_gray, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
    
    return cut_gray;
}


int calc_err(Mat gray_img) {
    static string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.169 port=8002 sync=false";
    static VideoWriter writer2(dst2, 0, (double)30, Size(640, 90), true);

    Mat labels, stats, centroids;
    int cnt = connectedComponentsWithStats(gray_img, labels, stats, centroids);

    Mat dst;
    cvtColor(gray_img, dst, COLOR_GRAY2BGR);
    static Point center_variable_l(160, 45);
    static Point center_variable_r(160+320, 45);

    int min_norm_l = INT_MAX;
    int min_norm_r = INT_MAX;

    Rect sel_l;
    Rect sel_r;

    for (int i = 1; i < cnt; i++) {
        int* p = stats.ptr<int>(i);
        Rect r(p[0], p[1], p[2], p[3]);
        rectangle(dst, r, Scalar(150, 150, 150), 1, 8);

        Point r_center = (r.br() + r.tl()) * 0.5;
        int diff_length = norm(center_variable_l - r_center);
        drawMarker(dst, r_center, Scalar(0, 0, 255), 0, 10, 2, 8);
        //L
        if (min_norm_l > diff_length && p[1] + p[3] > 60 && diff_length < 120) {
            sel_l = r;
            min_norm_l = diff_length;
            center_variable_l = r_center;
        }
    }

    for (int i = 1; i < cnt; i++) {
        int* p = stats.ptr<int>(i);
        Rect r(p[0], p[1], p[2], p[3]);

        Point r_center = (r.br() + r.tl()) * 0.5;
        int diff_length = norm(center_variable_r - r_center);
        drawMarker(dst, r_center, Scalar(0, 0, 255), 0, 10, 2, 8);
        //R
        if (min_norm_r > diff_length && p[1] + p[3] > 60 && diff_length < 120) {
            sel_r = r;
            min_norm_r = diff_length;
            center_variable_r = r_center;
        }
    }

    if (!sel_l.empty()) rectangle(dst, sel_l, Scalar(0, 255, 255), 2);
    if (!sel_r.empty()) rectangle(dst, sel_r, Scalar(0, 255, 255), 2);
    drawMarker(dst, center_variable_l, Scalar(255, 128, 255), 3, 10, 2, 8);
    drawMarker(dst, center_variable_r, Scalar(255, 128, 255), 3, 10, 2, 8);
    line(dst, center_variable_l, center_variable_r, Scalar(255, 0, 128), 2);

    Point pt_err((center_variable_l.x+center_variable_r.x)/2, (center_variable_l.y+center_variable_r.y)/2);
    drawMarker(dst, pt_err, Scalar(200, 0, 0), 4, 10, 2, 8);
    writer2 << dst;
    return pt_err.x;
}