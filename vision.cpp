#include "vision.hpp"

Mat preprocess(Mat input){
    static string dst3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.169 port=8003 sync=false";
    static VideoWriter writer3(dst3, 0, (double)30, Size(640, 90), false);

    Mat cut_gray;
    cvtColor(input(Rect(0, 270, 640, 90)), cut_gray, COLOR_BGR2GRAY);
    cut_gray = cut_gray + (Scalar(160) - mean(cut_gray));
    GaussianBlur(cut_gray, cut_gray, Size(9, 9), 3, 3);
    writer3 << cut_gray;

    threshold(cut_gray, cut_gray, 190, 256, THRESH_BINARY);
    morphologyEx(cut_gray, cut_gray, MORPH_CLOSE, getStructuringElement(0, Size(7, 7)));
    
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
    Rect fit_l;
    Rect fit_r;

    static int error = 0;
    int selected_l;
    int selected_r;
    int sel_cnt=0;

    for (int i = 1; i < cnt; i++) {
        int* p = stats.ptr<int>(i);
        Rect r(p[0], p[1], p[2], p[3]);
        
        //putText(dst, format("%d", i), Point(p[0] + 25, p[1] + 25), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 2);

        Point r_center = (r.br() + r.tl()) * 0.5;
        drawMarker(dst, r_center, Scalar(0, 0, 255), 0, 10, 2, 8);
        //L
        if (p[1]+p[3] > 80) {
            int diff = (-1*center_variable_l.x) - (160 - r_center.x);
            if (diff < 180 && diff > -180 && norm(center_variable_l-r_center) < 200) {
                //drawMarker(dst, findCentroid(cut_gray, r), Scalar(255, 128, 0), 1, 10, 2, 8);
                center_variable_l = r_center;
                selected_l = i;
                fit_l = r;
                sel_cnt++;
            }
        }
        //R
        if (p[1]+p[3] > 80) {
            int diff = (-1*center_variable_r.x) - (160 - r_center.x);
            if (diff < 180 && diff > -180 && norm(center_variable_r-r_center) < 200) {
                //drawMarker(dst, findCentroid(cut_gray, r), Scalar(255, 128, 0), 1, 10, 2, 8);
                center_variable_r = r_center;
                selected_r = i;
                fit_r = r;
                sel_cnt++;
            }
        }

    }
    if(sel_cnt >= 2) cout << "[INFO] MORE THAN TWO CANDIDATE. sel=" << selected_l << "," << selected_r << "/cnt=" << sel_cnt << endl;
    if(!fit_l.empty()) rectangle(dst, fit_l, Scalar(0, 255, 255), 2);
    if(!fit_r.empty()) rectangle(dst, fit_r, Scalar(0, 255, 255), 2);
    drawMarker(dst, center_variable_l, Scalar(255, 128, 255), 3, 10, 2, 8);
    drawMarker(dst, center_variable_r, Scalar(255, 128, 255), 3, 10, 2, 8);
    writer2 << dst;
    return error = 0;
}