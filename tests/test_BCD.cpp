#include<iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include<opencv2/opencv.hpp>



int main(int argc , char* argv[])
{
    cv::Mat gray_img = cv::imread("../maps/map.png",cv::IMREAD_GRAYSCALE);
    if(gray_img.empty()){
        std::cout<<"EMPTY \n";
    }
    else{
        std::cout<<"GOT THE IMAGE \n";
    }

    std::cout<<gray_img.size()<<"\n";


    for (int i = 0 ; i< gray_img.rows ; i++){
        for(int j = 0 ; j <gray_img.cols; j++){
            uchar pixel = gray_img.at<uchar>(i,j);

        }
    }


    cv::imshow("map", gray_img);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}