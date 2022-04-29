#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <chrono>
//#include <iostream>
//#include <fstream>

#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)
#define RGB2Y(R, G, B) CLIP(( ( 66*(R) + 129*(G) + 25*(B) + 128) >>8 ) + 16)
#define RGB2U(R, G, B) CLIP(( ( -38*(R) - 74*(G) + 112*(B) + 128) >>8) + 128)
#define RGB2V(R, G, B) CLIP((( 112*(R) - 94*(G) - 18*(B) + 128) >> 8) + 128)



using namespace cv;
using namespace std;
//using namespace std::chrono;


namespace ros_app_convert
{
    /**c
     * @brief  Convert ROS warpper class
     */

    class Convert
    {
    public:
        /**
         * @brief { Convert RGB TO UYVY }
         *
         */

        Convert(ros::NodeHandle *nh, ros::NodeHandle *private_nh)
        {
            std::string subImgTopic;
            std::string pubImgTopic;

            private_nh->param("subscribe_image_topic",  subImgTopic, std::string(""));
            private_nh->param("publish_image_topic",  pubImgTopic,  std::string(""));
            private_nh->param("convert",  convert,  std::string(""));
            private_nh->param("type",  type,  std::string(""));

            image_transport::ImageTransport it(*nh);
            m_imgPub = it.advertise(pubImgTopic,10);
            if (convert=="uyvy" || convert=="yuv422")
            {
                if (type=="compressed")
                {
                    m_imgSub = nh->subscribe<sensor_msgs::CompressedImage>(subImgTopic,1, &Convert::callback_compressed_uyvy, this);
                }

                else
                {
                    m_imgSub = nh->subscribe<sensor_msgs::Image>(subImgTopic,1, &Convert::callback_raw_uyvy, this);
                }

            }

            else if (convert=="raw" || convert=="rgb")
            {
                if (type=="compressed")
                {
                    m_imgSub = nh->subscribe<sensor_msgs::CompressedImage>(subImgTopic,1, &Convert::callback_compressed_rgb2rgb_raw, this);
                }
            }

            else if (convert=="nv12")
            {
                yuv_image = cv::Mat::zeros(cv::Size(width,height*1.5),CV_8UC1);
                if (type=="compressed")
                {
                    m_imgSub = nh->subscribe<sensor_msgs::CompressedImage>(subImgTopic,1, &Convert::callback_compressed_nv12, this);
                }

                else
                {
                    m_imgSub = nh->subscribe<sensor_msgs::Image>(subImgTopic,1, &Convert::callback_raw_nv12, this);
                }
            }


            ros::spin();
        }

        ~Convert()
        {
        }

        void callback_compressed_rgb2rgb_raw(const sensor_msgs::CompressedImageConstPtr& image_ptr)
        {
            image = cv::imdecode(cv::Mat(image_ptr->data),1);
	        cv::cvtColor(image,image,COLOR_BGR2RGB);
            width = image.cols;
            height = image.rows;
            rgb2rgb_raw(image);
        }

        void callback_compressed_uyvy(const sensor_msgs::CompressedImageConstPtr& image_ptr)
        {
            image = cv::imdecode(cv::Mat(image_ptr->data),1);
            width = image.cols;
            height = image.rows;
            if (yuv_mat_not_init)
            {
                yuv_image = cv::Mat::zeros(cv::Size(width,height),CV_8UC2);
                yuv_mat_not_init = false;
            }
            rgb2uyuv(image);
        }

        void callback_raw_uyvy(const sensor_msgs::ImageConstPtr& image_ptr)
        {
            image = cv_bridge::toCvShare(image_ptr,"bgr8")->image;
            width = image.cols;
            height = image.rows;
            if (yuv_mat_not_init)
            {
                yuv_image = cv::Mat::zeros(cv::Size(width,height),CV_8UC2);
                yuv_mat_not_init = false;
            }
            rgb2uyuv(image);
        }

        void callback_compressed_nv12(const sensor_msgs::CompressedImageConstPtr& image_ptr)
        {
            //auto start = high_resolution_clock::now();
            image = cv::imdecode(cv::Mat(image_ptr->data),1);
            //cv::cvtColor(image,image,cv::COLOR_BGR2YUV_YV12);
            width = image.cols;
            height = image.rows;
            if (yuv_mat_not_init)
            {
                yuv_image = cv::Mat::zeros(cv::Size(width,height*1.5),CV_8UC1);
                yuv_mat_not_init = false;
            }
            rgb2nv12(image);
            //auto stop = high_resolution_clock::now();
            //auto duration = duration_cast<microseconds>(stop-start);
            //std::cout << duration.count() << std::endl;
        }

        void callback_raw_nv12(const sensor_msgs::ImageConstPtr& image_ptr)
        {
            image = cv_bridge::toCvShare(image_ptr,"bgr8")->image;
            //cv::cvtColor(image,image,cv::COLOR_BGR2YUV_YV12);
            width = image.cols;
            height = image.rows;
            if (yuv_mat_not_init)
            {
                yuv_image = cv::Mat::zeros(cv::Size(width,height*1.5),CV_8UC1);
                yuv_mat_not_init = false;
            }
            rgb2nv12(image);
        }


        void rgb2rgb_raw(const cv::Mat& image)
        {
            try
            {
                header.stamp = ros::Time::now();
                out_msg = cv_bridge::CvImage(header,"rgb8",image).toImageMsg();
		        m_imgPub.publish(out_msg);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("Could not convert image!");
            }
        }

        void rgb2uyuv(const cv::Mat& image)
        {
            try
            {
		        convertRGB2UYVY(image,yuv_image);
                header.stamp = ros::Time::now();
                out_msg = cv_bridge::CvImage(header,"8UC2",yuv_image).toImageMsg();
		        m_imgPub.publish(out_msg);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("Could not convert image!");
            }

        }

        void rgb2nv12(const cv::Mat& image)
        {
            try
            {
		        //convertYV122NV12(image,yuv_image);
                convertRGB2NV12(image,yuv_image);
		        header.stamp = ros::Time::now();
                out_msg = cv_bridge::CvImage(header,"8UC1",yuv_image).toImageMsg();
		        m_imgPub.publish(out_msg);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("Could not convert image!");
            }

        }


        void convertRGB2UYVY(const cv::Mat& img, cv::Mat& yuv)
        {
            for (int ih=0;ih<img.rows;ih++)
            {
                const uint8_t* imgRowPtr = img.ptr<uint8_t>(ih);
                uint8_t* yuvRowPtr = yuv.ptr<uint8_t>(ih);
                for (int iw=0;iw<img.cols;iw+=2)
                {
                    const int imgColIdxBytes = iw*img.elemSize();
                    const int yuvColIdxBytes = iw*yuv.elemSize();

                    const uint8_t B1 = imgRowPtr[imgColIdxBytes+0];
                    const uint8_t G1 = imgRowPtr[imgColIdxBytes+1];
                    const uint8_t R1 = imgRowPtr[imgColIdxBytes+2];
                    const uint8_t B2 = imgRowPtr[imgColIdxBytes+3];
                    const uint8_t G2 = imgRowPtr[imgColIdxBytes+4];
                    const uint8_t R2 = imgRowPtr[imgColIdxBytes+5];

                    const int Y = RGB2Y(R1,G1,B1);
                    const int U = RGB2U(R1,G1,B1);
                    const int V = RGB2V(R1,G1,B1);
                    const int Y2 = RGB2Y(R2,G2,B2);


                    yuvRowPtr[yuvColIdxBytes+0]=cv::saturate_cast<uint8_t>(U);
                    yuvRowPtr[yuvColIdxBytes+1]=cv::saturate_cast<uint8_t>(Y);
                    yuvRowPtr[yuvColIdxBytes+2]=cv::saturate_cast<uint8_t>(V);
                    yuvRowPtr[yuvColIdxBytes+3]=cv::saturate_cast<uint8_t>(Y2);

                }
            }
        }

      void convertRGB2NV12(const cv::Mat& img, cv::Mat& yuv)
        {
            
            int counter = 0;
            for (int ih=0;ih<img.rows;ih+=2)
            {
                const uint8_t* imgRowPtr_0 = img.ptr<uint8_t>(ih);
                const uint8_t* imgRowPtr_1 = img.ptr<uint8_t>(ih+1);
                uint8_t* yuvRowPtr_0 = yuv.ptr<uint8_t>(ih);
                uint8_t* yuvRowPtr_1 = yuv.ptr<uint8_t>(ih+1);
                int uv_row = counter+img.rows-1;

                uint8_t* yuvRowPtrUV = yuv.ptr<uint8_t>(uv_row);
                counter++;

                for (int iw=0;iw<img.cols;iw+=2)
                {
                    const int imgColIdxBytes = iw*img.elemSize();
                    const int yuvColIdxBytes = iw*yuv.elemSize();
                    const uint8_t B00 = imgRowPtr_0[imgColIdxBytes+0];
                    const uint8_t G00 = imgRowPtr_0[imgColIdxBytes+1];
                    const uint8_t R00 = imgRowPtr_0[imgColIdxBytes+2];
                    const uint8_t B01 = imgRowPtr_0[imgColIdxBytes+3];
                    const uint8_t G01 = imgRowPtr_0[imgColIdxBytes+4];
                    const uint8_t R01 = imgRowPtr_0[imgColIdxBytes+5];

                    const uint8_t B10 = imgRowPtr_1[imgColIdxBytes+0];
                    const uint8_t G10 = imgRowPtr_1[imgColIdxBytes+1];
                    const uint8_t R10 = imgRowPtr_1[imgColIdxBytes+2];
                    const uint8_t B11 = imgRowPtr_1[imgColIdxBytes+3];
                    const uint8_t G11 = imgRowPtr_1[imgColIdxBytes+4];
                    const uint8_t R11 = imgRowPtr_1[imgColIdxBytes+5];

                    const int Y00 = RGB2Y(R00,G00,B00);
                    const int Y01 = RGB2Y(R01,G01,B01);
                    const int Y10 = RGB2Y(R10,G10,B10);
                    const int Y11 = RGB2Y(R11,G11,B11);

                    const int U00 = RGB2U(R00,G00,B00);
                    const int U01 = RGB2U(R01,G01,B01);
                    const int U10 = RGB2U(R10,G10,B10);
                    const int U11 = RGB2U(R11,G11,B11);

                    const int V00 = RGB2V(R00,G00,B00);
                    const int V01 = RGB2V(R01,G01,B01);
                    const int V10 = RGB2V(R10,G10,B10);
                    const int V11 = RGB2V(R11,G11,B11);

                    const int avg_U = (U00+U01+U10+U11)>>2;
                    const int avg_V = (V00+V01+V10+V11)>>2;;


                    yuvRowPtr_0[yuvColIdxBytes]=cv::saturate_cast<uint8_t>(Y00);
                    yuvRowPtr_0[yuvColIdxBytes+1]=cv::saturate_cast<uint8_t>(Y01);
                    yuvRowPtr_1[yuvColIdxBytes]=cv::saturate_cast<uint8_t>(Y10);
                    yuvRowPtr_1[yuvColIdxBytes+1]=cv::saturate_cast<uint8_t>(Y11);
                    yuvRowPtrUV[yuvColIdxBytes] = cv::saturate_cast<uint8_t>(avg_U);
                    yuvRowPtrUV[yuvColIdxBytes+1] = cv::saturate_cast<uint8_t>(avg_V);

                }

            }
           
        }

         void convertYV122NV12(const cv::Mat& yv12, cv::Mat& nv12)
        {

            int stride = (int)yv12.step[0];
            yv12.copyTo(nv12);
            cv::Mat V = cv::Mat(cv::Size(width/2,height/2),CV_8UC1,(unsigned char*)yv12.data+stride*height,stride/2);
            cv::Mat U = cv::Mat(cv::Size(width/2,height/2),CV_8UC1,(unsigned char*)yv12.data+stride*height+(stride/2)*(height/2),stride/2);
            for (int row=0;row<height/2;row++)
            {
                for (int col=0;col<width/2;col++)
                {
                    nv12.at<uchar>(height+row,2*col) = U.at<uchar>(row,col);
                    nv12.at<uchar>(height+row,2*col + 1) = V.at<uchar>(row,col);
                }
            }
            
        }

    private:
        ros::Subscriber m_imgSub;
        image_transport::Publisher m_imgPub;
        sensor_msgs::ImagePtr out_msg;
        std_msgs::Header header;
        cv::Mat image;
        cv::Mat yuv_image;
        int32_t width;
        int32_t height;
        std::string convert;
        std::string type;
        bool yuv_mat_not_init = true;

    };
}

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "convert_node");
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        ros_app_convert::Convert convert(&nh, &private_nh);

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }
}
