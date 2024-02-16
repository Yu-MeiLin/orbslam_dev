#include "rgbd-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System *SLAM) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{

    subImu_ = this->create_subscription<ImuMsg>("/imu", 1000, std::bind(&RgbdInertialNode::GrabImu, this, _1));
    subRgb_ = this->create_subscription<ImageMsg>("camera/rgb", 100, std::bind(&RgbdInertialNode::GrabRGB, this, _1));
    subDepth_ = this->create_subscription<ImageMsg>("camera/depth", 100, std::bind(&RgbdInertialNode::GrabDepth, this, _1));

    syncThread_ = new std::thread(&RgbdInertialNode::SyncWithImu, this);
}

RgbdInertialNode::~RgbdInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void RgbdInertialNode::GrabRGB(const ImageMsg::SharedPtr msgRGB)
{
    bufMutexRgb_.lock();

    if (!imgRgbBuf_.empty())
        imgRgbBuf_.pop();
    imgRgbBuf_.push(msgRGB);

    bufMutexRgb_.unlock();
}

void RgbdInertialNode::GrabDepth(const ImageMsg::SharedPtr msgDepth)
{
    bufMutexDepth_.lock();

    if (!imgDepthBuf_.empty())
        imgDepthBuf_.pop();
    imgDepthBuf_.push(msgDepth);

    bufMutexDepth_.unlock();
}

cv::Mat RgbdInertialNode::GetImage(const ImageMsg::SharedPtr msg, const std::string cvType)
{
     //Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, cvType);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void RgbdInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imRgb, imDepth;
        double tImRgb = 0, tImDepth = 0;
        if (!imgRgbBuf_.empty() && !imgDepthBuf_.empty() && !imuBuf_.empty())
        {
            tImRgb = Utility::StampToSec(imgRgbBuf_.front()->header.stamp);
            tImDepth = Utility::StampToSec(imgDepthBuf_.front()->header.stamp);


            bufMutexRgb_.lock();
            while ((tImDepth - tImRgb) > maxTimeDiff && imgRgbBuf_.size() > 1)
            {
                imgRgbBuf_.pop();
                tImRgb = Utility::StampToSec(imgRgbBuf_.front()->header.stamp);
            }
            bufMutexRgb_.unlock();
            
            bufMutexDepth_.lock();
            while ((tImRgb - tImDepth) > maxTimeDiff && imgDepthBuf_.size() > 1)
            {
                imgDepthBuf_.pop();
                tImDepth = Utility::StampToSec(imgDepthBuf_.front()->header.stamp);
            }
            bufMutexDepth_.unlock();

            

            if ((tImRgb - tImDepth) > maxTimeDiff || (tImDepth - tImRgb) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImRgb > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexRgb_.lock();
            imRgb = GetImage(imgRgbBuf_.front(), sensor_msgs::image_encodings::MONO8);
            imgRgbBuf_.pop();
            bufMutexRgb_.unlock();

            bufMutexDepth_.lock();
            imDepth = GetImage(imgDepthBuf_.front(),  sensor_msgs::image_encodings::MONO8);
            imgDepthBuf_.pop();
            bufMutexDepth_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImRgb)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imRgb, imRgb);
                clahe_->apply(imDepth, imDepth);
            }

            // cv::Size dsize = cv::Size(960, 540);
            // cv::Mat rgb_resize;
            // cv::resize(imRgb,rgb_resize,dsize,0,0, cv::INTER_NEAREST);
            // cv::Mat depth_resize;
            // cv::resize(imDepth,depth_resize,dsize,0,0, cv::INTER_NEAREST);
            SLAM_->TrackRGBD(imRgb, imDepth, tImRgb, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
