#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0 //条件编译

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;//第一帧
double last_image_time = 0;
bool init_pub = 0;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 1.判断是否是第一帧
    if(first_image_flag)
    {
        first_image_flag = false;// 更新：不再是第一帧图像
        first_image_time = img_msg->header.stamp.toSec();//记录第一个图像帧的时间
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    // 2.通过时间间隔判断相机数据流是否稳定，有问题则restart
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);// 复位
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();// 更新上一帧图像时间戳
    
    // 3.发布频率控制，保证每秒钟处理的Image小于FREQ，频率控制在10HZ以内
    // 并不是每读入一帧图像，就要发布特征点
    // 判断间隔时间内的发布次数
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)//round：四舍五入到最邻近整数
    {
        PUB_THIS_FRAME = true;// 发布当前帧
        // 时间间隔内的发布频率十分接近设定频率时，更新时间间隔起始时刻，并将数据发布次数置0
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    // 4.将图像编码8UC1转换为mono8,单色8bit
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;// ROS图像消息
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;//字节顺序，大端
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        // cv_bridge的toCVCopy函数将ROS图像消息转化为OpenCV图像，
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;//记录“feature tracker processing”开始时间

    // 5. 重要！！！trackerData[i].readImage读取图像数据
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)//单目
            //readImage()函数读取图像数据进行处理
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());//对图像用光流法进行特征点跟踪
        else//双目
        {
            if (EQUALIZE)// 光太亮或太暗，自适应直方图均衡化处理
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif

/*
trackerData[i].cur_img
trackerData[i].track_cnt       特征点被追踪次数
trackerData[i].cur_un_pts      3D点
trackerData[i].cur_pts         像素2D点
trackerData[i].ids             特征点ID
trackerData[i].pts_velocity    像素速度
*/

    // 6.更新全局ID
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);// 更新feature的id，主要是ID为-1的新检测的角点
        if (!completed)
            break;
    }

    // 7、将特征点id，矫正后归一化平面的3D点(x,y,z=1)，像素2D点(u,v)，像素的速度(vx,vy)，
    //封装成sensor_msgs::PointCloudPtr类型的feature_points实例中,发布到pub_img;
    if (PUB_THIS_FRAME)
    {
        pub_count++;
        // 重要！！！ feature_points
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);//3D点
        sensor_msgs::ChannelFloat32 id_of_point;//特征点ID
        sensor_msgs::ChannelFloat32 u_of_point;//像素2D点u
        sensor_msgs::ChannelFloat32 v_of_point;//像素2D点v
        sensor_msgs::ChannelFloat32 velocity_x_of_point;//像素速度vx
        sensor_msgs::ChannelFloat32 velocity_y_of_point;//像素速度vy

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);// 哈希表id
        for (int i = 0; i < NUM_OF_CAM; i++)// 循环相机数量
        {
            auto &un_pts = trackerData[i].cur_un_pts;//3D点
            auto &cur_pts = trackerData[i].cur_pts;//像素2D点
            auto &ids = trackerData[i].ids;//特征点ID
            auto &pts_velocity = trackerData[i].pts_velocity;//像素速度
            for (unsigned int j = 0; j < ids.size(); j++)// 特征点数量
            {
                if (trackerData[i].track_cnt[j] > 1)// 该特征点被追踪次数大于1
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);// 哈希表id  insert

                    geometry_msgs::Point32 p;// 大规模点云信息
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
       
        // 第一帧不发布，因为没有光流速度
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);

        // 8、将图像封装到cv_bridge::cvtColor类型的ptr实例中发布到pub_match
        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);// show_img灰度图转RGB（tmp_img）
                //显示追踪状态，越红越好，越蓝越不行
                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);//WINDOW_SIZE=20，特征点被追踪次数越多，len的值越大
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);//len的值越大，圆的线条颜色越红，表示追踪次数越多，越好
                    //cv::circle(图像，圆心坐标，圆的半径，圆的颜色（cv::Scalar(v1, v2, v3, v4)的这四个参数就依次是BGRA，即蓝、绿、红和透明度），圆线条粗细)

                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());//计算时间间隔
}

int main(int argc, char **argv)
{
    //ros初始化和设置句柄
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    
    //设置logger的级别。 只有级别大于或等于level的日志记录消息才会得到处理。
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // 1.读取yaml中的一些配置参数
    readParameters(n);//parameters.cpp中的函数

    // 2.读取每个相机实例对应的相机内参，NUM_OF_CAM 经常为1，单目，VINS-Mono主要是在单目相机下运行的程序，后期港科大研发VINS-Fusion面向更多种类的相机
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    // 3.判断是否加入鱼眼mask来去除边缘噪声
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // 4.订阅话题IMAGE_TOPIC(/cam0/image_raw),有图像发布到这个话题时，执行回调函数img_callback
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    // 5.发布feature点云，实例feature_points，跟踪的特征点，给后端优化用
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    //发布feature_img，实例ptr，跟踪的特征点图，给RVIZ用和调试用
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    //发布restart
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?