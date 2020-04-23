#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

template <typename T>//定义模板的固定格式
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)//读取yaml中的参数
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");//launch文件中会指明标签"config_file"的具体路径
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);//实例化一个FileStorage类对象
        /*
        #pragma region
        构造函数：cv::FileStorage(const string& source, int flags， const string& encoding=string());  

        source –存储或读取数据的文件名（字符串），其扩展名(.xml 或 .yml或者.yaml)决定文件格式。

        flags – 操作方式，包括：
            FileStorage::READ 打开文件进行读操作
            FileStorage::WRITE 打开文件进行写操作
            FileStorage::APPEND打开文件进行附加操作，在已有内容的文件里添加

        encoding—编码方式，用默认值就好。 

        FileStorage类的使用流程如下：
        （1）实例化一个FileStorage类对象
        （2）使用流操作符<<进行文件写入，>>进行文件读取，类似C++中的文件操作
        （3）使用FileStorage::release()函数析构掉类对象，并关闭文件

        写操作：
            int main(int argc, char** argv)
            {
                FileStorage file_("1.yml", FileStorage::WRITE);
                int num1 = 1, num2 = 2;
                file_<<"num1"<<num1;
                file_<<"num2"<<num2;
                Mat A(4,5,CV_32FC1,Scalar(10));
                file_<<"A"<<A;
                file_.release();
            }
            特别注意！
                file_<<"num1"<<num1;
            输入的是标签，这里如果写成　file_<<"num1："<<num1; 就会报错，不要画蛇添足加个冒号
        读操作：
            int main(int argc, char** argv)
            {
                FileStorage fs_ ("1.yml", FileStorage::READ);
                int r1, r2;
                fs_["num1"]>>r1;
                fs_["num2"]>>r2;   
                Mat A(4,5,CV_32FC1);
                fs_["A"]>>A;
                cout<< "r1 :"<<r1<<endl;
                cout<< "r2 :"<<r2<<endl;
                cout<< "A :"<<"\n"<<A<<endl;
                fs_.release();
            }
        #pragma endregion
        */
    if(!fsSettings.isOpened())//检查文件是否已经打开
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        /*
        #pragma region
        cout：写到标准输出的ostream对象,cout经过缓冲后输出，默认情况下是显示器。这是一个被缓冲的输出，是标准输出，并且可以重新定向
        cerr：输出到标准错误的ostream对象，常用于程序错误信息,cerr不经过缓冲而直接输出，一般用于迅速输出出错信息，是标准错误，
            默认情况下被关联到标准输出流，但它不被缓冲，也就说错误消息可以直接发送到显示器，而无需等到缓冲区或者新的换行符时，才被显示。一般情况下不被重定向

        对于为什么有cerr和clog:
        　　比如，你的程序遇到调用栈用完了的威胁（无限，没有出口的递归）。
        　　你说，你到什么地方借内存，存放你的错误信息？
        　　所以有了cerr。其目的，就是在你最需要它的紧急情况下，还能得到输出功能的支持。
        　　缓冲区的目的，就是减少刷屏的次数——比如，你的程序输出圣经中的一篇文章。不带缓冲的话，就会每写一个字母，就输出一个字母，然后刷屏。有了缓冲，
           你将看到若干句子“同时”就出现在了屏幕上（由内存翻新到显存，然后刷新屏幕）。
        #pragma endregion
        */
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");//launch文件中会指明标签"vins_folder"的具体路径

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"]; 
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
