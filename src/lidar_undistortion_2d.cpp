#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.9/pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>

/**
 * @brief sensor_msgs::laserscan
 * //头文件，关键是time_stamped与frame id
 * header:
 * seq: 448
 * stamp:
 * secs: 93
 * nsecs: 955000000
 * frame_id: "base_scan"
 * //最大角度与最小角度，这里是0~2pi,角度为rad
 * angle_min: 0.0
 * angle_max: 6.28318977356
 * //角度增量，实际上就是角度步长，单位为rad；
 * //(angle_max-angle_min)/angle_increment即为激光束选择一圈所获得的点云数据数量
 * angle_increment: 0.0175019223243
 * //关于激光点云运动时间的问题，目前未应用，直接借鉴官网解释
 * time_increment: 0.0
 * scan_time: 0.0
 * //测距最大范围与最小范围
 * range_min: 0.119999997318
 * range_max: 3.5
 * //点云所携带的环境信息，包括测距信息与强度信息，该数组的长度即为(angle_max-angle_min)/angle_increment
 * ranges: [0.6837663650512695, 0.6689223051071167, 0.6657941937446594,.....]
 * intensities: [5.823023349397141e-37, 5.823023349397141e-37, 5.823023349397141e-37......]
 **/

//如果使用调试模式，可视化点云，需要安装PCL
#define debug_ 1
#define debug_distortion_ 1
#define debug_undistortion_ 1
#define debug_final_ 1

#if debug_
pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");//初始化一个pcl窗口
#endif

class LidarMotionCalibrator
{
private:
    
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher scan_pub_;
    ros::Publisher pointcloud_pub_;

    tf::TransformListener* tf_;

    std::string scan_sub_topic_;
    std::string scan_pub_topic_;
    std::string pointcloud_pub_topic_;
    std::string lidar_frame_;
    std::string odom_frame_;

    bool enable_pub_pointcloud_;
    double lidar_scan_time_gain_;

#if debug_
    //可视化点云对象
    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
#endif

public:


    LidarMotionCalibrator(ros::NodeHandle node_handle)
    {
        nh_ = node_handle;
        tf_ = new tf::TransformListener(ros::Duration(10.0));

        ros::NodeHandle nh_param("~");
        nh_param.param<std::string>("scan_sub_topic", scan_sub_topic_, "/scan");
        nh_param.param<std::string>("scan_pub_topic", scan_pub_topic_, "/scan_undistortion");
        nh_param.param<bool>("enable_pub_pointcloud", enable_pub_pointcloud_, true);
        nh_param.param<std::string>("point_cloud_pub_topic", pointcloud_pub_topic_, "/pointcloud_undistortion");
        nh_param.param<std::string>("lidar_frame", lidar_frame_, "laser_link");
        nh_param.param<std::string>("odom_frame", odom_frame_, "oodm");
        nh_param.param<double>("lidar_scan_time_gain", lidar_scan_time_gain_, 1.0);

        scan_sub_ = nh_.subscribe(scan_sub_topic_, 10, &LidarMotionCalibrator::ScanCallBack, this);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_pub_topic_, 1);
        if(enable_pub_pointcloud_)
            pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_pub_topic_, 1);
    }


    ~LidarMotionCalibrator()
    {
        if(tf_!=NULL) delete tf_;
    }

#if debug_
    void visual_cloud_scan(const std::vector<float> &ranges_,const std::vector<double> &angles_,unsigned char r_,unsigned char g_,unsigned char b_)
    {
        unsigned char r = r_, g = g_, b = b_; //变量不要重名
        for(int i = 0; i < ranges_.size();i++)
        {
            if(ranges_[i] < 0.05 || std::isnan(ranges_[i]) || std::isinf(ranges_[i]))
                continue;

            pcl::PointXYZRGB pt;
            pt.x = ranges_[i] * cos(angles_[i]);
            pt.y = ranges_[i] * sin(angles_[i]);
            pt.z = 1.0;

            // pack r/g/b into rgb
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }
    }
#endif

    // 拿到原始的激光数据来进行处理
    void ScanCallBack(const sensor_msgs::LaserScanConstPtr& scan_msg)
    {
        // 转换到矫正需要的数据
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;

        // 得到最终点的时间
        int beamNum = scan_msg->ranges.size();
        endTime = startTime + ros::Duration(scan_msg->time_increment * lidar_scan_time_gain_ * beamNum);

        // 将数据复制出来
        std::vector<double> angles;
        std::vector<float> ranges;
        std::vector<float> intensities;

        // 注意这里是倒叙取出数据
        for(int alpha = beamNum - 1; alpha >= 0; alpha--)
        {
            double lidar_dist = scan_msg->ranges[alpha];

            if(std::isnan(lidar_dist) || lidar_dist < scan_msg->range_min)
                lidar_dist = 0.0;

            intensities.push_back(scan_msg->intensities[alpha]);
            ranges.push_back(lidar_dist);
            angles.push_back(scan_msg->angle_min + scan_msg->angle_increment * alpha);
        }

#if debug_
        visual_cloud_.clear();
#endif

#if debug_distortion_
        //转换为pcl::pointcloud for visuailization
        //数据矫正前、封装打算点云可视化、红色
        visual_cloud_scan(ranges,angles,255,0,0);
#endif

        tf::Stamped<tf::Pose> start_pose, end_pose;
        if(!getLaserPose(start_pose, ros::Time(startTime), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }
        if(!getLaserPose(end_pose,ros::Time(endTime), tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        // ROS_INFO("calibration start");
        // 进行矫正
        Lidar_Calibration(ranges,
                          angles,
                          startTime,
                          endTime,
                          tf_);

#if debug_undistortion_
        // 数据矫正之后
        visual_cloud_scan(ranges,angles,0,255,0);
#endif

        // ROS_INFO("calibration end");

        if(enable_pub_pointcloud_)
            publishPointCloud2(startTime, angles, ranges, intensities);

        publishScan(scan_msg, ranges, angles, intensities, start_pose, end_pose);

#if debug_
        g_PointCloudView.showCloud(visual_cloud_.makeShared());
#endif
    }

    
    /**
     * @brief 将ensor_msgs::LaserScan转换成sensor_msgs::PointCloud2并发布
     * */
    void publishPointCloud2(ros::Time start_time,
                            std::vector<double>& angles,
                            std::vector<float>& ranges,
                            std::vector<float>& intensities)
    {
        sensor_msgs::PointCloud2 pointcloud_msg;
        pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
        pcl::PointXYZI point_xyzi;

        for(int index = 0; index < angles.size(); ++index) {
            point_xyzi.x = ranges[index] * cos(angles[index]);
            point_xyzi.y = ranges[index] * sin(angles[index]);
            point_xyzi.z = 0.0;
            point_xyzi.intensity = intensities[index];
            pointcloud_pcl.push_back(point_xyzi);
        }

        pcl::toROSMsg(pointcloud_pcl, pointcloud_msg);
        pointcloud_msg.header.frame_id = lidar_frame_;
        pointcloud_msg.header.stamp = start_time;
        pointcloud_pub_.publish(pointcloud_msg);
    }


    /**
     * @name getLaserPose()
     * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
     *        得到dt时刻激光雷达在odom坐标系的位姿
     * @param odom_pos  机器人的位姿
     * @param dt        dt时刻
     * @param tf_
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = lidar_frame_;
        robot_pose.stamp_ = dt;   //设置为ros::Time()表示返回最近的转换关系

        // get the global pose of the robot
        try
        {
            if(!tf_->waitForTransform(odom_frame_, lidar_frame_, dt, ros::Duration(0.5)))             // 0.15s 的时间可以修改
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose(odom_frame_, robot_pose, odom_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }


    /**
     * @brief Lidar_MotionCalibration
     *        激光雷达运动畸变去除分段函数;
     *        在此分段函数中，认为机器人是匀速运动；
     * @param frame_base_pose       标定完毕之后的基准坐标系
     * @param frame_start_pose      本分段第一个激光点对应的位姿
     * @param frame_end_pose        本分段最后一个激光点对应的位姿
     * @param ranges                激光数据－－距离
     * @param angles                激光数据－－角度
     * @param startIndex            本分段第一个激光点在激光帧中的下标
     * @param beam_number           本分段的激光点数量
     */
    void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<float>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number)
    {
        // 每个位姿进行线性插值时的步长
        const double beam_step = 1.0 / (beam_number - 1);

        // 得到偏航角
        tf::Quaternion base_quaternion = frame_base_pose.getRotation();
        double base_angle = tf::getYaw(base_quaternion);
        tf::Quaternion start_quaternion = frame_start_pose.getRotation();
        double start_angle = tf::getYaw(start_quaternion);
        tf::Quaternion end_quaternion = frame_end_pose.getRotation();
        double end_angle = tf::getYaw(end_quaternion);

        // 得到位移
        tf::Vector3 base_point = frame_base_pose.getOrigin();
        base_point.setZ(0);
        tf::Vector3 start_point = frame_start_pose.getOrigin();
        start_point.setZ(0);
        tf::Vector3 end_point = frame_end_pose.getOrigin();
        end_point.setZ(0);

        for(int index = 0; index < beam_number; ++index) {

            const double lidar_range = ranges[startIndex + index];
            const double lidar_angle = angles[startIndex + index];

            // 进行插值
            tf::Pose frame_cur_pose;
            tf::Quaternion cur_quaternion = start_quaternion.slerp(end_quaternion, index * beam_step);
            tf::Vector3 cur_point = start_point.lerp(end_point, index * beam_step);
            frame_cur_pose.setOrigin(cur_point);
            frame_cur_pose.setRotation(cur_quaternion);

            // 如果激光雷达检测到的距离不为0
            if(!tfFuzzyZero(lidar_range) && !std::isinf(lidar_range)) {

                tf::Vector3 lidar_point;
                double lidar_x = lidar_range * cos(lidar_angle);
                double lidar_y = lidar_range * sin(lidar_angle);
                lidar_point.setValue(lidar_x, lidar_y, 0);
                
                tf::Vector3 corrected_lidar_point = tf::Pose(base_quaternion, base_point).inverse() * frame_cur_pose * lidar_point;

                ranges[startIndex + index] = hypot(corrected_lidar_point.getX(), corrected_lidar_point.getY());
                angles[startIndex + index] = atan2(corrected_lidar_point.getY(), corrected_lidar_point.getX());
            }
            else { // 如果为0

                // 里程计坐标系的角度
                double tmp_angle = tf::getYaw(cur_quaternion) + lidar_angle;
                // 将角度转换到-pai到pai之间
                tmp_angle = tfNormalizeAngle(tmp_angle);

                // 如果数据非法 则只需要设置角度就可以了。把角度换算成start_pos坐标系内的角度
                angles[startIndex + index] = tfNormalizeAngle(tmp_angle - base_angle);
            }

            // 相关日志的输出
            // ROS_INFO("\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t", 
            // startIndex + index, base_point.getX(), base_point.getY(), tf::getYaw(base_quaternion), 
            //                     cur_point.getX(), cur_point.getY(), tf::getYaw(cur_quaternion), 
            // lidar_range, lidar_angle, ranges[startIndex + index], angles[startIndex + index]);
        }
    }



    //激光雷达数据　分段线性进行插值　分段的周期为10ms
    //这里会调用Lidar_MotionCalibration()
    /**
     * @name Lidar_Calibration()
     * @brief 激光雷达数据　分段线性进行差值　分段的周期为5ms
     * @param ranges 激光束的距离值集合
     * @param angle　激光束的角度值集合
     * @param startTime　第一束激光的时间戳
     * @param endTime　最后一束激光的时间戳
     * @param tf_
    */
    void Lidar_Calibration(std::vector<float>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_)
    {
        //统计激光束的数量
        int beamNumber = ranges.size();
        if(beamNumber != angles.size())
        {
            ROS_ERROR("Error:ranges not match to the angles");
            return ;
        }

        // 5ms来进行分段
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        //起始时间 us
        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / beamNumber; // 每束激光数据的时间间隔

        //当前插值的段的起始坐标
        int start_index = 0;

        //起始点的位姿 这里要得到起始点位置的原因是　起始点就是我们的base_pose
        //所有的激光点的基准位姿都会改成我们的base_pose
        // ROS_INFO("get start pose");

        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0), tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        int cnt = 0;
        //基准坐标就是第一个位姿的坐标
        frame_base_pose = frame_start_pose;
        for(int i = 0; i < beamNumber; i++)
        {
            //分段线性,时间段的大小为interpolation_time_duration
            double mid_time = start_time + time_inc * (i - start_index);
            if(i == beamNumber - 1) {
                nh_.param<std::string>("odom_frame", odom_frame_, "oodm");
            }
            if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;

                //得到起点和终点的位姿
                //终点的位姿
                if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error",cnt);
                    return ;
                }

                //对当前的起点和终点进行插值
                //interpolation_time_duration中间有多少个点.
                int interp_count = i - start_index + 1;

                Lidar_MotionCalibration(frame_base_pose, // 开始时刻位姿
                                        frame_start_pose, // 当前线性插值开始时的位姿
                                        frame_mid_pose, // 当前线性插值结束时的位姿
                                        ranges, // 激光雷达距离数据
                                        angles, // 激光雷达角度数据
                                        start_index, // 当前线性插值开始时的index
                                        interp_count); // 需要插值多少个点

                //更新时间
                start_time = mid_time;
                start_index = i + 1; // 深蓝的程序在这里有BUG
                frame_start_pose = frame_mid_pose;
            }
        }
    }

    void publishScan(const sensor_msgs::LaserScanConstPtr& scan_msg,
                     std::vector<float>& ranges,
                     std::vector<double>& angles,
                     std::vector<float>& intensities,
                     tf::Stamped<tf::Pose>& start_pose,
                     tf::Stamped<tf::Pose>& end_pose)
    {
        
        sensor_msgs::LaserScan publish_msg;
        publish_msg.header = scan_msg->header;
        publish_msg.time_increment = scan_msg->time_increment;
        publish_msg.scan_time = scan_msg->scan_time;
        publish_msg.range_min = scan_msg->range_min;
        publish_msg.range_max = scan_msg->range_max;

        publish_msg.angle_max = scan_msg->angle_max;
        publish_msg.angle_min = scan_msg->angle_min;
        publish_msg.angle_increment = scan_msg->angle_increment;

        for(int alpha = 0; alpha < ranges.size(); ++alpha) {
            publish_msg.ranges.push_back(0);
            publish_msg.intensities.push_back(0);
        }
        // 测距和激光强度信息的发布
        // sensor_msgs::LaserScan的角度是按照分辨率给定的，相邻相差一致，而去畸变的角度差是不固定,所以需要做适当的修改
        for(int alpha = ranges.size() - 1; alpha >= 0; --alpha) {
            double angle = (angles[alpha] < 0 || tfFuzzyZero(angles[alpha])) ? angles[alpha] + 2 * M_PI : angles[alpha];
            //double angle = (angles[alpha] < 0 || tfFuzzyZero(angles[alpha])) ? angles[alpha] + M_PI : angles[alpha] + M_PI;
            angle += publish_msg.angle_min;
            int index = (int)((angle - publish_msg.angle_min) / publish_msg.angle_increment);
            if(index >= 0 && index < ranges.size()) {
                publish_msg.ranges[index] = ranges[alpha];
                publish_msg.intensities[index] = intensities[alpha];
            }
        }

        // 因为激光雷达的扫描顺序是倒的
        // publish_msg.angle_max = scan_msg->angle_max;
        // publish_msg.angle_min = scan_msg->angle_min - tf::getYaw(start_pose.getRotation()) + tf::getYaw(end_pose.getRotation());
        // publish_msg.angle_increment = (publish_msg.angle_max - publish_msg.angle_min) / (float)(angles.size() - 1);
        // for(int index = ranges.size() - 1; index >= 0; --index) {
        //     publish_msg.ranges.push_back(ranges[index]);
        // }

        // if(fabs(- tf::getYaw(start_pose.getRotation()) + tf::getYaw(end_pose.getRotation()) > 0.01)) {
        //     ROS_INFO("after- angle_min: %f, angle_max: %f, angle_increment: %f", publish_msg.angle_min, publish_msg.angle_max, publish_msg.angle_increment);
        //     ROS_INFO("orign- angle_min: %f, angle_max: %f, angle_increment: %f", scan_msg->angle_min, scan_msg->angle_max, scan_msg->angle_increment);

        //     float range_alpha, angle_alpha, range_beta, angle_beta;
        //     for(int alpha = ranges.size() - 1; alpha >= 0; --alpha) {
        //         range_alpha = ranges[alpha];
        //         angle_alpha = angles[alpha];

        //         int beta = ranges.size() - 1 - alpha;
        //         range_beta = publish_msg.ranges[beta];
        //         angle_beta = publish_msg.angle_min + beta * publish_msg.angle_increment;

        //         ROS_INFO("\t%d\t%f\t%f\t%f\t%f\t", alpha, range_alpha, angle_alpha, range_beta, angle_beta);
        //     }
        // }
#if debug_final_
        //封装数据的可视化的测试
        std::vector<double> angles_temp;
        std::vector<float> ranges_temp;
        for(int i = 0; i < publish_msg.ranges.size();i++)
        {
            double lidar_dist = publish_msg.ranges[i];
            double lidar_angle = publish_msg.angle_min + publish_msg.angle_increment * i;

            ranges_temp.push_back(lidar_dist);
            angles_temp.push_back(lidar_angle);
        }
        visual_cloud_scan(ranges_temp,angles_temp,255,255,255);
#endif

        scan_pub_.publish(publish_msg);
    }

    //使用点云将激光可视化

};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "LidarMotionCalib");

    LidarMotionCalibrator tmpLidarMotionCalib(ros::NodeHandle("~"));

    ros::spin();
    return 0;
}

