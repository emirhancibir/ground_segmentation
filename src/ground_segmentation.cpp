#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <ctime>

ros::Publisher ground_pub;
ros::Publisher obstacle_pub;

double distance_threshold = 0.05; // RANSAC mesafe eşiği
double z_threshold = 0.3;         // Z yükseklik filtresi
double normal_angle_threshold = 0.9; // 25 derece cos > 0.9 -> açı < 25 derece

void calculatePlaneWithRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double& a, double& b, double& c, double& d, int max_iterations) {
    
    
    // uyumlu degerler icin
    int best_inlier_count = 0;
    double best_a = 0, best_b = 0, best_c = 0, best_d = 0;

    // rastgele sayi icin simdinin timestampiyle rand sayi
    srand(time(0));
    // ROS_INFO("Starting RANSAC with %d points and %d iterations.", (int)num_points, max_iterations);

    for (int i = 0; i < max_iterations; ++i) {
        // rastgele 3 nokta normalize point size ile
        int idx1 = rand() % cloud->size();
        int idx2 = rand() % cloud->size();
        int idx3 = rand() % cloud->size();

        if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3)
        {
            --i; 
            continue;
        }

        pcl::PointXYZ p = cloud->points[idx1]; /// üç point düzlem denklemi için p q r 
        pcl::PointXYZ q = cloud->points[idx2];
        pcl::PointXYZ r = cloud->points[idx3];

        // ROS_DEBUG("Selected points for RANSAC: P1(%f, %f, %f), P2(%f, %f, %f), P3(%f, %f, %f)", p.x, p.y, p.z, q.x, q.y, q.z, r.x, r.y, r.z);

        // duzlem denklem
        double temp_a, temp_b, temp_c, temp_d;
        double v1x = q.x - p.x;
        double v1y = q.y - p.y;
        double v1z = q.z - p.z; // PQ === a
        double v2x = r.x - p.x;
        double v2y = r.y - p.y;
        double v2z = r.z - p.z; // PR vector === b   // a x b = normal (vektorel)

        temp_a = (v1y * v2z) - (v1z * v2y); // axb = determinant i j k , v1 ,,,, v2,,,,
        temp_b = (v1z * v2x) - (v1x * v2z);
        temp_c = (v1x * v2y) - (v1y * v2x);
        temp_d = -(temp_a * p.x + temp_b * p.y + temp_c * p.z);
        ROS_INFO("duzlem denklemi %fx+%fy+%fz+%f=0",temp_a, temp_b, temp_c,temp_d);

        double norm = std::sqrt(temp_a * temp_a + temp_b * temp_b + temp_c * temp_c);
        if (norm == 0.0) continue; // normal vektör sifir

        double cos_angle = std::abs(temp_c) / norm; // Z bileseni
        if (cos_angle < normal_angle_threshold) {
            // Düzlem Z eksenine paralel değilse, geçersiz
            ROS_WARN("duzlem normali  Z eksenine yakin degil. Cos angle: %f", cos_angle);
            continue;
        }
        int inlier_count = 0;
        for (const auto& point : cloud->points) {
            double distance = std::abs(temp_a * point.x + temp_b * point.y + temp_c * point.z + temp_d) / norm;
            if (distance < distance_threshold && cos_angle > normal_angle_threshold) {
                ++inlier_count;
            }
        }

        // Eğer daha iyi bir model bulduysak, kaydet
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_a = temp_a;
            best_b = temp_b;
            best_c = temp_c;
            best_d = temp_d;
            ROS_INFO("Yeni en iyi düzlem: %d inlier ile (%fx + %fy + %fz + %f = 0)", best_inlier_count, best_a, best_b, best_c, best_d);
        }
    }

    if (best_inlier_count > 0) {
        // En iyi düzlemi döndür
        a = best_a;
        b = best_b;
        c = best_c;
        d = best_d;
    }
    else {
        ROS_WARN("RANSAC uygun bir düzlem bulamadı.");
    }
    
}

void applyHeightFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud, double z_threshold) {

    // int initial_count = input_cloud->size();
    for (const auto& point : input_cloud->points) {
        if (point.z < z_threshold) { // Yüksekliği z_threshold'tan küçük olan noktaları ekle
            filtered_cloud->points.push_back(point);
        }
    }
}

void segmentGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud, double a, double b, double c, double d) {
    double norm = std::sqrt(a * a + b * b + c * c);
    // ROS_INFO("Segmenting ground and obstacles based on plane equation: %fx + %fy + %fz + %f = 0", a, b, c, d);

    for (const auto& point : input_cloud->points) {
        double distance = std::abs(a * point.x + b * point.y + c * point.z + d) / norm;
        if (distance < distance_threshold) {
            ground_cloud->points.push_back(point); // Zemin noktası
        } else {
            obstacle_cloud->points.push_back(point); // Engel noktası
        }
    }

    // ROS_INFO("Segmentation completed. Ground points: %d, Obstacle points: %d", (int)ground_cloud->size(), (int)obstacle_cloud->size());
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
    // ROS_INFO("Received a point cloud message.");

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*input_cloud_msg, *input_cloud);


    if (input_cloud->size() < 3) {
        ROS_WARN("Not enough points for segmentation.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    applyHeightFilter(input_cloud, filtered_cloud, z_threshold);

    if (filtered_cloud->size() < 3) {
        ROS_WARN("Not enough points after height filtering.");
        return;
    }


    double a, b, c, d;
    calculatePlaneWithRANSAC(filtered_cloud, a, b, c, d, 100); // 100 iter

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    segmentGround(input_cloud, ground_cloud, obstacle_cloud, a, b, c, d);

    sensor_msgs::PointCloud2 ground_msg, obstacle_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    pcl::toROSMsg(*obstacle_cloud, obstacle_msg);


    ground_msg.header = input_cloud_msg->header;
    obstacle_msg.header = input_cloud_msg->header;
    ground_pub.publish(ground_msg);
    obstacle_pub.publish(obstacle_msg);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_segmentation_with_ransac");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("distance_threshold", distance_threshold, 0.05);
    pnh.param("z_threshold", z_threshold, 0.3);
    pnh.param("normal_angle_threshold", normal_angle_threshold, 0.9);

    std::string input_topic, ground_topic, nonground_topic;
    pnh.param<std::string>("input_topic", input_topic, "/cloud_all_fields_fullframe");
    pnh.param<std::string>("ground_topic", ground_topic, "/ground_points");
    pnh.param<std::string>("nonground_topic", nonground_topic, "/nonground_points");

    ros::Subscriber sub = nh.subscribe(input_topic, 1, pointCloudCallback);
    ground_pub = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 1);
    obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>(nonground_topic, 1);

    ROS_INFO("Ground segmentation with RANSAC node is running.");
    ros::spin();
    return 0;
}

