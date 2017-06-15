//OPENCV
#include <opencv2/opencv.hpp>

using namespace cv;

namespace skimap
{
namespace utils
{

/**
 */
struct PointRGB
{
    cv::Point3f point;
    cv::Vec3b color;
    float w;
};

/**
 */
struct Camera
{
    double fx, fy, cx, cy;
    int width, height;
    double min_distance;
    double max_distance;
    int point_cloud_downscale;
    cv::Mat camera_matrix;

    void build3DTensor(cv::Mat &rgb, cv::Mat &depth, cv::Mat &tensor, int downsample_factor = 1)
    {
        downsample_factor = downsample_factor > 1 ? downsample_factor : 1;
        tensor = cv::Mat(rgb.rows, rgb.cols, CV_32FC3);

        for (int y = 0; y < depth.rows; y += downsample_factor)
        {
            for (int x = 0; x < depth.cols; x += downsample_factor)
            {
                float d = depth.at<float>(y, x);
                cv::Vec3f p3d;
                p3d[0] = (d / this->fx) * (x - this->cx);
                p3d[1] = (d / this->fy) * (y - this->cy);
                p3d[2] = d;
                //printf("(%f,%f,%f,%f)  %d,%d,%f = %f,%f,%f\n", this->fx, this->fy, this->cx, this->cy, y, x, d, p3d[0], p3d[1], p3d[2]);
                if (p3d[2] != p3d[2])
                    p3d[2] = 0.0f;
                if (p3d[2] < this->min_distance && p3d[2] > this->max_distance)
                    p3d[2] = 0.0f;

                tensor.at<cv::Vec3f>(y, x) = p3d;
            }
        }
    }

    /**
     * Load Camera Object from YAML file
     */
    static Camera loadFromFile(std::string yaml_file)
    {
        Camera camera;
        FileStorage fs(yaml_file, FileStorage::READ);
        camera.width = fs["image_width"];
        camera.height = fs["image_height"];
        fs["camera_matrix"] >> camera.camera_matrix;
        std::cout << camera.camera_matrix.type() << std::endl;
        camera.fx = camera.camera_matrix.at<double>(0, 0);
        camera.fy = camera.camera_matrix.at<double>(1, 1);
        camera.cx = camera.camera_matrix.at<double>(0, 2);
        camera.cy = camera.camera_matrix.at<double>(1, 2);
        camera.min_distance = fs["min_distance"];
        camera.max_distance = fs["max_distance"];
        return camera;
    }

} camera;
}
}
