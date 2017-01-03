#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Parameters needed by the planar range image object:

	// Image size. Both Kinect and Xtion work at 640x480.
	int imageSizeX = 640;
	int imageSizeY = 480;
	// Center of projection. here, we choose the middle of the image.
	float centerX = 640.0f / 2.0f;
	float centerY = 480.0f / 2.0f;
	// Focal length. The value seen here has been taken from the original depth images.
	// It is safe to use the same value vertically and horizontally.
	float focalLengthX = 525.0f, focalLengthY = focalLengthX;
	// Sensor pose. Thankfully, the cloud includes the data.
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
								 cloud->sensor_origin_[1],
								 cloud->sensor_origin_[2])) *
								 Eigen::Affine3f(cloud->sensor_orientation_);
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius (e.g., 0.03 == 3cm).
	float noiseLevel = 0.0f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored.
	float minimumRange = 0.0f;

	// Planar range image object.
	pcl::RangeImagePlanar rangeImagePlanar;
	rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
			centerX, centerY, focalLengthX, focalLengthX,
			sensorPose, pcl::RangeImage::CAMERA_FRAME,
			noiseLevel, minimumRange);

	// Visualize the image.
	pcl::visualization::RangeImageVisualizer viewer("Planar range image");
	viewer.showRangeImage(rangeImagePlanar);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		// Sleep 100ms to go easy on the CPU.
		pcl_sleep(0.1);
	}
}
