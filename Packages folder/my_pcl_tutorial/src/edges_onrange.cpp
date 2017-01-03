#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the borders.
	pcl::PointCloud<pcl::BorderDescription>::Ptr borders(new pcl::PointCloud<pcl::BorderDescription>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Convert the cloud to range image.
	int imageSizeX = 640, imageSizeY = 480;
	float centerX = (640.0f / 2.0f), centerY = (480.0f / 2.0f);
	float focalLengthX = 525.0f, focalLengthY = focalLengthX;
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
								 cloud->sensor_origin_[1],
								 cloud->sensor_origin_[2])) *
								 Eigen::Affine3f(cloud->sensor_orientation_);
	float noiseLevel = 0.0f, minimumRange = 0.0f;
	pcl::RangeImagePlanar rangeImage;
	rangeImage.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
			centerX, centerY, focalLengthX, focalLengthX,
			sensorPose, pcl::RangeImage::CAMERA_FRAME,
			noiseLevel, minimumRange);

	// Border extractor object.
	pcl::RangeImageBorderExtractor borderExtractor(&rangeImage);

	borderExtractor.compute(*borders);

	// Visualize the borders.
	pcl::visualization::RangeImageVisualizer* viewer = NULL;
	viewer = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(rangeImage,
			 -std::numeric_limits<float>::infinity(),
			 std::numeric_limits<float>::infinity(),
			 false, *borders, "Borders");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		// Sleep 100ms to go easy on the CPU.
		pcl_sleep(0.1);
	}
}
