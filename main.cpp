#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QtDebug>
#include <QtCore/qdiriterator.h>


using namespace std;
using namespace boost::filesystem;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;


QStringList getAllPCDFilesInDir(QDir dir) {
    QDirIterator dirIterator(dir);

    QStringList pathList;

    while(dirIterator.hasNext()) {

        dirIterator.next();

        if(QFileInfo(dirIterator.filePath()).isFile())
        if(QFileInfo(dirIterator.filePath()).suffix() == "pcd") {
            pathList << dirIterator.filePath();
        }
    }

    pathList.sort();

    return pathList;
}

int loadPCDFileSafe(string file, CloudT::Ptr cloud) {
    if(pcl::io::loadPCDFile(file, *cloud) != 0) {
        cout << "Failed to load .pcd file at " << file << endl;
        return -1;
    }
    else
        return 0;
}

pcl::ModelCoefficientsPtr planarSac(CloudT::Ptr cloud) {
    pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients(true);

    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return coefficients;
    }

    std::cout << "Model coefficients: " << coefficients->values[0] << " "
    << coefficients->values[1] << " "
    << coefficients->values[2] << " "
    << coefficients->values[3] << std::endl;

    return coefficients;
}


pcl::IndicesPtr filterByPlane(CloudT::Ptr cloud, float distanceThold = 0.01f) {
    pcl::ModelCoefficientsPtr coefficients = planarSac(cloud);
    pcl::IndicesPtr indices (new std::vector<int>);
    float a(coefficients->values[0]), b (coefficients->values[1]),
            c(coefficients->values[2]), d(coefficients->values[3]);
    for(size_t i = 0; i<cloud->size(); ++i) {
        PointT point = cloud->at(i);
        float proj = a * point.x + b * point.y + c * point.z + d;
        proj /= sqrt(a*a + b*b + c*c);
        if(proj < -distanceThold)
            indices->push_back(static_cast<int>(i));
    }

    return indices;
}

CloudT::Ptr removePlaneFilteredPoints(CloudT::Ptr cloud) {
    pcl::IndicesPtr indices = filterByPlane(cloud);

    pcl::ExtractIndices<PointT> extract;

    CloudT::Ptr result(new CloudT);

    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.setKeepOrganized(true);
    extract.filter(*result);

    return result;
}

int main(int argc, char** argv) {

    // Objects for storing the point clouds
    CloudT::Ptr cloudA(new CloudT);
    CloudT::Ptr cloudB(new CloudT);

    QStringList files = getAllPCDFilesInDir(QDir(argv[1]));

    if(files.empty()) {
        cout << "No .pcd files found in directory: " << argv[1] << endl;
        return -1;
    }
    else
        cout << files.size() << " .pcd files found in directory " << argv[1] << endl;

    // Read the first PCD file from disk
    QStringListIterator stringIter(files);

    if(stringIter.hasNext())
        if(loadPCDFileSafe(stringIter.next().toStdString(), cloudA) != 0)
            return -1;

    // Change resolution object, with a resolution of 128
    // (resolution at lowest octree level.
    // TODO: Add arg parser for this value
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(128.0f);
    cloudA = removePlaneFilteredPoints(cloudA);

    // Add cloudA to the octree
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();

    // The change detector object is able to store two clouds at the same time
    // with this we can reset the buffer but keep the previous saved tree
    octree.switchBuffers();

    // Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
            (new pcl::visualization::PCLVisualizer ("Voxel Diff"));

    viewer->addPointCloud<PointT>(cloudA, "CloudDiff");
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.05);
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    camera.view[1] *= -1;
    viewer->setCameraParameters(camera);

    while(stringIter.hasNext() && !viewer->wasStopped()) {

//        for(int i=0; i< 5; i++)
//            if( stringIter.hasNext())
//                stringIter.next();

        string filePath(stringIter.next().toStdString());
        cout << filePath << endl;
        if (loadPCDFileSafe(filePath, cloudB) != 0)
            return -1;

//        cloudB = removePlaneFilteredPoints(cloudB);

        // Get a vector with the indices of all the points that are new in cloud B,
        // when compared with the ones that existed in cloud A.
        pcl::IndicesPtr newPoints(new std::vector<int>);

        cout << "Cloud B size: " << cloudB->size() << endl;

        octree.setInputCloud(cloudB);
        octree.addPointsFromInputCloud();
        octree.getPointIndicesFromNewVoxels(*newPoints);

        cout << "New Points: " << newPoints->size() << std::endl;

        CloudT::Ptr cloudExtracted(new CloudT);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloudB);
        extract.setIndices(newPoints);
        extract.filter(*cloudExtracted);

        viewer->removePointCloud("CloudDiff");
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudB);
        viewer->addPointCloud<PointT>(cloudExtracted, rgb, "CloudDiff");

        viewer->spinOnce(1);
        boost::this_thread::sleep (boost::posix_time::microseconds(100));
//
        // Swap buffers and pointer
        octree.switchBuffers();
        cloudA = cloudB;
    }

    return 0;
}