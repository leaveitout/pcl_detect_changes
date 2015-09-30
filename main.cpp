#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
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

    if(loadPCDFileSafe(stringIter.next().toStdString(), cloudA) != 0)
        return -1;

    // Change resolution object, with a resolution of 128
    // (resolution at lowest octree level.
    // TODO: Add arg parser for this value
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(256.0f);

    // Add cloudA to the octree
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();

    // The change detector object is able to store two clouds at the same time
    // with this we can reset the buffer but keep the previous saved tree
    octree.switchBuffers();

    // Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
            (new pcl::visualization::PCLVisualizer ("Voxel Diff"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while(stringIter.hasNext() || !viewer->wasStopped()) {
        string filePath = stringIter.next().toStdString();
        cout << filePath << endl;
        if (loadPCDFileSafe(filePath, cloudB) != 0)
            return -1;

        // Get a vector with the indices of all the points that are new in cloud B,
        // when compared with the ones that existed in cloud A.
        vector<int> newPoints;

        octree.setInputCloud(cloudB);
        octree.addPointsFromInputCloud();
        octree.getPointIndicesFromNewVoxels(newPoints);

        for (size_t i = 0; i < newPoints.size(); ++i) {
            cout << "Point (" << cloudB->points[newPoints[i]].x << ", "
            << cloudB->points[newPoints[i]].y << ", "
            << cloudB->points[newPoints[i]].z
            << ") was not in cloud A but is in cloud B" << std::endl;
        }
        cout << newPoints.size() << std::endl;

        // Swap buffers and pointer
        octree.switchBuffers();
        cloudA = cloudB;
    }

    return 0;
}