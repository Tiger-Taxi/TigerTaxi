#include <iostream>
#include <assert.h>
#include <signal.h>
#include <string>


#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/OccupancyGrid.h"
// #include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
 
//octomap 
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
int output_count;
ros::Publisher m_mapPub;

octomap::KeyRay m_keyRay;  // temp storage for ray casting
octomap::OcTreeKey m_updateBBXMin;
octomap::OcTreeKey m_updateBBXMax;

double m_maxRange = -1;
std::string m_worldFrameId = "odom";
std::string m_baseFrameId = "base_link";
bool m_useHeightMap = true;
double m_colorFactor;

bool m_latchedTopics = true;
bool m_publishFreeSpace = false;

double m_res = 0.05;
unsigned m_treeDepth = 0;
unsigned m_maxTreeDepth = 0;

double m_pointcloudMinX = -std::numeric_limits<double>::max();
double m_pointcloudMaxX = std::numeric_limits<double>::max();
double m_pointcloudMinY = -std::numeric_limits<double>::max();
double m_pointcloudMaxY = std::numeric_limits<double>::max();
double m_pointcloudMinZ = -std::numeric_limits<double>::max();
double m_pointcloudMaxZ = std::numeric_limits<double>::max();
double m_occupancyMinZ = -std::numeric_limits<double>::max();
double m_occupancyMaxZ = std::numeric_limits<double>::max();
double m_minSizeX = 0.0;
double m_minSizeY = 0.0;
bool m_filterSpeckles = false;

bool m_filterGroundPlane = false;
double m_groundFilterDistance = 0.04;
double m_groundFilterAngle = 0.15;
double m_groundFilterPlaneDistance = 0.07;

bool m_compressMap = true;

bool m_initConfig = true;

// downprojected 2D map:
bool m_incrementalUpdate;
nav_msgs::OccupancyGrid m_gridmap;
bool m_publish2DMap = true;
bool m_mapOriginChanged;
octomap::OcTreeKey m_paddedMinKey;
unsigned m_multires2DScale;
bool m_projectCompleteMap =  true;

void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) {
  if (map.info.resolution != oldMapInfo.resolution){
    ROS_ERROR("Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x)/map.info.resolution +0.5);
  int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y)/map.info.resolution +0.5);

  if (i_off < 0 || j_off < 0
      || oldMapInfo.width  + i_off > map.info.width
      || oldMapInfo.height + j_off > map.info.height)
  {
    ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

  for (int j =0; j < int(oldMapInfo.height); ++j ){
    // copy chunks, row by row:
    fromStart = oldMapData.begin() + j*oldMapInfo.width;
    fromEnd = fromStart + oldMapInfo.width;
    toStart = map.data.begin() + ((j+j_off)*m_gridmap.info.width + i_off);
    copy(fromStart, fromEnd, toStart);

//    for (int i =0; i < int(oldMapInfo.width); ++i){
//      map.data[m_gridmap.info.width*(j+j_off) +i+i_off] = oldMapData[oldMapInfo.width*j +i];
//    }

  }

}

// save latest 3d point cloud to local
void octo_callback(const sensor_msgs::PointCloud2ConstPtr& _cloud){
    pcl::fromROSMsg( *_cloud, *output_cloud);
    //cout<<"point cloud loaded, point size = "<< output_cloud->points.size()<<endl;
        // octo tree coefficient
    octomap::OcTree tree( 0.05 ); //resolution
    ROS_INFO("Hey, it got called");

    for (int i = 0; i < output_cloud->points.size() ; ++i)
    {
        tree.updateNode( octomap::point3d(  output_cloud->points[i].x, 
                                            output_cloud->points[i].y, 
                                            output_cloud->points[i].z), true );
    }
    // update octomap
    tree.updateInnerOccupancy();


    // TODO: Publish to /map or some shit
     m_gridmap.header.frame_id = m_worldFrameId;
     m_gridmap.header.stamp = ros::Time::now();
     nav_msgs::MapMetaData oldMapInfo = m_gridmap.info;
     octomap::OcTree* m_octree = &tree;

     // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
     double minX, minY, minZ, maxX, maxY, maxZ;
     m_octree->getMetricMin(minX, minY, minZ);
     m_octree->getMetricMax(maxX, maxY, maxZ);

     octomap::point3d minPt(minX, minY, minZ);
     octomap::point3d maxPt(maxX, maxY, maxZ);
     octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
     octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

     ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

     // add padding if requested (= new min/maxPts in x&y):
     double halfPaddedX = 0.5*m_minSizeX;
     double halfPaddedY = 0.5*m_minSizeY;
     minX = std::min(minX, -halfPaddedX);
     maxX = std::max(maxX, halfPaddedX);
     minY = std::min(minY, -halfPaddedY);
     maxY = std::max(maxY, halfPaddedY);
     minPt = octomap::point3d(minX, minY, minZ);
     maxPt = octomap::point3d(maxX, maxY, maxZ);

     octomap::OcTreeKey paddedMaxKey;
     if (!m_octree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)){
       ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
       return;
     }
     if (!m_octree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)){
       ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
       return;
     }

     ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
     assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

     m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
     m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
     m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;

     int mapOriginX = minKey[0] - m_paddedMinKey[0];
     int mapOriginY = minKey[1] - m_paddedMinKey[1];
     assert(mapOriginX >= 0 && mapOriginY >= 0);

     // might not exactly be min / max of octree:
     octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
     double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
     m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
     m_gridmap.info.resolution = gridRes;
     m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
     m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
     if (m_maxTreeDepth != m_treeDepth){
       m_gridmap.info.origin.position.x -= m_res/2.0;
       m_gridmap.info.origin.position.y -= m_res/2.0;
     }

     // workaround for  multires. projection not working properly for inner nodes:
     // force re-building complete map
     if (m_maxTreeDepth < m_treeDepth)
       m_projectCompleteMap = true;


     if(m_projectCompleteMap){
       ROS_DEBUG("Rebuilding complete 2D map");
       m_gridmap.data.clear();
       // init to unknown:
       m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

     } else {

        if (true){
           ROS_DEBUG("2D grid map size changed to %dx%d", m_gridmap.info.width, m_gridmap.info.height);
           adjustMapData(m_gridmap, oldMapInfo);
        }
        nav_msgs::OccupancyGrid::_data_type::iterator startIt;
        size_t mapUpdateBBXMinX = std::max(0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0]))/int(m_multires2DScale));
        size_t mapUpdateBBXMinY = std::max(0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1]))/int(m_multires2DScale));
        size_t mapUpdateBBXMaxX = std::min(int(m_gridmap.info.width-1), (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0]))/int(m_multires2DScale));
        size_t mapUpdateBBXMaxY = std::min(int(m_gridmap.info.height-1), (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1]))/int(m_multires2DScale));

        assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
        assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

        size_t numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;

        // test for max idx:
        uint max_idx = m_gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
        if (max_idx  >= m_gridmap.data.size())
          ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, m_gridmap.data.size(), m_gridmap.info.width, m_gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);

        // reset proj. 2D map in bounding box:
        for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j){
           std::fill_n(m_gridmap.data.begin() + m_gridmap.info.width*j+mapUpdateBBXMinX,
                       numCols, -1);
        }

     }
     m_mapPub.publish(m_gridmap);

}

// interrupt handler
/*void save_interrupt(int s){
    // octo tree cooefficient
    octomap::OcTree tree( 0.05 );
 
    for (int i = 0; i < output_cloud->points.size() ; i++)
    {
        tree.updateNode( octomap::point3d(  output_cloud->points[i].x, 
                                            output_cloud->points[i].y, 
                                            output_cloud->points[i].z), true );
    }
    // update octomap
    tree.updateInnerOccupancy();

    // output file to current working dir
    stringstream ss; //convert int to str
    ss << output_count;
    string idx = ss.str();
    tree.writeBinary( "output_bt" + idx + ".bt" );    // saving .bt file
    pcl::io::savePCDFileASCII ("output_pcd" + idx + ".pcd", *output_cloud);

    cout<<"SUCCESS: Output .bt file is saved!! "<<endl;
    output_count++;
 
}*/


int main( int argc, char** argv )
{
    char c;
    // start ros
    ros::init(argc, argv, "loam_to_octomap");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1, octo_callback);
    m_mapPub = n.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, m_latchedTopics);

    ros::Rate r(10);
    
    // create > save .bt file interrupt
    //signal(SIGINT,save_interrupt);

    ros::spin();
    
    return 0;

}
