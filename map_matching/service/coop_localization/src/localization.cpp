#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <chrono>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/features2d.hpp"


using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using namespace std;
using namespace cv;
using namespace std::chrono_literals;
using namespace std::placeholders;

const int MAX_FEATURES = 100;
const int matches_PERCENT = 0.15f;

class Localisation : public rclcpp::Node {
   public:
    Localisation() : Node("localisation"){
        own_map_sub = this->create_subscription<OccupancyGrid>("/robot1/map", 10, std::bind(&Localisation::OwnMapCallback, this, _1));
        other_map_sub = this->create_subscription<OccupancyGrid>("/robot2/global_costmap/costmap", 10, std::bind(&Localisation::OtherMapCallback, this, _1));

    }

    private:
     rclcpp::Subscription<OccupancyGrid>::SharedPtr own_map_sub;
     rclcpp::Subscription<OccupancyGrid>::SharedPtr other_map_sub;
     OccupancyGrid own_map;
     OccupancyGrid other_map;
     Mat own_img;
     Mat other_img;
     std::vector<KeyPoint> own_KeyPoints, other_KeyPoints;
     Mat own_descriptors, other_descriptors;
     Ptr<Feature2D> orb;
     std::vector<std::vector<DMatch>> knn_matches;
     std::vector<DMatch> matches;
     std::vector<DMatch> goodMatches;
     Mat img_matches;
     vector<Point2f> linevector;

     void OwnMapCallback(const OccupancyGrid::SharedPtr msg_owm) {
         own_map = *msg_owm;
         own_img = mapToCvMAt(own_map);
         imwrite("robot1_map.png", own_img);
         auto keypoint_robot1 = findKeypointsAndDescriptors(own_KeyPoints, own_img, own_descriptors);
         imwrite("robot1_keypoints.png", keypoint_robot1);
         MatchFeatures(own_descriptors, other_descriptors);
         TestMatches();
         extracGoodMatches();
         showMatches();
         imwrite("desc1.png", own_descriptors);
         imwrite("desc2.png", other_descriptors);
     }

     void OtherMapCallback(const OccupancyGrid::SharedPtr msg_other){
         other_map = *msg_other;
         other_img = mapToCvMAt(other_map);
         imwrite("robot2_map.png", other_img);
         auto keypoint_robot2 = findKeypointsAndDescriptors(other_KeyPoints, other_img, other_descriptors);
         imwrite("robot2_keypoint.png", keypoint_robot2);

     }

     Mat mapToCvMAt(const OccupancyGrid &map){
         auto width = map.info.width;
         auto heigth = map.info.height;
         Mat img(heigth, width, CV_8U);

         for (unsigned int row = 0; row < heigth; row++) {
             for (unsigned int col = 0; col < width; col++){
                 auto cell = map.data[row * width + col];
                 try {
                     switch (cell)
                     {
                     case 0:
                         img.at<uint8_t>(row, col) = 255;
                         break;
                     case 1:
                         img.at<uint8_t>(row, col) = 0;
                         break;
                     case 100:
                         img.at<uint8_t>(row, col) = 0;
                         break;
                     case -1:
                         img.at<uint8_t>(row, col) = 255;
                         break;
                     default:
                         img.at<uint8_t>(row, col) = 255;
                         break;
                     }
                 } catch(std::out_of_range const &exc){
                     cerr << "Out of range";
                 }
             }
         }
         if(img.empty()){
         }
         return img;
     }

     Mat findKeypointsAndDescriptors(std::vector<KeyPoint> &key, Mat map, Mat &descriptor){
         orb = ORB::create(MAX_FEATURES);
         orb->detectAndCompute(map, Mat(), key, descriptor);
         Mat img_keypoints;
         drawKeypoints(map, key, img_keypoints, Scalar(240, 230, 140));
         imwrite("des.png", descriptor);
         return img_keypoints;
     }

     void MatchFeatures(const Mat own_des , const Mat other_des){
         auto matcher = BFMatcher(NORM_HAMMING2, true);
         matches.clear();
         matcher.match(own_descriptors, other_descriptors, matches, noArray());
         if(matches.empty()){
             cout << "TESSSSSSSSSSSST" << endl;
         }
         cout << "good:"<< matches.size() << endl;
         cout << "own:" << own_KeyPoints.size() << endl;
         cout << "other:" << other_KeyPoints.size() << endl;
     }

     void TestMatches(){
         vector<Point2f> own_points, other_points;
         for (size_t i = 0; i < matches.size(); i++) {
             own_points.push_back(own_KeyPoints[matches[i].queryIdx].pt);
             other_points.push_back(other_KeyPoints[matches[i].trainIdx].pt);
             linevector.push_back(Point2f(own_points[i].x - other_points[i].x, own_points[i].y - other_points[i].y));
             cout << "TESTTTT" << endl;
         }
         cout << "own_points:" << own_points.size() << own_points[5] << endl;
         cout << "other_points:" << other_points.size() << other_points[5] << endl;
         Mat h = findHomography(own_points, other_points, RANSAC);
         imwrite("HOMOGRA.png", h);
         cout << h << endl;
         cout << "Homogr:" << h.size << endl;
            
             std::vector<Point2f> obj_corners(4);
    
    obj_corners[0] = Point2f(0, 0);
    obj_corners[1] = Point2f( (float)other_img.cols, 0 );
    obj_corners[2] = Point2f( (float)other_img.cols, (float)other_img.rows );
    obj_corners[3] = Point2f( 0, (float)other_img.rows );
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, h);
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line(img_matches, scene_corners[0] + Point2f((float)other_img.cols, 0),
               scene_corners[1] + Point2f((float)other_img.cols, 0), Scalar(0, 255, 0), 4);
    line( img_matches, scene_corners[1] + Point2f((float)other_img.cols, 0),
          scene_corners[2] + Point2f((float)other_img.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f((float)other_img.cols, 0),
          scene_corners[3] + Point2f((float)other_img.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f((float)other_img.cols, 0),
          scene_corners[0] + Point2f((float)other_img.cols, 0), Scalar( 0, 255, 0), 4 );
    //-- Show detected matches
     }

     void extracGoodMatches(){
         goodMatches.clear();
         Point2f goodMatchesdistnace = searchVector();
         int matchesCount = 0;
         for (size_t i = 0;i < matches.size(); i++) {
             if (linevector[i].x - 2 <= goodMatchesdistnace.x && linevector[i].x +2 >= goodMatchesdistnace.x &&
             linevector[i].y - 2 <= goodMatchesdistnace.y && linevector[i].y +2 >= goodMatchesdistnace.y){
                 goodMatches.push_back(matches[i]);
                 matchesCount++;
                 if (matchesCount == 4){
                     break;
                 }
             }
         }
     }

     Mat showMatches(){
         Mat img_Matches;
         drawMatches(own_img, own_KeyPoints, other_img, other_KeyPoints, goodMatches, img_Matches, Scalar::all(-1), Scalar::all(-1));
         imwrite("allmatches.png", img_Matches);
         return img_Matches;
     }

     float searchDistance(){
         vector<float> Distances;
         int DistancePair = 0;
         int MaxDistancePair = 0;
         float OutputDistance = 0.0;
         for (size_t i = 0; i < matches.size(); i++) {
             DistancePair = 0;
             for (size_t j = i+1; j < matches.size(); j++) {
                 if(matches[i].distance -1 <= matches[j].distance && matches[i].distance + 1 >= matches[j].distance){
                     DistancePair++;
                 }
             }
             if(DistancePair > MaxDistancePair){
                 MaxDistancePair = DistancePair;
                 OutputDistance = matches[i].distance;
             }
         }
         return OutputDistance;
     }
     Point2f searchVector(){
         cout << "TESTTTT 3" << linevector.size() << endl;
         int DistancePairFound = 0;
         int MaxDistancePair = 0;
         Point2f Output(0,0);
         for (size_t i = 0; i < linevector.size(); i++) {
             DistancePairFound = 0;
             cout << "TESTTTT 4" << endl;
             for (size_t j = i + 1; j < linevector.size(); j++) {
                 if(linevector[i].x -2 <= linevector[j].x && linevector[i].x + 2 >= linevector[j].x && 
                 linevector[i].y -2 <= linevector[j].y && linevector[i].y + 2 >= linevector[j].y){
                     DistancePairFound++;
                 }
             }
             if(DistancePairFound > MaxDistancePair){
                 MaxDistancePair = DistancePairFound;
                 Output = linevector[i];
             }
         }
         cout << "TESTTTT 5" << endl;
         return Output;
     }
};

    


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    auto node = make_shared<Localisation>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}