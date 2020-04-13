#include <iostream> 
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <float.h>

#define INF 1000.0
#define pi 3.145926
#define full_deg 6.28319
#define fric_deg 0.785398
 
using namespace std;
typedef struct{
  double x;
  double y;
}Point;
typedef vector<string> StringVector;
typedef vector<float> FloatVector;
typedef vector<int> IntVector;
typedef vector<Point> PointVector;
typedef vector<pair<Point,Point>> PointPairs;
typedef vector<vector<int>> PointVectorVec;
typedef vector<pair<int,int>> IntIntVec;

typedef struct{
Point robot_1;
int angle_1;
Point robot_2;
int angle_2;
}Final_result;


bool onSegment (Point o_SP, Point o_SQ, Point o_SR)
{
    if (o_SQ.x <= std::max(o_SP.x, o_SR.x) && o_SQ.x >= std::min(o_SP.x, o_SR.x) && o_SQ.y <= std::max(o_SP.y, o_SR.y) && o_SQ.y >= std::min(o_SP.y, o_SR.y))    
        return true;
    return false;
}



int orientation (Point o_SP, Point o_SQ, Point o_SR)
{
    double nVal = ((o_SQ.y - o_SP.y) * (o_SR.x - o_SQ.x)) - ((o_SQ.x - o_SP.x) * (o_SR.y - o_SQ.y));
    if (nVal == 0.0) return 0;  
    return (nVal > 0.0) ? 1: 2; 
}



bool doIntersect (Point o_SP1, Point o_SQ1, Point o_SP2, Point o_SQ2)
{

    int nO1 = orientation (o_SP1, o_SQ1, o_SP2);
    int nO2 = orientation (o_SP1, o_SQ1, o_SQ2);
    int nO3 = orientation (o_SP2, o_SQ2, o_SP1);
    int nO4 = orientation (o_SP2, o_SQ2, o_SQ1);

    if ((nO1 != nO2) && (nO3 != nO4))return true;
    if (nO1 == 0 && onSegment (o_SP1, o_SP2, o_SQ1)) return true;

    if (nO2 == 0 && onSegment (o_SP1, o_SQ2, o_SQ1)) return true;

    if (nO3 == 0 && onSegment (o_SP2, o_SP1, o_SQ2)) return true;

    if (nO4 == 0 && onSegment (o_SP2, o_SQ1, o_SQ2)) return true;

    return false;
}


bool isInside (PointVector o_SPolygon, int nNumber, Point o_SP)
{
    if (nNumber < 3)  return false;
    Point o_SPointExtreme = {INF, o_SP.y};
    int nCounting = 0, nI = 0;
    do
    {
        int nNext = (nI+1) % nNumber;
        if (doIntersect (o_SPolygon[nI], o_SPolygon[nNext], o_SP, o_SPointExtreme))
        {

            if (orientation (o_SPolygon[nI], o_SP, o_SPolygon[nNext]) == 0)
               return onSegment (o_SPolygon[nI], o_SP, o_SPolygon[nNext]);
            nCounting++; 
        }
        nI = nNext;
    } while (nI != 0);
    return nCounting & 1; 
}
 

int boundingBoxCheck (PointVector o_SPolygon,Point o_SP,int  nNoOfPoints)
{

    return (isInside (o_SPolygon, nNoOfPoints, o_SP));
}



void printVertices(PointVector points){

  for (int i=0;i<points.size();i++){
    //cout<<points[i].x<<","<<points[i].y<<endl;
  }
}

PointVector ReadVertices(const string& filename, char sep)
{
  
  ifstream src(filename);
  FloatVector vertices;
  PointVector output;
  if (!src)
  {
    cerr << "\aError opening file.\n\n";
    exit(EXIT_FAILURE);
  }
  string buffer;

  while(getline(src, buffer))
  {
    size_t strpos = 0;
    size_t endpos;
    endpos= buffer.find(sep); 
    while (endpos < buffer.length())
    {  
      vertices.push_back(stof(buffer.substr(strpos,endpos - strpos)));
      strpos = endpos + 1;
      endpos = buffer.find(sep, strpos);
    }
    vertices.push_back(stof(buffer.substr(strpos)));
  }

  for (int i=0;i< vertices.size();i=i+2){
    Point point_temp;
    point_temp.x= vertices[i];
    point_temp.y= vertices[i+1];
    output.push_back(point_temp);
  }

  return output;
}

PointPairs generatePointPairs(PointVector points){
  PointPairs output;
 //cout<<endl;
  for(int i=0;i<points.size()-1;i++){
    output.push_back(make_pair(points[i],points[i+1]));
  }
  output.push_back(make_pair(points[points.size()-1],points[0]));   
  return output;
}

PointVector generateGraspPositions(PointPairs pointPairs){
  PointVector output;
  for (int i=0;i<pointPairs.size();i++)
  {
    Point mid_point;
    mid_point.x = (pointPairs[i].first.x + pointPairs[i].second.x)/2;
    mid_point.y = (pointPairs[i].first.y + pointPairs[i].second.y)/2;
    output.push_back(mid_point);
  }
  return output;
}

void printPointPairs(PointPairs pointpairs){
  //cout<<"Point pairs are"<<endl;
  for(int i=0;i<pointpairs.size();i++){
    //cout<< pointpairs[i].first.x<<","<< pointpairs[i].first.y << "\t"<< pointpairs[i].second.x << ","<<pointpairs[i].second.y<<endl;
  }
  //cout << endl;
}

PointPairs computeFrictionCones(PointVector grasp_points,PointPairs pointPairs,PointVector points){
  double height_of_cone = 5;
  FloatVector slope_angles;
  for (int i=0;i< pointPairs.size();i++){
    float slope = (float)(pointPairs[i].second.y - pointPairs[i].first.y) / (float)(pointPairs[i].second.x - pointPairs[i].first.x);
    slope = -1/slope;
    slope_angles.push_back(atan(slope));    
    //cout<< "slope is "<<round(atan(slope) * 180/pi)<<endl;
  }
  PointPairs cone_pairs;
  for (int i=0;i<slope_angles.size();i++){
    double left_slope=0,right_slope=0;
    Point left_point,right_point;
    left_slope = fmod((full_deg + slope_angles[i] + fric_deg),full_deg);
    right_slope = fmod((full_deg + slope_angles[i] - fric_deg), full_deg) ;
    left_point.x = grasp_points[i].x + (0.5* cos(left_slope)); // 0.5 to check if the point is inside the poolygn or not.
    left_point.y = grasp_points[i].y + (0.5* sin(left_slope));
    right_point.x = grasp_points[i].x + (0.5* cos(right_slope));
    right_point.y = grasp_points[i].y + (0.5* sin(right_slope));
    if( (!boundingBoxCheck(points,left_point,points.size())) ||  (!boundingBoxCheck(points,right_point,points.size())))
    {
      left_slope = left_slope + pi;
      right_slope = right_slope + pi;
      
      left_point.x = grasp_points[i].x + (0.5* cos(left_slope));
      left_point.y = grasp_points[i].y + (0.5* sin(left_slope));
      right_point.x = grasp_points[i].x + (0.5* cos(right_slope));
      right_point.y = grasp_points[i].y + (0.5* sin(right_slope));      
    }
    
    left_point.x = left_point.x + (height_of_cone * cos(left_slope)); // extending the cone size by height_of_cone variable. 
    left_point.y = left_point.y + (height_of_cone * sin(left_slope));
    right_point.x = right_point.x + (height_of_cone * cos(right_slope));
    right_point.y = right_point.y + (height_of_cone * sin(right_slope));        
    cone_pairs.push_back(make_pair(right_point,left_point));
  }

  //cout<< "cone_pairs size is "<< cone_pairs.size()<<endl;
  printPointPairs(cone_pairs);

  return cone_pairs;
  
}

PointVectorVec computeGraspPairs(PointVector grasp_points,PointPairs pointPairs){
  PointVectorVec output;

  for(int i=0;i<grasp_points.size();i++)
  {
    PointVector temp;
    vector<int> possible_grasp_points;  
    temp.push_back(pointPairs[i].first);
    temp.push_back(pointPairs[i].second);
    temp.push_back(grasp_points[i]);
    for(int j=0;j<grasp_points.size();j++){
      if(i==j)
        continue;
      if(boundingBoxCheck(temp,grasp_points[j],temp.size()))
      {
        possible_grasp_points.push_back(j);
      }
    }
    output.push_back(possible_grasp_points);
  }

  return output;
}

void printOptimalGrasps(IntIntVec input){
  for(int i=0;i<input.size();i++){
    //cout<<input[i].first<<","<<input[i].second<<endl;
  }
}

IntIntVec removeDuplicatesOfOptimalGrasps(IntIntVec input){
  IntIntVec output;
  for(int i=0;i<input.size();i++){
    for(int j=0;j<input.size();j++){
      if((input[i].first == input[j].second) && (input[i].second == input[j].first))
        input.erase(input.begin()+j);        
    }
  }
  return input;
  
}

IntIntVec findOptimalGrasp(PointVectorVec vector_points){
  IntIntVec output;  
  for(int i=0;i<vector_points.size();i++){
    for(int j=0;j<vector_points[i].size();j++){
      vector<int>::iterator it; 
      it = std::find(vector_points[vector_points[i][j]].begin(),vector_points[vector_points[i][j]].end(),i);
      if(it != vector_points[vector_points[i][j]].end()){
        output.push_back(make_pair(i,vector_points[i][j]));
      }
      else{
        continue;
      }
    }
  }
  printOptimalGrasps(removeDuplicatesOfOptimalGrasps(output));
  return removeDuplicatesOfOptimalGrasps(output);
}



void printVectorVec(PointVectorVec vector_points){
  for(int i=0;i<vector_points.size();i++){
    for(int j=0; j < vector_points[i].size(); j++){
      //cout<< vector_points[i][j]<<"\t";
    }
    //cout<<endl;
  }
}

void getDistance(double * distance, double x1 ,double y1, double x2, double y2)
{
    double a = x1-0;
    double b = y1-0;
    double c =  x2-0;
    double d =  y2-0;
    *distance = sqrt(pow(a, 2) + pow(b, 2)) + sqrt(pow(c, 2) + pow(d, 2));
}


Final_result getFinalOptimalGrasp(PointVector grasp_points,IntIntVec pairs , IntVector approach_angles){

  Final_result output;
  double smallest_distance = DBL_MAX;
  int smallest_pair=0;
  for(int i=0;i<pairs.size();i++){
    double temp=0.0;
    getDistance(&temp,grasp_points[pairs[i].first].x,grasp_points[pairs[i].first].y,grasp_points[pairs[i].second].x,grasp_points[pairs[i].second].y );
    //cout<< "The distances of pair are" << temp<<endl;
    if(temp < smallest_distance && (approach_angles[pairs[i].first] == 180 - approach_angles[pairs[i].second]) ){
      smallest_pair = i;
      smallest_distance = temp;
    }
    else
      continue;
  }
  //cout<< "The smallest pair is "<< pairs[smallest_pair].first<< ","<<pairs[smallest_pair].second<<endl;
  output.robot_1 = grasp_points[pairs[smallest_pair].first];
  output.angle_1 = approach_angles[pairs[smallest_pair].first];
  output.robot_2 = grasp_points[pairs[smallest_pair].second];
  output.angle_2 = approach_angles[pairs[smallest_pair].second];
  

  return output;
  
}

vector<int> calculate_approach_angles(PointVector graspPositions) {
  vector<int> output;
  for(int i=0;i<graspPositions.size();i++){
    double angle= atan2(graspPositions[i].y,graspPositions[i].x)*180/pi;
    //cout << "angle is "<<((int)(round(angle) + 180) % 360)<<endl;
    output.push_back(((int)(round(angle) + 180) % 360));
  }
  return output;
}


int main(){
  string filename = "vertices.txt";
  ofstream out_file;
  out_file.open ("grasp_data.txt");

  PointVector points = ReadVertices(filename, '\t');

  //cout<< "Points from file are " << points.size()<<endl;
  printVertices(points);

  PointPairs pointPairs= generatePointPairs(points);

  printPointPairs(pointPairs);

  PointVector graspPositions = generateGraspPositions(pointPairs);
  IntVector approach_angles= calculate_approach_angles(graspPositions);

  //cout << "No of grasp Positions" << graspPositions.size()<<endl;
  printVertices(graspPositions);

  PointPairs cone_coordinates = computeFrictionCones(graspPositions,pointPairs,points);

  PointVectorVec vectors_of_cone_coordinates = computeGraspPairs(graspPositions,cone_coordinates);
  //cout<< "Size of vectors_of_cone_coordinates "<< vectors_of_cone_coordinates.size()<<endl;
  printVectorVec(vectors_of_cone_coordinates);

  IntIntVec pair_of_Optimal_Grasps = findOptimalGrasp(vectors_of_cone_coordinates);
  
  Final_result final_grasp_position = getFinalOptimalGrasp(graspPositions,pair_of_Optimal_Grasps,approach_angles);

  out_file<< (long double)final_grasp_position.robot_2.x*0.1<<" "<<(long double)final_grasp_position.robot_2.y*0.1<<" "<<final_grasp_position.angle_2<<endl;
  out_file<< (long double)final_grasp_position.robot_1.x*0.1<<" "<<(long double)final_grasp_position.robot_1.y*0.1<<" "<<final_grasp_position.angle_1<<endl;
  out_file.close();
  
  return 0;
}
