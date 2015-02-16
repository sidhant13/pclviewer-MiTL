/******************************************
 *Project: PCL_Visualization
 *- The main class to read an array of point cloud files iteratively and cast them into point cloud objects.
 *- Optionally add triangulation and color information to the point cloud by editing the ./config/property.ini file.
 *- Display the point cloud, each file for 1 sec and the last one for additional 2 seconds.
 *- If a user clicks on any point in the cloud, save that info into a file.
 *-
 *- Display of point cloud runs on main thread and the point cloud formation and tranformation happens on concurrent thread.
 ******************************************/

#include <iostream>
#include <fstream>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <boost/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <time.h>
#include <map>
#include <sys/time.h>
#include <ctime>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "triangulate.h"
#include "parse.h"

using std::cout; using std::endl;

/**Global Variables**/
const std::string property_key[][2]={{"Triangulate","true"},                //The first value defines the property key and second its default value
                                     {"zoom1","Red"},
                                     {"zoom2","Orange"},
                                     {"zoom3","Blue"},
                                     {"zoom4","Green"}
                                    };
int view_r(0); int view_l(0); int view_f(0);
std::map<std::string,std::string> properties;
/**Global Variables**/

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> pclviewer;

//Structure for the arguments to pass to the visualizer callback functions.
struct callback_args{
  pclviewer viewerPtr;
  pcl::PointXYZ clicked_point;
  bool clicked=false;
  bool type;
  bool exit=false;
  std::string input;
};

//Structure for the arguments to pass to the concurrent thread preaparing point cloud and triangulation geometry
struct return_args_pcthread{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
  pcl::PolygonMesh triangles;
}*return_objects;


/*
 * Write the timestamp and target coordinates selected by the user to a file saved in the res fodler by default.
*/
int writeToFile(pcl::PointXYZ p, std::string output_file){

    using namespace boost::posix_time;
    ofstream outfile;
    ptime now = second_clock::local_time();
    outfile.open(output_file.c_str() , ios::app | ios::out );
    if(outfile.fail()){
        cout << "File could not be opened" << endl;
        return 0;
    }

    outfile << to_simple_string(now) << endl;
    outfile << p.x  << " " << p.y << " " << p.z << "\n" << endl;
    outfile.close();
    system(("gedit " + output_file).c_str() );
}

/*
 *  Function registered to KeyEvent callback.
*/
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* args)
{
    //If a key is pressed without first selecting a point, then invalidate the event.
    struct callback_args* data = (struct callback_args *)args;
    if(!data->clicked)
        return;

    //If y is pressed, confirm the target and update the callback args with the same.
    if (event.getKeySym () == "y" && event.keyDown ()){
        data->input="y";
        cout << "[KeyEventCallback] y Pressed. Target confirmed with coordinates " << data->clicked_point.x << " " << data->clicked_point.y << " " << data->clicked_point.z << endl;
        data->viewerPtr->addText("y pressed. Target saved to file. Exiting", 250.0 , 40.0 , 12.0, 1.0, 1.0, 1.0, "user response",view_f);
        //data->viewerPtr->spinOnce(100);
        data->clicked=false;
        data->exit=true;
    }
    //If n is pressed update the callback args with the same.
    else if(event.getKeySym () == "n" && event.keyDown ()){
        data->input="n";
        data->viewerPtr->addText("n pressed. Continuing to zoom", 250.0 , 40.0 , 12.0, 1.0, 1.0, 1.0, "user response",view_f);
        cout << "[KeyEventCallback] n pressed. Continuing to zoom" << endl;
        data->viewerPtr->spinOnce(100);
        data->viewerPtr->removeShape("user prompt");
        data->viewerPtr->removeShape("user response");
        data->viewerPtr->removeShape("point_marker");
        data->clicked=false;
    }
}

/*
 * Function registered to point clicking callback.
*/
void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
    cout << "\n";
    struct callback_args* data = (struct callback_args *)args;

    //If clicked twice, then invalidate the second one.
    if(data->clicked){
        cout << "[PointClickCallback] Note:Clear user prompt first"  << endl;
        return;
    }

    int idx = event.getPointIndex ();
    if (idx == -1)
       return;

    pcl::PointXYZ point;
    event.getPoint(point.x,point.y,point.z);
    cout << "[PointClickCallback] Picked point with coordinates " << point.x << " " << point.y << " " << point.z << endl;

    //Update the callback args to provide the main function with the coordinates of target and also update the viewer object with the sphere at the target position.
    data->viewerPtr->addSphere(point, 5.0, 255.0, 255.0, 255.0, "point_marker");
    data->viewerPtr->addText("Press y to confirm and n to discard", 20.0 , 40.0 , 12.0, 1.0, 1.0, 1.0, "user prompt", view_f);
    cout << "[PointClickCallback] Asking for user permission" << endl;

    data->viewerPtr->spinOnce();
    data->clicked_point=point;
    data->clicked=true;
}


/*
*Add the point cloud to the visualizer object, update point cloud, create viewports and adjust camera.
*/
pclviewer initializePointCloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                                                           pcl::PolygonMesh triangles,
                                                                           pclviewer viewer, int iteration,
                                                                           std::string triangulate
                                                                           )
{
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  int point_size=1;
  if(iteration==0){
        viewer->createViewPort(0.0, 0.0, 0.33, 1.0, view_r);
        viewer->createViewPort(0.33, 0.0, 0.66, 1.0, view_f);
        viewer->createViewPort(0.66, 0.0, 1.0, 1.0, view_l);
        viewer->setBackgroundColor (0, 0, 0);
        viewer->setSize(1300,650);
  }
  viewer->removeAllPointClouds(view_r);
  if(triangulate=="true"){
    viewer->removePolygonMesh("polygon1",view_r);
    viewer->removePolygonMesh("polygon2",view_f);
    viewer->removePolygonMesh("polygon3",view_l);
  }
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud,rgb,"Right View",view_r);
  if(triangulate=="true")
  viewer->addPolygonMesh(triangles,"polygon1", view_r);
  viewer->createViewPortCamera(view_r);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "Right View");
  viewer->setCameraPosition(-150.0*3,-500.0*3,0.0,  0.0,-1.0,0.0,  0.0,0.0,1.0, view_r);

  viewer->removeAllPointClouds(view_f);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud,rgb,"Front View",view_f);
  if(triangulate=="true")
  viewer->addPolygonMesh(triangles,"polygon2", view_f);
  viewer->createViewPortCamera(view_f);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "Front View");
  viewer->setCameraPosition(-400.0*2,250.0*2,0.0,  -1.0,0.0,0.0,  0.0,0.0,1.0, view_f);


  viewer->removeAllPointClouds(view_l);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud,rgb,"Left View",view_l);
  if(triangulate=="true")
  viewer->addPolygonMesh(triangles,"polygon3", view_l);
  viewer->createViewPortCamera(view_l);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "Left View");

  //viewer->addCoordinateSystem(100,view_r);
  viewer->setCameraPosition(150.0*3,500.0*3,0.0,  0.0,1.0,0.0,  0.0,0.0,1.0, view_l);
  return (viewer);
}

//Function to get user inputs from .ini file
void getProperties(parse* mParse){
    int num_properties=5;
    for(int i=0; i<num_properties; i++){
       mParse->getValue( property_key[i][0], properties[property_key[i][0]], properties[property_key[i][1]]);
    }
}

/** Read the point cloud file and form a cloud pointer object **/
int preparePointCloud(int itr, parse mParse, std::string filename)
{
    cout << "\n";
    ifstream pcfile;
    cout << "[PreparePointCloud] Opening file " << filename << endl;
    pcfile.open(filename.c_str());
    if(pcfile.fail()){
          cout << "[PreparePointCloud] File could not be opened. Exiting\n";
          return 0;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    return_objects->cloud_ptr=cloud_ptr;
    std::string line;
    int rgb;
    rgb= mParse.nameToRgb(properties[property_key[itr+1][0]]);

    pcl::PointXYZRGB simple_point;
    while (std::getline(pcfile, line)){
        std::istringstream iss(line);
        iss >> simple_point.x;     iss >> simple_point.y;      iss >> simple_point.z;
        simple_point.rgba=rgb;
        cloud_ptr->points.push_back(simple_point);
    }
    pcfile.close();

    cloud_ptr->width = (int) cloud_ptr->points.size();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = true;
    cout << "[PreparePointCloud] Rendering " << cloud_ptr->width << " points\n";

    if(properties[property_key[0][0]]=="true" || properties[property_key[0][0]]=="True"){
      triangulate t(cloud_ptr,itr+1);                                                       //Passing the iteration parameter to scale the Triangulation parameters according to point cloud dataset
      return_objects->triangles = t.getPolygonMesh();
    }

}

//Main Program
int main (int argc, char** argv)
{
  /**Read the .ini Property File**/
  parse mParse;
  if(mParse.getFileReadErrorFlag())             //exit if .INI file was not opened
      return 0;
  getProperties(&mParse);
  cout << "\n";
  /**Read the .ini Property File**/

  pclviewer viewer (new pcl::visualization::PCLVisualizer ("Point Cloud Viewer"));          //The PCL visualizer object

  /**File parametres**/
  const std::string rel_filepath="../res/";                                                 //Path where point cloud files will be searched for
  const std::string def_pcdfile[4]={"l1.xyz", "l2.xyz", "l3.xyz", "l4.xyz"};                //Name of the point cloud files.
  std::string output_file="target.log";                                                     //Name of the file where the user selected coordiantes/target will be stored.
  /**File Parameters**/

  int itr=0,ms1, ms2, ms3=0,ms4=0;
  int spintime=1000;                                                                        //The time for which a particular zoom level will be dispalyed in ms. default 1 sec
  std::string string_time;
  double elapsed_time;
  struct timeval tp;
  struct callback_args cb_args;                                                             //The struct passed to pcl visualizer callback functions
    cb_args.viewerPtr= viewer;
  //struct return_args_pcthread* return_objects;

  /**Initialize Thread object that will run concurrently to this main program**/
  boost::thread t;
  return_objects= new return_args_pcthread;                                                 //Memory assignment to the
  /**Initialize Thread object that will run concurrently to this main program**/

  preparePointCloud(itr, mParse, rel_filepath + def_pcdfile[0]);                            //Read the first point cloud file and its corresponding color from the properties fetched

  /**Start an infinite loop and ext after all the files, 4 in number are read and displayed**/
  while(1)
  {
    //Add the cloud to the visualizer object, create viewports and adjust camera
    viewer = initializePointCloud(return_objects->cloud_ptr, return_objects->triangles, viewer, itr, properties[property_key[0][0]]);
    //Prepare Time string to be displayed in the middle front viewport
    elapsed_time= 4.0-itr;
    string_time= boost::lexical_cast<std::string>(elapsed_time);

    /**Add the time string to the viewer object and register callback**/
    if(itr==0){
        viewer->addText (string_time+"s", 250.0 , 20.0 , 12.0, 1.0, 1.0, 1.0, "time",view_f);
        viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);
        viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&cb_args);
    }
    viewer->updateText (string_time+"s", 250.0 , 20.0 , 12.0, 1.0, 1.0, 1.0, "time");
    /****/

    // Runt the preparePointCloud() function on a paralled thread to
    // Prepare cloud pointer and triangulated geometry for the next iteration so as to make the transition from zoom level to another smooth**/
    if(itr<3)
        t=boost::thread(preparePointCloud,itr+1, mParse, rel_filepath + def_pcdfile[itr+1]);

    /**Start the timer to control the lifetime of a particular view**/
    gettimeofday(&tp, NULL);
    ms1 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    int _ms1=ms1;
    /**Start the timer to control the lifetime of a particular view**/

    //The loop that runs for the lifetime of one zoom level i.e 1 sec
    while(!viewer->wasStopped ())
    {
        viewer->spinOnce(10);

        gettimeofday(&tp, NULL);
        ms2 = tp.tv_sec * 1000 + tp.tv_usec / 1000;

        /**Code to check whether the spintime of a particular view has expired or not**/
        if(ms2-ms1>spintime){
            if(itr==3){
                spintime= 3000;
                itr++;
                viewer->addText ("Into 2 second extra time", 20.0 , 20.0 , 12.0, 1.0, 0.0, 0.0, "extratime",view_f);
                continue;
            }
            break;
        }
        //Display the running time from 4s to -2 sec
        string_time = boost::lexical_cast<std::string>(elapsed_time-((ms2-ms1)/1000.0));
        if(itr>3)
            viewer->updateText (string_time.substr(0,4)+"s", 250.0 , 20.0 , 12.0, 1.0, 1.0, 1.0, "time");
        else
            viewer->updateText (string_time.substr(0,3)+"s", 250.0 , 20.0 , 12.0, 1.0, 1.0, 1.0, "time");

        /**If a point was clicked on the viewer, pause time and wait for the user input. If user confirms then exit else start from the time where it stopped**/
        while(cb_args.clicked==true){
          viewer->spinOnce(100);
          gettimeofday(&tp, NULL);
          ms3 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
          ms3=ms3-ms2;
          ms1=_ms1+ms3;
        }
        if(cb_args.exit)
          break;
        _ms1=ms1;
  }

    //gettimeofday(&tp, NULL);
    //ms4 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  ms3=0;

  //early exit. If a user confirmed the target, straightaway exit without displaying the other point clouds.
  if(cb_args.exit){
     viewer->close();
     writeToFile(cb_args.clicked_point, rel_filepath + output_file);
     return 1;
  }

  if(itr>=3){
    viewer->close();
    return 1;
  }
  //wait for the preapePointCloud concurrent thread to finish before starting another itreation.
  t.join();
  itr++;
 }

}
