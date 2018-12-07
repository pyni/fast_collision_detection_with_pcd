#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h> 
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <fstream> 
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <math.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h> 
#define random(x) (rand()%x)
using namespace std; 
void CrossProduct(double a[3], double b[3], double ret[3])
{
    ret[0] = a[1] * b[2] - a[2] * b[1];
    ret[1] = a[2] * b[0] - a[0] * b[2];
    ret[2] = a[0] * b[1] - a[1] * b[0];
}

double DotProduct(double a[3], double b[3])
{
    double result;
    result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    return result;
}

double Normalize(double v[3])
{
    double result;

    result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    return result;
}

void Rotate(double** rotateMatrix, double u[3], double ret[3]){
    ret[0]=rotateMatrix[0][0]*u[0]+rotateMatrix[0][1]*u[1]+rotateMatrix[0][2]*u[2];
    ret[1]=rotateMatrix[1][0]*u[0]+rotateMatrix[1][1]*u[1]+rotateMatrix[1][2]*u[2];
    ret[2]=rotateMatrix[2][0]*u[0]+rotateMatrix[2][1]*u[1]+rotateMatrix[2][2]*u[2];
}

void RotationMatrix(double angle, double u[3], double rotatinMatrix[3][3])
{
    double norm = Normalize(u);
    
    u[0] = u[0] / norm;
    u[1] = u[1] / norm;
    u[2] = u[2] / norm;

    rotatinMatrix[0][0] = cos(angle) + u[0] * u[0] * (1 - cos(angle));
    rotatinMatrix[0][1] = u[0] * u[1] * (1 - cos(angle) - u[2] * sin(angle));
    rotatinMatrix[0][2] = u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));

    rotatinMatrix[1][0] = u[2] * sin(angle) + u[0] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][1] = cos(angle) + u[1] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][2] = -u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
      
    rotatinMatrix[2][0] = -u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][1] = u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][2] = cos(angle) + u[2] * u[2] * (1 - cos(angle));

}

//cal transport
void Calculation3d(double vectorBefore[3], double vectorAfter[3], double rotatinMatrix[3][3])
{
    double  rotationAxis[3];
    double rotationAngle;
    CrossProduct(vectorBefore, vectorAfter, rotationAxis);
    rotationAngle = acos(DotProduct(vectorBefore, vectorAfter) / Normalize(vectorBefore) / Normalize(vectorAfter));
    RotationMatrix(rotationAngle, rotationAxis, rotatinMatrix);
}

void Calculation4d(double vectorBefore[3], double vectorAfter[3], double rotatinMatrix[4][4])
{
    double rotate3d[3][3];
	Calculation3d(vectorBefore,vectorAfter, rotate3d);

	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			rotatinMatrix[i][j] = rotate3d[i][j];

	for(int i = 0; i < 3; i++)
	{
		rotatinMatrix[i][3] = 0;
		rotatinMatrix[3][i] = 0;
	}

	rotatinMatrix[3][3] = 1;
}
int
main (int argc, char** argv)
{


 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonan (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_plane (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/yuan/doc/suction_ws/src/readpcd2rviz/src/movefree2.ply", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  // draw the point cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  //viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "registered point cloud");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "registered point cloud");

  // draw the samples
  


  std::vector<double> maskset;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (2.2); 
 sor.setKeepOrganized(true);
 sor.filter (*cloud_filtered);




int chang=400;//(1000*0.001)
int kuan=400;//(1000*0.001)
double standardlength=0.2;//标准检测长宽
double suctionradius=0.01;



 for  (int f=0  ;f<cloud_filtered->points.size( );f++)
 {
if(isnan(cloud_filtered->points[f].x))
{ 
;
}
else
{
pcl::PointXYZ testpoint;
testpoint.x=cloud_filtered->points[f].x;
testpoint.y=cloud_filtered->points[f].y; 
testpoint.z=cloud_filtered->points[f].z;
cloud_nonan->points.push_back(testpoint);

}
}

cloud_filtered=cloud_nonan;
std::cout<<"the number of point cloud without plane:"<<cloud_filtered->points.size()  <<std::endl;//176162
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr transformed_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);
	n.setInputCloud (cloud_filtered);
	n.setSearchMethod (tree);
	n.setKSearch (15);
	n.compute (*normals);

std::cout<<"normal computation over"  <<std::endl;// 
int countnum=0;
	 for  (int f=0  ;f<normals->points.size( );f++)
	 {
	if( cloud_filtered->points[f].z>0.02 and cloud_filtered->points[f].z<0.07)
	{ 
	normals->points[f].normal_x=cloud_filtered->points[f].x/sqrt(cloud_filtered->points[f].x*cloud_filtered->points[f].x+cloud_filtered->points[f].y*cloud_filtered->points[f].y);
	 normals->points[f].normal_y=cloud_filtered->points[f].y/sqrt(cloud_filtered->points[f].x*cloud_filtered->points[f].x+cloud_filtered->points[f].y*cloud_filtered->points[f].y);
	 normals->points[f].normal_z=0;
	maskset.push_back(1.0);
	countnum=countnum+1;
	}
	else if(sqrt(cloud_filtered->points[f].x*cloud_filtered->points[f].x+cloud_filtered->points[f].y*cloud_filtered->points[f].y)<0.006 and cloud_filtered->points[f].z>0.12)
	{
	normals->points[f].normal_x=0;
	 normals->points[f].normal_y=0;
	 normals->points[f].normal_z=1.0;
	maskset.push_back(1.0);
	countnum=countnum+1;
	}
	else
	{
	normals->points[f].normal_x=0;
	 normals->points[f].normal_y=0;
	 normals->points[f].normal_z=0;
	maskset.push_back(0.0);
	}
	 }



 

std::cout<<"the number of normals:"  <<countnum<<std::endl;


	    // 可视化 
	 //viewer->addPointCloud<pcl::PointXYZ>( cloud, "samples cloudori");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples cloudori");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0,0.0, "samples cloudori");



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//下面开始对物体做变换



	Eigen::Matrix4f transform_0 = Eigen::Matrix4f::Identity();

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	// Here we defined a 45° (PI/4) rotation around the Z axis and a translation on the X axis. 
	float theta2 =  M_PI/3; // The angle of rotation in radians
	transform_0 (0,0) = cos (theta2);
	transform_0 (0,2) = -sin(theta2);
	transform_0 (2,0) = sin (theta2);
	transform_0 (2,2) = cos (theta2);
	//    (row, column)

	// Define a translation of 2.5 meters on the x axis.
	transform_0 (0,3) = 0.0;

	//    (row, column)

	// Define a translation of 2.5 meters on the x axis.
	 
	// Print the transformation
	//printf ("Method #1: using a Matrix4f\n");
	//std::cout << transform_0 << std::endl;


	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ >::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ > ());
	/*
	void pcl::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in, 
		                    pcl::PointCloud< PointT > &  cloud_out,  
		                    const Eigen::Matrix4f &  transform  ) 
	*/
	// Apply an affine transform defined by an Eigen Transform.
	pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_0);





	 transformed_normals=normals;


	for(int k=0 ; k<transformed_normals->points.size();k++)
	{ 
	// transformed_normals
	 Eigen::Vector4f v(transformed_normals->points[k].normal_x,transformed_normals->points[k].normal_y,transformed_normals->points[k].normal_z,1);
 
	 Eigen::Vector4f newv=transform_0*v;
//std::cout<< newv[0]<<  ","<<newv[1]<<  ","<< newv[2]<<  ","<< newv[3]<<std::endl;
	transformed_normals->points[k].normal_x=newv[0];
	transformed_normals->points[k].normal_y=newv[1];
	transformed_normals->points[k].normal_z=newv[2];

	}
 




/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//添加平面




double minposition=9999.9;

for(int k=0 ; k<transformed_cloud->points.size();k++)
	{  
               if(transformed_cloud->points[k].z< minposition )
                 minposition =transformed_cloud->points[k].z ;

	}

 

	 for  (int p=0  ;p<chang;p++)
	 for  (int q=0  ;q<kuan;q++)
	 {
	pcl::PointXYZ testpoint;
	testpoint.x=- 200*0.001+p*0.001;
	testpoint.y=- 200*0.001+q*0.001; 
	testpoint.z=minposition;
	transformed_cloud ->points.push_back(testpoint);

	pcl::Normal  pcltest;
        pcltest.normal_x=0;  
        pcltest.normal_y=0;  
        pcltest.normal_z=0;   
	transformed_normals->points.push_back(pcltest);



	 }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//下面对每个法线做碰撞检测
  countnum=0;
for(int k=0 ; k<transformed_normals->points.size();k++)
	{ 

         if(transformed_normals->points[k].normal_x==0 and transformed_normals->points[k].normal_y==0 and transformed_normals->points[k].normal_z==0 )
	         ;
         else
	{
countnum=countnum+1;
////////////////////////////////////////////////////////////////////////////////
//下面是对子的点云进行探讨
	double vector[3]  = {transformed_normals->points[k].normal_x,transformed_normals->points[k].normal_y,transformed_normals->points[k].normal_z};
 	double pointzero[3]  = {transformed_cloud ->points[k].x,transformed_cloud ->points[k].y,transformed_cloud ->points[k].z};

 //std::cout <<vector[0] << ","<<vector[1] << "," <<vector[2]   << std::endl;
	//    (row, column)

	// Define a translation of 2.5 meters on the x axis.
	 
	// Print the transformation
	//printf ("Method #1: using a Matrix4f\n");
	//std::cout << transform_1 << std::endl;


	// Executing the transformation
 
       //法线方程目前是点向式，点为transformed_cloud ->points[k]，方向为vectorBefore

//故点向式方程为:
//  (x-pointzero[0])/vector[0]=(y-pointzero[1])/ vector[1]=*(z-pointzero[2])/vector[2]=t
//故一般式方程为:
//vector[0]* x-vector[0]*pointzero[0] + vector[1]* y- vector[1]*pointzero[1] + vector[2]* y- vector[2]*pointzero[2] =0
//同时，过这个点的平面方程为:
//A(x-pointzero[0])+B(y-pointzero[1])+C(z-pointzero[2])=0
//则有:

//https://wenku.baidu.com/view/ed88f4bcfab069dc51220129.html


//pcl::PointCloud<pcl::PointXYZ >::Ptr transformed_cloud3 (new pcl::PointCloud<pcl::PointXYZ > ());

 
//pcl::PointCloud<pcl::PointXYZ >::Ptr transformed_cloud4 (new pcl::PointCloud<pcl::PointXYZ > ());

 


double A=vector[0];
double B=vector[1];

double C=vector[2];

double D=-vector[0]*pointzero[0]- vector[1]*pointzero[1]- vector[2]*pointzero[2];


        for  (int f=0;f<transformed_cloud->points.size();f++)
	{
double t=(transformed_cloud->points[f].x-pointzero[0])/vector[0];
double xc=vector[0]*t+pointzero[0];
double yc=vector[1]*t+pointzero[1];
double zc=vector[2]*t+pointzero[2];
double distancetoaxis=sqrt((xc-transformed_cloud->points[f].x)*(xc-transformed_cloud->points[f].x)+(yc-transformed_cloud->points[f].y)*(yc-transformed_cloud->points[f].y)+(zc-transformed_cloud->points[f].z)*(zc-transformed_cloud->points[f].z));
 //std::cout <<  A*(transformed_cloud->points[f].x-pointzero[0])+B*(transformed_cloud->points[f].y-pointzero[1])+C*(transformed_cloud->points[f].z-pointzero[2]) << std::endl;
double distancetoplanevector=  (A*(transformed_cloud->points[f].x-pointzero[0])+B*(transformed_cloud->points[f].y-pointzero[1])+C*(transformed_cloud->points[f].z-pointzero[2]))/sqrt( A*A+B*B+C*C)  ;
//std::cout << distancetoaxis<< distancetoplanevector<< std::endl;


            if(  distancetoaxis < suctionradius  and  distancetoplanevector  > 0.01)//这个说明这个法线方向有碰撞，则其对应的法线置0

	{
//transformed_cloud3->points.push_back(transformed_cloud->points[f]); 
            transformed_normals->points[k].normal_x=0;
            transformed_normals->points[k].normal_y=0;
            transformed_normals->points[k].normal_z=0;

	}

	}









//transformed_cloud4->points.push_back(transformed_cloud ->points[k]); 


 // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
 //	 viewer->addPointCloud<pcl::PointXYZ>( transformed_cloud2, "samples filter");
 //	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "samples filter");
 //	 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples filter"); 

	// viewer->addPointCloud<pcl::PointXYZ>( transformed_cloud3, "samples filter3");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "samples filter3");
 	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0,1.0, "samples filter3"); 

	// viewer->addPointCloud<pcl::PointXYZ>( transformed_cloud4, "samples filter4");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "samples filter4");
 //	 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples filter4"); 

//	viewer->addCoordinateSystem (0.1);
//	while (!viewer->wasStopped ())
//	 {
//	  viewer->spinOnce (100);
 //	  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
// }

 



	}



	}

std::cout<<"the number of normals:"  <<countnum<<std::endl;


//至此，物体+平面点云为transformed_cloud； 物体+平面点云法线为transformed_normals； 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//dropout + 记录



int rationumber=3000*1000/(transformed_cloud->points.size()/( chang*0.001*kuan*0.001/( standardlength*standardlength)));//我们只考虑0.2*0.2这部分区域的点云尽量接近3000
 std::cout<<transformed_cloud->points.size() <<";"<<chang*0.001*kuan*0.001/( standardlength*standardlength)<<std::endl;
  for (int gg=0;gg<1000;gg++)
{      
        char path[1000] ;
	sprintf(path,"/home/yuan/doc/suction_ws/src/readpcd2rviz/src/data/%d.txt",gg); 
	//std::cout<<path<<std::endl;
	ofstream outfile(path,ios::app); 
	//std::cout<<normals->points.size()<<","<<transformed_cloud->points.size()<<std::endl;
        int key=0;


        while (key==0)
{       key=1;
 	pcl::PointCloud<pcl::PointXYZ>::Ptr negpoint (new pcl::PointCloud<pcl::PointXYZ>);
 	pcl::PointCloud<pcl::PointXYZ>::Ptr pospoint (new pcl::PointCloud<pcl::PointXYZ>);
	//以下 是干扰球的参数
        double centerx=-double(random(200))/1000.0+0.1;
        double centery=-double(random(200))/1000.0+0.1 ;
        double centerz=-double(random(400))/1000.0+0.2;
	double radius=double(random(300))/1000.0;
	//以下 是随机初始化位置的参数
        double squarecenterx=-double(random(chang))/1000.0+0.001*chang*0.5 ;
        double squarecentery=-double(random(kuan))/1000.0+0.001*kuan*0.5 ;
        
	for(int k=0 ; k<normals->points.size();k++)
		{   
 //std::cout<< "rationumber：" <<rationumber<<std::endl;
         double rannum=double(random(1000));
          if ( rannum> rationumber) 
                continue; 
//std::cout<<k<<std::endl;
        if( sqrt((transformed_cloud->points[k].x-centerx)*(transformed_cloud->points[k].x-centerx)+(transformed_cloud->points[k].y-centery)*(transformed_cloud->points[k].y-centery)+(transformed_cloud->points[k].z-centerz)*(transformed_cloud->points[k].z-centerz))>radius  and transformed_cloud->points[k].x>(squarecenterx-standardlength*0.5) and transformed_cloud->points[k].x<(squarecenterx+standardlength*0.5) and transformed_cloud->points[k].y>(squarecentery-standardlength*0.5) and transformed_cloud->points[k].y<(squarecentery+standardlength*0.5))
	{



//outfile<<transformed_cloud->points[k].x<<" "<<transformed_cloud->points[k].y<<" "<<transformed_cloud->points[k].z<< " "<<double(normals->points[k].normal_x)<<" "<<double(normals->points[k].normal_y)<<" "<<double(normals->points[k].normal_z)<<" "<<double(maskset[k])<<std::endl;
	if (maskset[k]==1)
        {
        pospoint->points.push_back(transformed_cloud->points[k]);
	}
        else
        negpoint->points.push_back(transformed_cloud->points[k]);      
	} 
        }
         
        if ((negpoint->points.size()+ pospoint->points.size())<120 )
          key=0;
        else
            ;
        //  std::cout<<( negpoint->points.size()+ pospoint->points.size())<<std::endl;
        

        // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	 //viewer->addPointCloud<pcl::PointXYZ>( negpoint, "samples filter");
	  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples filter");
	   //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples filter");


	//  viewer->addPointCloud<pcl::PointXYZ>( pospoint, "samples filter2");
	//  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "samples filter2");
	//   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,0.0, "samples filter2");
//	  viewer->addCoordinateSystem (0.1);
//	while (!viewer->wasStopped ())
//	{
//	   viewer->spinOnce (100);
//	  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//	 }

}
	outfile.close();
 //

	  // draw the samples
}	 

 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//可视化

	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(transformed_cloud, transformed_normals, 1, 0.005, "normals");//(这里，每 10 个点显示一个)及每个法线的长度 
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.6,0.6, "normals");


	 viewer->addPointCloud<pcl::PointXYZ>( transformed_cloud, "samples filter");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "samples filter");
	 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples filter"); 
	viewer->addCoordinateSystem (0.1);
	while (!viewer->wasStopped ())
	 {
	  viewer->spinOnce (100);
	  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	 }
   // draw the samples
  //viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud2, "samples cloud3");
 // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "samples cloud3");
//  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloud3");
//viewer->addCoordinateSystem (0.1);
//while (!viewer->wasStopped ())
 // {
 //   viewer->spinOnce (100);
 //   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 // }
 }
