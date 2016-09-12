#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/registration/ia_kfpcs.h>

#include "include/kfpcs_ia_data.h"
#include "include/short_filter.h"
#include "include/sift_keypoint_large.h"
#include<stdlib.h>
#include<iostream>
//#include <string>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;
using namespace std;
typedef struct{
	int ID;
	string path;
}cloudItem;
typedef struct{
	int ID_src;
	int ID_tgt;
	double *ground_truth;
}pairItem;
void getFyOmigaKaFromRotattionMatrix(double &fy,double &omiga,double &ka,double *Rotation);
void RotationMatrixCalculate(double fy,double omiga,double ka,double *&Rotation);
int register_by_4PCS(const char *source_file,const char *target_file,double voxel_size,double min_contrast,double approx_overlap,double abort_score,double *&paras_coarse,int &dur);
void conversionRad(double &rad)
{
	const double Pi=3.141592653;
	const double Pi2=2*3.141592653;
	int n=int(rad/Pi2);
	rad-=(n*Pi2);
	if(rad>Pi)
		rad-=Pi2;
	else if(rad<-Pi)
		rad+=Pi2;
}
int main (int argc, char *argv[])
{
    argv[1]="..//input//list.txt";
    argv[2]="..//input//groundtruth.txt";
    printf("init.\n");
    printf("\t1.1.load list file.\n");
	char *project_file=argv[1];
	char *parasmeter_file=argv[2];
	vector<cloudItem>Clouds;
	vector<pairItem>pairs;

    printf("\t1.2.load groundtruth data file.\n");
	FILE *fp_=fopen(parasmeter_file,"r");
	FILE *fp=fopen(project_file,"r");
	char tmp[400];int n_cloud;
	fscanf(fp,"%s %d",tmp,&n_cloud);
	for(int i=0;i<n_cloud;i++)
	{
		cloudItem cloud;
		fscanf(fp,"%d %s",&cloud.ID,tmp);
		cloud.path=tmp;
		Clouds.push_back(cloud);
	}
	int n_pair;
	fscanf(fp,"%s %d",tmp,&n_pair);
	for(int i=0;i<n_pair;i++)
	{
		pairItem pair;
        fscanf(fp,"%d %d",&pair.ID_src,&pair.ID_tgt);
		pairs.push_back(pair);
		double data_tmp[6];int ID_src,ID_tgt;
		fscanf(fp_,"%d %d %lf %lf %lf %lf %lf %lf\n",&ID_src,&ID_tgt,&data_tmp[0],&data_tmp[1],&data_tmp[2],&data_tmp[3],&data_tmp[4],&data_tmp[5]);
		pairs[i].ground_truth=new double[6];
		for(int j=0;j<6;j++)
		{
			pairs[i].ground_truth[j]=data_tmp[j];
		}
	}
	fclose(fp);
	fclose(fp_);
    printf("2.main loop.\n");
	FILE *fp_log=fopen("log.txt","w");
	for(int i=1;i<n_pair;i++)
	{
        printf("\t2.1 registration by k4pcs.\n");
		int idx_src,idx_tgt;
		for(int j=0;j<n_cloud;j++)
		{
			if(pairs[i].ID_src==Clouds[j].ID)
				idx_src=j;
			if(pairs[i].ID_tgt==Clouds[j].ID)
				idx_tgt=j;
		}
		double *paras_coarse; int dur;
		register_by_4PCS(Clouds[idx_src].path.data(),Clouds[idx_tgt].path.data(),voxel_size,min_contrast,approx_overlap,abort_score,paras_coarse,dur);

		double MAE,MTE;
		MAE=0;MTE=0;
		MAE=fabs(paras_coarse[0]*180/3.141592653-pairs[i].ground_truth[0])+fabs(paras_coarse[1]*180/3.141592653-pairs[i].ground_truth[1])+fabs(paras_coarse[2]*180/3.141592653-pairs[i].ground_truth[2]);
		MTE=fabs(paras_coarse[3]-pairs[i].ground_truth[3])+fabs(paras_coarse[4]-pairs[i].ground_truth[4])+fabs(paras_coarse[5]-pairs[i].ground_truth[5]);
		fprintf(fp_log,"%d %d %d %lf %lf\n",pairs[i].ID_src,pairs[i].ID_tgt,dur/1000,MAE,MTE);
        printf("\t2.1 result judgement.\n");
		PCL_INFO("%d %d dur %ds MAE %lf MTE %lf %d//%d\n",pairs[i].ID_src,pairs[i].ID_tgt,dur/1000,MAE,MTE,i,n_pair);
		
	}
	fclose(fp_log);
}
int register_by_4PCS(const char *source_file,const char *target_file,double voxel_size,double min_contrast,double approx_overlap,double abort_score,double *&paras_coarse,int &dur)
{
	//source_file = argv[1];
	//target_file = argv[2];

	//voxel_size = atof(argv[3]);// optimally set to estimated scan resolution in overlap
	//min_contrast = atof(argv[4]); // set to extract approximately 1000-5000 keypoints
	//approx_overlap = atof(argv[5]); // rough estimation of scan overlap
	//abort_score = atof(argv[6]); // zero to avoid early termination
	clock_t t_overall_start,t_overall_end;
	t_overall_start=clock();
  cout << endl;
  clock_t t_start,t_end,t_dur;
  // load raw point clouds
  cout << "Loading point clouds..";
  t_start=clock();
  PointCloud <PointXYZI>::Ptr cloud_source (new PointCloud <PointXYZI>), cloud_target (new PointCloud <PointXYZI>);
  pcl::io::loadPCDFile(source_file, *cloud_source);
  pcl::io::loadPCDFile (target_file, *cloud_target);
  
  cout << "ok!" << endl;
  t_end=clock();
  PCL_INFO("load data dur%ds %d %d\n",(t_end-t_start)/CLOCKS_PER_SEC,cloud_source->width,cloud_target->width);


  // first filter step: voxel grid 
  cout << "Apply voxel grid filter..";
  t_start=clock();
  PointCloud <PointXYZI>::Ptr voxel_cloud_source (new PointCloud <PointXYZI>);
  PointCloud <PointXYZI>::Ptr voxel_cloud_target (new PointCloud <PointXYZI>);
  //sampleLeafsized <PointXYZI> (cloud_source, *voxel_cloud_source, voxel_size);
  //sampleLeafsized <PointXYZI> (cloud_target, *voxel_cloud_target, voxel_size);
  const float voxel_size_=voxel_size;
  sampleLeafsized  (cloud_source, *voxel_cloud_source,voxel_size_);
  sampleLeafsized  (cloud_target, *voxel_cloud_target,voxel_size_);
  cout << "ok!" << endl;
  t_end=clock();
  PCL_INFO("voxel grid filter dur%ds %d %d\n",(t_end-t_start)/CLOCKS_PER_SEC,voxel_cloud_source->width,voxel_cloud_target->width);


  // second filter step: keypoint detector
  cout << "Extract DoG keypoints..";
  t_start=clock();
  PointCloud <PointXYZI>::Ptr keypoint_cloud_source (new PointCloud <PointXYZI>);
  PointCloud <PointXYZI>::Ptr keypoint_cloud_target (new PointCloud <PointXYZI>);

  SIFTKeypoint <PointXYZI, PointXYZI> dog_extract;
  dog_extract.setScales (voxel_size, 5, 5);
  dog_extract.setMinimumContrast (min_contrast); 

  dog_extract.setInputCloud (voxel_cloud_source);  
  dog_extract.compute (*keypoint_cloud_source);

  dog_extract.setInputCloud (voxel_cloud_target);
  dog_extract.compute (*keypoint_cloud_target);
  pcl::io::savePCDFileBinary("key_src.pcd",*keypoint_cloud_source);
  pcl::io::savePCDFileBinary("key_tgt.pcd",*keypoint_cloud_target);
  cout << "ok!" << endl;
  t_end=clock();
  PCL_INFO("sift key-point detection dur%ds %d %d\n",(t_end-t_start)/CLOCKS_PER_SEC,
           keypoint_cloud_source->width,keypoint_cloud_target->width);

  // matching keypoint clouds
  cout << "Matching source and target keypoints..";
  t_start=clock();
  PointCloud <PointXYZI> keypoint_cloud_source_aligned;

  KFPCSInitialAlignment <PointXYZI, PointXYZI> kfpcs_ia;
  kfpcs_ia.setInputSource (keypoint_cloud_source);
  kfpcs_ia.setInputTarget (keypoint_cloud_target);
  kfpcs_ia.setApproxOverlap (approx_overlap);
  kfpcs_ia.setScoreThreshold (abort_score);
  kfpcs_ia.setDelta (voxel_size, false);
  kfpcs_ia.setNumberOfSamples(5000);
  kfpcs_ia.align (keypoint_cloud_source_aligned);
  cout << "ok!" << endl;
  t_end=clock();
  PCL_INFO("4PCS dur%ds\n",(t_end-t_start)/CLOCKS_PER_SEC);
  

  // display resulting transformation matrix
  Eigen::Matrix4f groundtruth;
  groundtruth << -0.4940855669, 0.8694132493, -0.0002334294, -3.8336765762,
    -0.8694023095, -0.4940806630, -0.0048910886, 2.3153953480,
    -0.0043677102, -0.0022136723, 0.9999880113, 0.6687818594,
    0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000;

  //cout << "\nGroundtruth transformation matrix\n" << groundtruth << endl;
  cout << "\nResulting coarse transformation matrix\n" << kfpcs_ia.getFinalTransformation () << endl;
  t_overall_end=clock();
  dur=t_overall_end-t_overall_start;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src_t (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_source,*cloud_src_t,kfpcs_ia.getFinalTransformation () );
  pcl::io::savePCDFileBinary("cloud_src_t.pcd",*cloud_src_t);
  pcl::io::savePCDFileBinary("cloud_src.pcd",*cloud_source);
  pcl::io::savePCDFileBinary("cloud_tgt.pcd",*cloud_target);
  //<<<<<<<<<<<<<<<<<<------------------------------------------------------------------------------------------------------------------转换参数分解--------------------->>>>>>>>>>>>>>>>>
  //旋转矩阵分解
  Eigen::Matrix4f transform=kfpcs_ia.getFinalTransformation ();
  double rotationMat[9];
  rotationMat[0]=transform(0,0);	rotationMat[1]=transform(0,1);	rotationMat[2]=transform(0,2);
  rotationMat[3]=transform(1,0);	rotationMat[4]=transform(1,1);	rotationMat[5]=transform(2,1);
  rotationMat[6]=transform(2,0);	rotationMat[7]=transform(1,2);	rotationMat[8]=transform(2,2);
  getFyOmigaKaFromRotattionMatrix(paras_coarse[0],paras_coarse[1],paras_coarse[2],rotationMat);
  //平移矩阵
  paras_coarse[3]=kfpcs_ia.getFinalTransformation ()(0,3);
  paras_coarse[4]=kfpcs_ia.getFinalTransformation ()(1,3);
  paras_coarse[5]=kfpcs_ia.getFinalTransformation ()(2,3);

  double check=0;
  double *rotationMat_=new double[9];
  RotationMatrixCalculate(paras_coarse[0],paras_coarse[1],paras_coarse[2],rotationMat_);
  for(int i=0;i<9;i++)
  {
	  check+=fabs(rotationMat_[i]-rotationMat[i]);
  }
  check+=fabs( paras_coarse[3]-transform(0,3));
  check+=fabs( paras_coarse[4]-transform(1,3));
  check+=fabs( paras_coarse[5]-transform(2,3));
  if(check>0.3)
	  printf("转换参数分解错误！！！！\n");
  return (0);
}
void RotationMatrixCalculate(double fy,double omiga,double ka,double *&Rotation)
{
	Rotation=new double[9];
	Rotation[0]=cos(fy)*cos(ka)-sin(fy)*sin(omiga)*sin(ka);
	Rotation[1]=-cos(fy)*sin(ka)-sin(fy)*sin(omiga)*cos(ka);
	Rotation[2]=-sin(fy)*cos(omiga);
	Rotation[3]=cos(omiga)*sin(ka);
	Rotation[4]=cos(omiga)*cos(ka);
	Rotation[5]=-sin(omiga);
	Rotation[6]=sin(fy)*cos(ka)+cos(fy)*sin(omiga)*sin(ka);
	Rotation[7]=-sin(fy)*sin(ka)+cos(fy)*sin(omiga)*cos(ka);
	Rotation[8]=cos(fy)*cos(omiga);
}
void getFyOmigaKaFromRotattionMatrix(double &fy,double &omiga,double &ka,double *Rotation)
{
	Eigen::MatrixXd X(3,1);Eigen::MatrixXd B(9,3); Eigen::MatrixXd L(9,1);
	X(1,0)=0;
	X(0,0)=0;
	X(2,0)=0;
	for(int i=0;i<50;i++)
	{
		fy=X(0,0);	omiga=X(1,0);	ka=X(2,0);
		B(0,0)=- cos(ka)*sin(fy) - cos(fy)*sin(ka)*sin(omiga);	B(0,1)=-cos(omiga)*sin(fy)*sin(ka);	B(0,2)=- cos(fy)*sin(ka) - cos(ka)*sin(fy)*sin(omiga);
		B(1,0)=sin(fy)*sin(ka) - cos(fy)*cos(ka)*sin(omiga);	B(1,1)=-cos(ka)*cos(omiga)*sin(fy);	B(1,2)=sin(fy)*sin(ka)*sin(omiga) - cos(fy)*cos(ka);
		B(2,0)=-cos(fy)*cos(omiga);	B(2,1)=sin(fy)*sin(omiga);	B(2,2)=0;
		B(3,0)=0;	B(3,1)=-sin(ka)*sin(omiga);	B(3,2)=cos(ka)*cos(omiga);
		B(4,0)=0;	B(4,1)=-cos(ka)*sin(omiga);	B(4,2)=-cos(omiga)*sin(ka);
		B(5,0)=0;	B(5,1)=-cos(omiga);	B(5,2)=0;
		B(6,0)=cos(fy)*cos(ka) - sin(fy)*sin(ka)*sin(omiga);	B(6,1)=cos(fy)*cos(omiga)*sin(ka);	B(6,2)=cos(fy)*cos(ka)*sin(omiga) - sin(fy)*sin(ka);
		B(7,0)=- cos(fy)*sin(ka) - cos(ka)*sin(fy)*sin(omiga);	B(7,1)=cos(fy)*cos(ka)*cos(omiga);	B(7,2)=- cos(ka)*sin(fy) - cos(fy)*sin(ka)*sin(omiga);
		B(8,0)=-cos(omiga)*sin(fy);	B(8,1)=-cos(fy)*sin(omiga);	B(8,2)=0;

		L(0,0)=Rotation[0]-(cos(fy)*cos(ka)-sin(fy)*sin(omiga)*sin(ka));
		L(1,0)=Rotation[1]-(-cos(fy)*sin(ka)-sin(fy)*sin(omiga)*cos(ka));
		L(2,0)=Rotation[2]-(-sin(fy)*cos(omiga));
		L(3,0)=Rotation[3]-(cos(omiga)*sin(ka));
		L(4,0)=Rotation[4]-(cos(omiga)*cos(ka));
		L(5,0)=Rotation[5]-(-sin(omiga));
		L(6,0)=Rotation[6]-(sin(fy)*cos(ka)+cos(fy)*sin(omiga)*sin(ka));
		L(7,0)=Rotation[7]-(-sin(fy)*sin(ka)+cos(fy)*sin(omiga)*cos(ka));
		L(8,0)=Rotation[8]-(cos(fy)*cos(omiga));

		Eigen::MatrixXd NN=(B.transpose()*B);
		Eigen::MatrixXd NN_=NN.inverse();
		Eigen::MatrixXd X_=NN_*B.transpose()*L;
		X+=X_;

		double error=fabs(L(0,0))+fabs(L(1,0))+fabs(L(2,0))+fabs(L(3,0))+fabs(L(4,0))+fabs(L(5,0))+fabs(L(6,0))+fabs(L(7,0))+fabs(L(8,0));
		//std::cout<<error<<std::endl;
	}

	conversionRad(fy);
	conversionRad(omiga);
	conversionRad(ka);
}
