void HeightMap::constructGridClouds(const VPointCloudI::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count, size_t &box_count, size_t &copy_count)
{
  float min[100][100];
  float max[100][100];
  //float num_obs[grid_dim_][grid_dim_];
  //float num_clear[grid_dim_][grid_dim_];
  int num_obs[100][100];
  int num_clear[100][100];
  //int num_obs[grid_dim_][grid_dim_];
  //int num_clear[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  std::vector<ObjIdentifier> list;
  velodyne_height_map::Obstacle obstacle;
  //list.resize(1000);
  //memset(&init, 0, grid_dim_*grid_dim_);
  
  
  //Construct lines
  visualization_msgs::Marker lines;
  lines.header.frame_id = "velodyne";
  lines.header.stamp = ros::Time::now();
  lines.type = visualization_msgs::Marker::LINE_LIST;
  //lines.action = visualization_msgs::Marker::DELETE;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.05;
  lines.scale.y = 0.05;
  lines.color.a = 1.0;
  lines.ns = "my lines";
  lines.id = 1;
  lines.lifetime = ros::Duration(0.01); 
  lines.points.clear();


  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      init[x][y]=false;
      num_obs[x][y]=0;
      num_clear[x][y]=0;
    }
  }

  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan->points[i].z;
        max[x][y] = scan->points[i].z;
        num_obs[x][y] = 0;
        num_clear[x][y] = 0;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan->points[i].z);
        max[x][y] = MAX(max[x][y], scan->points[i].z);
      }
    }
  }





// ignoring the objects above the car
 for (unsigned i = 0; i < npoints; ++i)
 {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y])
    {
        if (min[x][y]>=car_height_)
        {
            max[x][y]=min[x][y];
            //ROS_INFO_STREAM("min[x][y] is " << min[x][y] <<"\n");
        }
     }
  }





  // calculate number of obstacles in each cell
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {  
        num_obs[x][y]++;
      } else {
        num_clear[x][y]++;
      }
    }
  }


  //ROS_INFO_STREAM("here" << "\n");
  ClassifyClusters(num_obs,&list, min, max);
  IdentifyClusters(&list);
  
  

  for (int i=0;i<list.size();i++)
  {
	ROS_INFO_STREAM(list[i].type);
  }
  ROS_INFO_STREAM("\n");
  // create clouds from grid
  double grid_offset=grid_dim_/2.0*m_per_cell_;
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      /*if (num_obs[x][y]>0) {

        obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) 255.0;
        obs_count++;
      }*/
      if (num_clear[x][y]>0) {
        clear_cloud_.points[empty_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].z = height_diff_threshold_;
        //clear_cloud_.channels[0].values[empty_count] = (float) 255.0;
        empty_count++;
      }
    }
  }

  //code for planar segmentation
/*
//  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
//  pcl::copyPointCloud(scan, cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);	//output point cloud
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::SACSegmentation<VPointI> seg;

  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setInputCloud (scan);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)

  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  cloud->header.stamp = scan->header.stamp;
  cloud->header.frame_id = scan->header.frame_id;
  cloud->points.resize(npoints);

  cout << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
  {
   cloud->points[i].x=scan->points[inliers->indices[i]].x;
   cloud->points[i].y=scan->points[inliers->indices[i]].y;
   cloud->points[i].z=scan->points[inliers->indices[i]].z;
  }

  cloud->points.resize(inliers->indices.size());			//use cloud to show planes


  cout<<"=======MODEL COEFFICIENTS========="<<endl;
  cout<<"a=="<<coefficients->values[0]<<endl;
  cout<<"b=="<<coefficients->values[1]<<endl;
  cout<<"c=="<<coefficients->values[2]<<endl;
*/



geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;
  for (int i=0;i<list.size();i++)
  {
	copy_count=0;
	obstacle_copy.points.clear();
	obstacle_copy.points.resize(npoints);
	for (int j=list[i].minrow;j<=list[i].maxrow;j++)
	{
		for (int k=list[i].mincol;k<=list[i].maxcol;k++)
		{
			if (num_obs[j][k]>0){
			    obstacle_cloud_.points[obs_count].x = -grid_offset + (j*m_per_cell_+m_per_cell_/2.0);
        		obstacle_cloud_.points[obs_count].y = -grid_offset + (k*m_per_cell_+m_per_cell_/2.0);
        		obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
			
        		if (list[i].type==1) 
			{	obstacle_cloud_.points[obs_count].r=255;   	// white -car
				obstacle_cloud_.points[obs_count].g=255;
				obstacle_cloud_.points[obs_count].b=255;
				lines.color.r = 1.0f ;
				lines.color.g = 1.0f;
				lines.color.b = 1.0f ;


			}
			else if (list[i].type==2) 
			{	obstacle_cloud_.points[obs_count].r=255;	// red  -truck
				obstacle_cloud_.points[obs_count].g=255;
				obstacle_cloud_.points[obs_count].b=255;
				lines.color.r = 1.0f ;
				lines.color.g = 1.0f;
				lines.color.b = 1.0f ;

			}
			else if (list[i].type==3) 
			{	obstacle_cloud_.points[obs_count].r=255;		//blue -rickshaw
				obstacle_cloud_.points[obs_count].g=255;
				obstacle_cloud_.points[obs_count].b=255;
				lines.color.r = 1.0f ;
				lines.color.g = 1.0f;
				lines.color.b = 1.0f ;
			}
			else if (list[i].type==4) 
			{	obstacle_cloud_.points[obs_count].r=255;		//green - bike
				obstacle_cloud_.points[obs_count].g=255;
				obstacle_cloud_.points[obs_count].b=255;
				lines.color.r = 1.0f ;
				lines.color.g = 1.0f;
				lines.color.b = 1.0f ;

			}
			else if (list[i].type==5) 
			{	obstacle_cloud_.points[obs_count].r=255;	//yellow - tree,wall etc
				obstacle_cloud_.points[obs_count].g=255;
				obstacle_cloud_.points[obs_count].b=255;
				lines.color.r = 1.0f ;
				lines.color.g = 1.0f;
				lines.color.b = 1.0f ;

			}
			else if (list[i].type==6) 
			{	obstacle_cloud_.points[obs_count].r=255;		//cyan - people
				obstacle_cloud_.points[obs_count].g=255;
				obstacle_cloud_.points[obs_count].b=255;
				lines.color.r = 1.0f ;
				lines.color.g = 1.0f;
				lines.color.b = 1.0f ;

			}
			else if (list[i].type==7) 
			{	obstacle_cloud_.points[obs_count].r=255;	//pink - cant decipher
				obstacle_cloud_.points[obs_count].g=255;
				obstacle_cloud_.points[obs_count].b=255;
				lines.color.r = 1.0f ;
				lines.color.g = 1.0f;
				lines.color.b = 1.0f ;
			}
			


       			obstacle_copy.points[copy_count].x=obstacle_cloud_.points[obs_count].x;
       			obstacle_copy.points[copy_count].y=obstacle_cloud_.points[obs_count].y;
       			obstacle_copy.points[copy_count].z=obstacle_cloud_.points[obs_count].z;

       			obs_count++;
       			copy_count++;
			}
		}
	}

	obstacle_copy.points.resize(copy_count);

	Eigen::Vector4f centroid;
	Eigen::Matrix3f evecs;
	Eigen::Vector3f evals;
	Eigen::Matrix3f cov_matrix;

	pcl::compute3DCentroid(obstacle_copy,centroid);
    pcl::computeCovarianceMatrix(obstacle_copy, centroid, cov_matrix);
	pcl::eigen33(cov_matrix, evecs, evals);

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr copy_ptr (new pcl::PointCloud<pcl::PointXYZRGBL>);
	*copy_ptr=obstacle_copy;

//	cout<<"[("<<evals[0]<<"  ,  "<<evals[1]<<"  ,  "<<evals[2]<<")]"<<endl;
//	cout<<"[("<<evecs(0,0)<<","<<evecs(0,1)<<","<<evecs(0,2)<<")]"<<endl;
//	cout<<"|("<<evecs(1,0)<<","<<evecs(1,1)<<","<<evecs(1,2)<<")|"<<endl;
//	cout<<"[("<<evecs(2,0)<<","<<evecs(2,1)<<","<<evecs(2,2)<<")]"<<endl;
//	cout<<"=+=+=+=+=+="<<endl;

	//Code for the bounding box here
	//if(list[i].type!=5 && list[i].type!=7){
	float x1,x2,y1,y2,z1,z2;
	x1= -grid_offset + (list[i].minrow*m_per_cell_+m_per_cell_/2.0);
	x2= -grid_offset + (list[i].maxrow*m_per_cell_+m_per_cell_/2.0);
	y1= -grid_offset + (list[i].mincol*m_per_cell_+m_per_cell_/2.0);
	y2= -grid_offset + (list[i].maxcol*m_per_cell_+m_per_cell_/2.0);
	z1= list[i].minz;
	z2= list[i].maxz;
	


	//Publishes the diagonal points for the 3D bounding box
	if(x1!=x2 && y1!=y2){
	obstacle.min_x = x1;
	obstacle.max_x = x2;
	obstacle.min_y = y1;
	obstacle.max_y = y2;
	obstacle.min_z = z1;
	obstacle.max_z = z2;
	obstacle.obstacle_class = list[i].type;
	obstacle.obstacleID = i;
	obstacles_array.obstacles.push_back(obstacle);
	

	//This code is for visualization of bounding boxes on rviz
	//rviz does not support wireframe cube so 12 lines are needed to be plotted for each of box
	//Note that this slows down rviz if the number of boxes is high

	p1.x = x1; p1.y = y1; p1.z = z1;
	p2.x = x2; p2.y = y1; p2.z = z1;
	p3.x = x1; p3.y = y1; p3.z = z2;
	p4.x = x2; p4.y = y1; p4.z = z2;
	p5.x = x1; p5.y = y2; p5.z = z1;
	p6.x = x2; p6.y = y2; p6.z = z1;
	p7.x = x1; p7.y = y2; p7.z = z2;
	p8.x = x2; p8.y = y2; p8.z = z2;

		//ROTATE THE BOX BY AN ANGLE OF THETA1

		double theta1 = atan2 (evecs(1,2),evecs(0,2));
		double theta2 = atan2 (evecs(1,1),evecs(0,1));

        //cout<<"Product of slopes:  "<<(tan(theta1)*tan(theta2))<<endl;

		double center_x,center_y,del_x,del_y;
		center_x=(x1+x2)/2;  center_y=(y1+y2)/2;
		double alpha = atan2 (x2-center_x,center_y-y1);
		double l=sqrt((x2-center_x)*(x2-center_x)+(y1-center_y)*(y1-center_y));

		p2.x=l*sin(theta1+alpha) + center_x;
		p2.y=center_y - l*cos(theta1+alpha);

		p5.x=center_x - l*sin(theta1+alpha);
		p5.y=l*cos(theta1+alpha) + center_y;

		p1.x=center_x - l*sin(alpha-theta1);
		p1.y=center_y - l*cos(alpha-theta1);

		p6.x=l*sin(alpha-theta1) + center_x;
		p6.y=l*cos(alpha-theta1) + center_y;

		float slope1=tan(theta1);
		float slope2=tan(theta2);
		Eigen::Vector3f line1(-slope1,1,(slope1*centroid[0]-centroid[1]));				//Ax+By+C=0
		Eigen::Vector3f line2(-slope1,1,(slope1*centroid[0]-centroid[1]));
		Eigen::Vector3f line3(-slope2,1,(slope2*centroid[0]-centroid[1]));
		Eigen::Vector3f line4(-slope2,1,(slope2*centroid[0]-centroid[1]));

		float normalLength=hypot(line1[0],line1[1]);
		float farthest_pt_rt1=0,farthest_pt_lt1=0;
		for(int k=0;k<obstacle_copy.points.size();++k)
		{
		float distance=((line1[0]*obstacle_copy.points[k].x) + (line1[1]*obstacle_copy.points[k].y) + line1[2])/normalLength;
		if(distance>0)
			farthest_pt_rt1=MAX(abs(distance),farthest_pt_rt1);
		if(distance<0)
			farthest_pt_lt1=MAX(abs(distance),farthest_pt_lt1);
		}

		line1[2]=line1[2]+(farthest_pt_lt1*normalLength);
		line2[2]=line2[2]-(farthest_pt_rt1*normalLength);
        //cout<<"dist between line 1 and line 2:  "<<abs((line2[2]-line1[2])/normalLength)<<endl;

		normalLength=hypot(line3[0],line3[1]);
		float farthest_pt_rt2=0,farthest_pt_lt2=0;
		for(int k=0;k<obstacle_copy.points.size();++k)
		{
		float distance=((line3[0]*obstacle_copy.points[k].x) + (line3[1]*obstacle_copy.points[k].y) + line3[2])/normalLength;
		if(distance>0)
			farthest_pt_rt2=MAX(abs(distance),farthest_pt_rt2);
		if(distance<0)
			farthest_pt_lt2=MAX(abs(distance),farthest_pt_lt2);
		}

		line3[2]=line3[2]+(farthest_pt_lt2*normalLength);
		line4[2]=line4[2]-(farthest_pt_rt2*normalLength);
        //cout<<"dist between line 3 and line 4;  "<<abs((line3[2]-line4[2])/normalLength)<<endl;

		p1.x=(line1[1]*line3[2] - line1[2]*line3[1])/(line1[0]*line3[1] - line3[0]*line1[1]);	//pt of intersect of line 1 and line 3
		p1.y=(line1[2]*line3[0] - line3[2]*line1[0])/(line1[0]*line3[1] - line3[0]*line1[1]);

		p2.x=(line2[1]*line3[2] - line2[2]*line3[1])/(line2[0]*line3[1] - line3[0]*line2[1]);	//pt of intersect of line 2 and line 3
		p2.y=(line2[2]*line3[0] - line3[2]*line2[0])/(line2[0]*line3[1] - line3[0]*line2[1]);

		p5.x=(line1[1]*line4[2] - line1[2]*line4[1])/(line1[0]*line4[1] - line4[0]*line1[1]);	//pt of intersect of line 1 and line 4
		p5.y=(line1[2]*line4[0] - line4[2]*line1[0])/(line1[0]*line4[1] - line4[0]*line1[1]);

		p6.x=(line2[1]*line4[2] - line2[2]*line4[1])/(line2[0]*line4[1] - line4[0]*line2[1]);	//pt of intersect of line 2 and line 4
		p6.y=(line2[2]*line4[0] - line4[2]*line2[0])/(line2[0]*line4[1] - line4[0]*line2[1]);

//		p6.x= -line1[0]*((line1[0]*p6.x + line1[1]*p6.y + line1[2])/(normalLength*normalLength)) + p6.x;
//		p6.y= -line1[1]*((line1[0]*p6.x + line1[1]*p6.y + line1[2])/(normalLength*normalLength)) + p6.y;
//
//		p5.x= -line1[0]*((line1[0]*p5.x + line1[1]*p5.y + line1[2])/(normalLength*normalLength)) + p5.x;
//		p5.y= -line1[1]*((line1[0]*p5.x + line1[1]*p5.y + line1[2])/(normalLength*normalLength)) + p5.y;
//
//
//		p1.x= -line[0]*((line[0]*p1.x + line[1]*p1.y + line[2])/(normalLength*normalLength)) + p1.x;
//		p1.y= -line[1]*((line[0]*p1.x + line[1]*p1.y + line[2])/(normalLength*normalLength)) + p1.y;
//
//		p2.x= -line[0]*((line[0]*p2.x + line[1]*p2.y + line[2])/(normalLength*normalLength)) + p2.x;
//		p2.y= -line[1]*((line[0]*p2.x + line[1]*p2.y + line[2])/(normalLength*normalLength)) + p2.y;

		p3.x=p1.x; p3.y=p1.y;
		p4.x=p2.x; p4.y=p2.y;
		p7.x=p5.x; p7.y=p5.y;
		p8.x=p6.x; p8.y=p6.y;


		//code for publishing box_cloud
		if((x1>=0)&&(y1>=-8)&&(y2<=8))
		{
	        box_cloud.points[box_count].x = p1.x;
	        box_cloud.points[box_count].y = p1.y;
	        box_cloud.points[box_count].z = p1.z;
	        box_count++;
	        box_cloud.points[box_count].x = p8.x;
	        box_cloud.points[box_count].y = p8.y;
	        box_cloud.points[box_count].z = p8.z;
	        box_count++;
		}

//		p9.x=centroid[0]; p9.y=centroid[1]; p9.z=centroid[2];
//		lines.points.push_back(p9);
//		p10.x=p9.x + 5*evecs(0,2); p10.y=p9.y + 5*evecs(1,2); p10.z=p9.z + 5*evecs(2,2);
//		lines.points.push_back(p10);

	lines.points.push_back(p1); lines.points.push_back(p2);
	lines.points.push_back(p1); lines.points.push_back(p3);
	lines.points.push_back(p2); lines.points.push_back(p4);
	lines.points.push_back(p3); lines.points.push_back(p4);
	lines.points.push_back(p5); lines.points.push_back(p6);
	lines.points.push_back(p5); lines.points.push_back(p7);
	lines.points.push_back(p6); lines.points.push_back(p8);
	lines.points.push_back(p7); lines.points.push_back(p8);
	lines.points.push_back(p1); lines.points.push_back(p5);
	lines.points.push_back(p2); lines.points.push_back(p6);
	lines.points.push_back(p3); lines.points.push_back(p7);
	lines.points.push_back(p4); lines.points.push_back(p8);
	line_list.markers.push_back(lines);
	}
  }
  obstacles_array.numberOfDetections = list.size();
  list.clear();
//  for(int m=0;m<obstacles_array.numberOfDetections;m++)
//  {
//		  cout<<"min_x="<<obstacles_array.obstacles[m].min_x;
//		  cout<<"   max_x="<<obstacles_array.obstacles[m].max_x;
//		  cout<<"   min_y="<<obstacles_array.obstacles[m].min_y;
//		  cout<<"   max_y="<<obstacles_array.obstacles[m].max_y;
//		  cout<<"   min_z="<<obstacles_array.obstacles[m].min_z;
//		  cout<<"   max_z="<<obstacles_array.obstacles[m].max_z;
//		  cout<<"   obstacleID="<<obstacles_array.obstacles[m].obstacleID<<endl;
//	  }
}
