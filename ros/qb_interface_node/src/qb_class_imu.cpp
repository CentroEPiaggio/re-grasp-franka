
#include "qb_class_imu.h"

#include <vector>   
#include "std_msgs/Float64MultiArray.h"

qb_class_imu::qb_class_imu(){

	// Variables to get param 
	vector<int> ID_imuboard;

	string aux;

	// Initialize ROS Node
	node_ = new ros::NodeHandle("qb_interface_node_imu_");

	// Get param from roslaunch or yaml file
	node_->searchParam("/IDimuboards", aux);
	node_->getParam(aux, ID_imuboard);
	node_->param<double>("/step_time_imu", step_time_imu_, 0.002);
	node_->param<int>("/hand_step_div", hand_step_div_, 10);
	node_->param<bool>("/compute_angles", compute_angles_, true);


    qbImuBoard* tmp_imuboard;

    for (int i = ID_imuboard.size(); i--;) {

        tmp_imuboard = new qbImuBoard(qb_comm_, ID_imuboard[i]);
        
       	// IF an error is find
        if (tmp_imuboard == NULL){
        	cout << "[ERROR] Unable to allocate space for imu board structure." << endl;
            return;
        }

        imuboard_chain_.push_back(tmp_imuboard);
    } 


	// Initialize publisher and subscriber

	if (!imuboard_chain_.empty()){

		// Publisher initialize

		//IREPAOLO
        imuboard_pub_acc_vett  = node_->advertise<std_msgs::Float64MultiArray>("/qb_class_imu/acc_vett", 1);
        imuboard_pub_gyro_vett  = node_->advertise<std_msgs::Float64MultiArray>("/qb_class_imu/gyro_vett", 1);

		imuboard_pub_acc_  = node_->advertise<qb_interface::inertialSensorArray>("/qb_class_imu/acc", 1);
		imuboard_pub_gyro_ = node_->advertise<qb_interface::inertialSensorArray>("/qb_class_imu/gyro", 1);

		Acc_.resize(17,3);		//allocate max size instead of (imuboard_chain.[0]->n_imu_)
		Acc_old_.resize(17,3);
		Gyro_.resize(17,3);

		Acc_.setZero();
		Acc_old_.setZero();
		Gyro_.setZero();

	}


}

qb_class_imu::~qb_class_imu(){

}

bool qb_class_imu::readIMU(){
	qb_interface::inertialSensor tmp_acc, tmp_gyro, tmp_mag;
	qb_interface::inertialSensorArray acc, gyro;
	qb_interface::temperature tmp_temp;
	qb_interface::temperatureArray temp;
	//IREPAOLO
	std::vector<double> acc_vett, gyro_vett;

	// std::cout << "# board " << imuboard_chain_.size() << std::endl;	

	for (int k = imuboard_chain_.size(); k--;){
	    
		if (imuboard_chain_[k]->getImuReadings() < 0)
			return false;
		
		 for (int i = 0; i < imuboard_chain_[k]->n_imu_; i++) 
		 {
			
			 // printf("IMU: %d\n", imuboard_chain_[k]->ids_[i]);
		
			if (imuboard_chain_[k]->imu_table_[5*i + 0])
			{
				tmp_acc.board_id = imuboard_chain_[k]->getID();
				tmp_acc.id = imuboard_chain_[k]->ids_[i];
				tmp_acc.x  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i];
				tmp_acc.y  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+1];
				tmp_acc.z  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+2];
				acc.m.push_back(tmp_acc);

				Acc_(i,0) = tmp_acc.x; 
				Acc_(i,1) = tmp_acc.y; 
				Acc_(i,2) = tmp_acc.z; 


                //IREPAOLO
				acc_vett.push_back(tmp_acc.x); 
				acc_vett.push_back(tmp_acc.y);
				acc_vett.push_back(tmp_acc.z);
				

			}
			
			if (imuboard_chain_[k]->imu_table_[5*i + 1])
			{
				tmp_gyro.board_id = imuboard_chain_[k]->getID();
				tmp_gyro.id = imuboard_chain_[k]->ids_[i];
				tmp_gyro.x  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+3];
				tmp_gyro.y  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+4];
				tmp_gyro.z  = imuboard_chain_[k]->imu_values_[(3*3+4+1)*i+5];
				gyro.m.push_back(tmp_gyro);

				Gyro_(i,0) = tmp_gyro.x; 
				Gyro_(i,1) = tmp_gyro.y; 
				Gyro_(i,2) = tmp_gyro.z; 
				
				//IREPAOLO
				gyro_vett.push_back(tmp_gyro.x); 
				gyro_vett.push_back(tmp_gyro.y);
				gyro_vett.push_back(tmp_gyro.z);

			}
			// verify if this usleep is needed
			//usleep(0.5);
		}
	
	}
	
	if ((Acc_old_ - Acc_).sum() != 0)
 	{
		std_msgs::Float64MultiArray vett_a, vett_g;
        vett_a.data=acc_vett;
		vett_g.data=gyro_vett;
		imuboard_pub_acc_.publish(acc);
		imuboard_pub_acc_vett.publish(vett_a);
		imuboard_pub_gyro_.publish(gyro);	
		imuboard_pub_gyro_vett.publish(vett_g);		
 	}

 	Acc_old_ = Acc_;

}


