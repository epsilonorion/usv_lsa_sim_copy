#ifndef FREEFLOATINGGAZEBOFLUID_H
#define FREEFLOATINGGAZEBOFLUID_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include "water_current/GetSpeed.h"
#include "wind_current/GetSpeed.h"
#include <thread>
#include <cmath>

namespace gazebo
{

    class model_st
    {
	public:
	model_st() { std::cerr<<"\n Init model_st()";};
        std::string name;
        physics::ModelPtr model_ptr;
        ros::Publisher state_publisher;
    };

    class link_st
    {
	public: link_st()
		{
			std::cerr<<"\n Init link_st()"; waterSurface.Set(0,0,0);
			usingLocalWaterVelocity = false;
			usingNoneWaterVelocity = true;
			usingLocalWindVelocity = false;
			usingNoneWindVelocity = true;
		};
        std::string model_name;
        physics::LinkPtr link;
        ignition::math::Vector3<double> buoyant_force;
        ignition::math::Vector3<double> buoyancy_center;
        ignition::math::Vector3<double> linear_damping;
        ignition::math::Vector3<double> angular_damping;

	ignition::math::Vector3<double> waterSurface;
	ros::Subscriber water_subscriber;
	ros::Subscriber waterCurrent_subscriber;

	ignition::math::Vector3<double> water_velocity_;
	bool usingLocalWaterVelocity;
	bool usingNoneWaterVelocity;
	bool usingLocalWindVelocity;
	bool usingNoneWindVelocity;
    ros::Subscriber waterVelocity_subscriber;
    ros::ServiceClient water_velocity_serviceClient_;
    ros::ServiceClient wind_velocity_serviceClient_;
	ignition::math::Vector3<double> wind_velocity_;
	double frontal_area;
	double lateral_area;
	double lateral_length;
    bool running;

        double limit;
        void initServiceClient(ros::NodeHandle* rosnode)
        {
        	std::cerr<<"\n initializing water service client";
        	water_velocity_serviceClient_ = rosnode->serviceClient<water_current::GetSpeed>("/waterCurrent");
        	std::cerr<<" ... done";
        	running = true;
        }

        void initWindServiceClient(ros::NodeHandle* rosnode)
		{

			std::cerr<<"\n initializing wind service client";
			wind_velocity_serviceClient_ = rosnode->serviceClient<wind_current::GetSpeed>("/windCurrent");
			std::cerr<<" ... done";
			running = true;
		}
        void stop() { running = false; }

        ~link_st(){
        	std::cerr<<"\n called link destructor!";
        	running = false;
			if(the_thread.joinable()) the_thread.join();
		}

        void ThreadLoop() {
        	ros::Rate r(10);
        	r.sleep();
				while(running)
				{

					water_current::GetSpeed srv;
					srv.request.x = waterSurface.X();
					srv.request.y = waterSurface.Y();
					//std::cerr<<"\n calling services 1! water: "<<usingLocalFluidVelocity;
					if (usingLocalWaterVelocity && water_velocity_serviceClient_.call(srv))
					{
						water_velocity_.X() = srv.response.x;
						water_velocity_.Y() = srv.response.y;
						//std::cerr << "\n ============== fluidWater "<<model_name<<"="<<link->GetName()<<" ("<<water_velocity_.X()<<", "<<water_velocity_.Y()<<")";
					}
					else if (usingLocalWaterVelocity)
					{
							ROS_WARN("FreefloatingPlugin::Failed to call service waterCurrent %s::%s", model_name.c_str(), link->GetName().c_str());


							ros::Rate s(1);
							s.sleep();
					}
					//std::cerr<<"\n calling services 2! wind: "<<usingLocalWindVelocity;
					wind_current::GetSpeed srv_wind;
					srv_wind.request.x = link->WorldPose().Pos().Z();
					srv_wind.request.y = link->WorldPose().Pos().Y();
					srv_wind.request.z = link->WorldPose().Pos().Z();
					if (usingLocalWindVelocity && wind_velocity_serviceClient_.call(srv_wind))
					{
						wind_velocity_.X() = srv_wind.response.x;
						wind_velocity_.Y() = srv_wind.response.y;
						wind_velocity_.Z() = srv_wind.response.z;
						//std::cerr << "\n ============== wind "<<model_name<<"="<<link->GetName()<<"pos: ("<<link->WorldPose().Pos().X()<<", "<<link->WorldPose().Pos().Y()<<", "<<link->WorldPose().Pos().Z()<<") wind: ("<<wind_velocity_.X()<<", "<<wind_velocity_.Y()<<", "<<wind_velocity_.Z()<<")";
						//std::cerr << "\n ============== wind "<<model_name<<"="<<link->GetName()<<" ("<<wind.x<<", "<<wind.y<<")";
					}
					else if (usingLocalWindVelocity)
					{
							ROS_WARN("FreefloatingPlugin::Failed to call service windCurrent %s::%s", model_name.c_str(), link->GetName().c_str());

							ros::Rate s(1);
							s.sleep();
					}
					//std::cerr<<"\n all services called!";
					r.sleep();
				}
            }

	void processSurfaceData(const geometry_msgs::Point::ConstPtr& pt)
	{
		//std::cerr<<"\n --------------------------------------- z: "<<pt->z;
		waterSurface.X() = pt->x;
		waterSurface.Y() = pt->y;
		waterSurface.Z() = pt->z;
	}

	void createSubscriberWaterSurface(ros::NodeHandle *nh, std::string topic)
	{
		water_subscriber = nh->subscribe(topic, 1, &link_st::processSurfaceData, this);
	}

	void processWaterCurrentData(const geometry_msgs::Vector3::ConstPtr& pt)
		{
			water_velocity_.X() = pt->x;
			water_velocity_.Y() = pt->y;
			water_velocity_.Z() = pt->z;
		}

	void createSubscriberWaterCurrent(ros::NodeHandle *nh, std::string topic)
	{
		waterCurrent_subscriber = nh->subscribe(topic, 1, &link_st::processWaterCurrentData, this);
	}

	void Start(){
		the_thread = std::thread(&link_st::ThreadLoop,this);
	    }
	private:
	    std::thread the_thread;
    };

class FreeFloatingFluidPlugin : public  WorldPlugin
{
public:
    FreeFloatingFluidPlugin() {}
    ~FreeFloatingFluidPlugin()
    {
	ROS_INFO("Closing FreeFloatingFluidPlugin");
        rosnode_->shutdown();
        delete rosnode_;
        for (int i = 0; i < buoyant_links_.size(); i++)
		{
			buoyant_links_[i]->stop();
			delete buoyant_links_[i];
		}


    }

    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    virtual void Update();

private:




    // parse a Vector3 string
    void ReadVector3(const std::string &_string, ignition::math::Vector3<double> &_vector);
    // parse a new model
    void ParseNewModel(const physics::ModelPtr &_model);
    // removes a deleted model
    void RemoveDeletedModel(std::vector<model_st*>::iterator &_model_it);
    // parse received fluid velocity message


    // parse received fluid velocity message
    void FluidVelocityCallBack(const geometry_msgs::Vector3ConstPtr& _msg);

    bool getSpeedCallback(water_current::GetSpeed::Request &req, water_current::GetSpeed::Request &res);
private:
    // plugin options
    bool has_surface_;
    ignition::math::Vector4<double> surface_plane_;
    std::string description_;

    // general data
    ros::NodeHandle* rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_event_;

    // links that are subject to fluid effects
    std::vector<link_st*> buoyant_links_;
    // models that have been parsed
    std::vector<model_st*> parsed_models_;

    // subscriber to fluid velocity (defined in the world frame)
    ros::Subscriber fluid_velocity_subscriber_;
    ignition::math::Vector3<double> fluid_velocity_;
    tf::TransformBroadcaster broadcaster;
    double param_update_rate;
    ros::Time prevUpdateTime;

};
GZ_REGISTER_WORLD_PLUGIN(FreeFloatingFluidPlugin)
}
#endif // FREEFLOATINGGAZEBOFLUID_H
