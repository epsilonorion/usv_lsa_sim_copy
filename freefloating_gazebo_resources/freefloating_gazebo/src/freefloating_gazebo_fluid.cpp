#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <tinyxml.h>
#include <urdf_parser/urdf_parser.h>
#include <gazebo/common/CommonTypes.hh>

#include <freefloating_gazebo/freefloating_gazebo_fluid.h>

using std::cout;
using std::endl;
using std::string;

namespace gazebo
{

	void
	FreeFloatingFluidPlugin::ReadVector3 (const std::string &_string, ignition::math::Vector3<double> &_vector)
	{
		std::stringstream ss (_string);
		double xyz[3];
		for (unsigned int i = 0; i < 3; ++i)
			ss >> xyz[i];
		_vector.Set (xyz[0], xyz[1], xyz[2]);
	}

	void
	FreeFloatingFluidPlugin::Load (physics::WorldPtr _world, sdf::ElementPtr _sdf)
	{

		//std::cerr << "\n ===== FreeFloatingFluidPlugin loading";
		ROS_INFO("############### Loading freefloating_fluid plugin");  
		this->world_ = _world;

		// register ROS node
		rosnode_ = new ros::NodeHandle ("gazebo");
		// initialize the prevUpdateTime
		this->prevUpdateTime = ros::Time::now ();


		// parse plugin options
		description_ = "robot_description";
		has_surface_ = false;
		surface_plane_.Set (0, 0, 1, 0); // default ocean surface plane is Z=0
		std::string fluid_topic = "current";

		if (_sdf->HasElement ("descriptionParam"))
			description_ = _sdf->Get < std::string > ("descriptionParam");
		if (_sdf->HasElement ("surface"))
		{
			has_surface_ = true;
			// get one surface point
			ignition::math::Vector3<double> surface_point;
			ReadVector3 (_sdf->Get < std::string > ("surface"), surface_point);
			// get gravity
			const ignition::math::Vector3<double> WORLD_GRAVITY = world_->Physics ()->World()->Gravity ().Normalize ();
			// water surface is orthogonal to gravity
			surface_plane_.Set (WORLD_GRAVITY.X(), WORLD_GRAVITY.Y(), WORLD_GRAVITY.Z(), WORLD_GRAVITY.Dot (surface_point));
			// push on parameter server
			rosnode_->setParam ("surface", surface_point.Z());
		}

		if (_sdf->HasElement ("fluidTopic"))
			fluid_topic = _sdf->Get < std::string > ("fluidTopic");


		if (!_sdf->HasElement("updateRate"))
		{
		    // if parameter tag does NOT exist
		    std::cout << "Missing parameter <updateRate> in FreeFloating_Gazebo Plugin, default to 0" << std::endl;
		    param_update_rate = 0;
		}
		    // if parameter tag exists, get its value
		else param_update_rate = _sdf->Get<double>("updateRate");

		// initialize subscriber to water current
		ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3> (
		    fluid_topic, 1, boost::bind (&FreeFloatingFluidPlugin::FluidVelocityCallBack, this, _1), ros::VoidPtr (),
		    &callback_queue_);
		fluid_velocity_.Set (0, 0, 0);
		fluid_velocity_subscriber_ = rosnode_->subscribe (ops);

		// Register plugin update
		update_event_ = event::Events::ConnectWorldUpdateBegin (boost::bind (&FreeFloatingFluidPlugin::Update, this));

		// Clear existing links
		buoyant_links_.clear ();
		parsed_models_.clear ();

		Update();
		ROS_INFO("############### Loaded freefloating_fluid plugin.");
	}

	void
	FreeFloatingFluidPlugin::Update ()
	{



		//std::cerr<<"\n FreeFloatingFluidPlugin::Update ()";
		// activate callbacks
		callback_queue_.callAvailable ();

		// look for new world models
		unsigned int i;
		std::vector<model_st*>::iterator model_it;
		bool found;
		for (i = 0; i < world_->ModelCount (); ++i)
		{
			found = false;
			for (model_it = parsed_models_.begin (); model_it != parsed_models_.end (); ++model_it)
			{
				if (world_->ModelByIndex (i)->GetName () == (*model_it)->name)
					found = true;
			}
			if (!found && !(world_->ModelByIndex (i)->IsStatic ())) // model not in listand not static, parse it for potential buoyancy flags
				ParseNewModel (world_->ModelByIndex (i));
		}

		// look for deleted world models
		model_it = parsed_models_.begin ();
		while (model_it != parsed_models_.end ())
		{
			found = false;
			for (i = 0; i < world_->ModelCount (); ++i)
			{
				if (world_->ModelByIndex (i)->GetName () == (*model_it)->name)
					found = true;
			}
			if (!found) // model name not in world anymore, remove the corresponding links
				RemoveDeletedModel (model_it);
			else
				++model_it;
		}

		// here buoy_links is up-to-date with the links that are subject to buoyancy, let's apply it
		ignition::math::Vector3<double> actual_force, cob_position, velocity_difference, torque;
		double signed_distance_to_surface;

		const ignition::math::Vector3<double> WORLD_GRAVITY = world_->Physics ()->World()->Gravity ().Normalize ();
		for (std::vector<link_st*>::iterator link_it = buoyant_links_.begin (); link_it != buoyant_links_.end (); ++link_it)
		{
			//std::cerr<<"\n world Force["<<(*link_it)->link->GetName()<<"]: "<<(*link_it)->link->GetWorldForce();
			// get world position of the center of buoyancy
			cob_position = (*link_it)->link->WorldPose ().Pos()
			    + (*link_it)->link->WorldPose ().Rot().RotateVector ((*link_it)->buoyancy_center);
			// start from the theoretical buoyancy force
			actual_force = (*link_it)->buoyant_force;

			if (has_surface_)
			{
				//surface_plane_.Set(0, 0, 1, (*link_it)->waterSurface.z);
				//surface_plane_.Set(0, 0, 1, 0);

				// water surface is orthogonal to gravity
				surface_plane_.Set (WORLD_GRAVITY.X(), WORLD_GRAVITY.Y(), WORLD_GRAVITY.Z(),
				                    WORLD_GRAVITY.Dot ((*link_it)->waterSurface));

				// adjust force depending on distance to surface (very simple model)
				signed_distance_to_surface = surface_plane_.W() - surface_plane_.X() * cob_position.X()
				    - surface_plane_.Y() * cob_position.Y() - surface_plane_.Z() * cob_position.Z();
				//std::cerr<<"\n signed_distance_to_surface: "<<signed_distance_to_surface<<" z: "<<(*link_it)->waterSurface.z;
				//if ((*link_it)->model_name.compare("barcoDiferencial")==0)
				//std::cerr<<"\n "<<(*link_it)->model_name<<"  : "<<signed_distance_to_surface<<" b.z: "<<cob_position.z<<" w.z: "<<surface_plane_.w;
				//std::cerr<<"\n "<<(*link_it)->model_name<<" estah com z: "<<(*link_it)->waterSurface.z;
				if (signed_distance_to_surface > -(*link_it)->limit)
				{
					if (signed_distance_to_surface > (*link_it)->limit)
					{
						actual_force *= 0;
					}
					else
					{
						actual_force *= cos (
						M_PI / 4. * (signed_distance_to_surface / (*link_it)->limit + 1));
					}
				}
			}

			// calc wind forces and moments (See Chapter 8 of Handbook of Marine Craft Hydrodynamics and Motion Control - Thor I. Fossen)
			// Vw is the wind Speed; Bw is the angle of wind
			// Yrw angle of attack

			if ((*link_it)->usingNoneWindVelocity != true)
			{
				double mult = (cob_position.Z() + (*link_it)->limit - surface_plane_.W())/(2.0 * (*link_it)->limit);
				if (mult > 1.0)
					mult = 1.0;
				if (mult < 0.0)
					mult = 0.0;

				ignition::math::Vector3<double> Vw  = (*link_it)->wind_velocity_;
				double Afw = (*link_it)->frontal_area*mult; // frontal area
				double Alw = (*link_it)->lateral_area*mult; // lateral area
				double Hlw = signed_distance_to_surface; // height of centroid lateral area
				double Hfw = signed_distance_to_surface; // height of centroid frontal area
				double Loa = (*link_it)->lateral_length; // length overall
				//double urw = u - Vw * cos(Bw - W);
				//double urw = (*link_it)->link->GetWorldLinearVel ().x - Vw.x;
				double urw = (*link_it)->link->WorldPose ().Rot().RotateVectorReverse ((*link_it)->link->WorldLinearVel () - Vw).X();
				//double vrw = v - Vw * sin(Bw - W);
				double vrw = (*link_it)->link->WorldPose ().Rot().RotateVectorReverse ((*link_it)->link->WorldLinearVel () - Vw).Y();
				//double vrw = (*link_it)->link->GetWorldLinearVel ().y - Vw.y;
				double Pa = 1.184;
				double Sl = 0.05;

				// speed boat values
				double CDt = 0.90;
				double CDlaf0 = 0.55;  // coefficient to front wind
				double CDlafPI = 0.60; // coefficient to tail wind
				double tetha = 0.60;
				double k = 1.1;
				// fishing vessel values
				/*double CDt = 0.95;
				double CDlaf0 = 0.70;
				double CDlafPI = 0.70;
				double tetha = 0.40;
				double k = 1.1;*/



				double Yrw = - std::atan2(vrw, urw);
				double Vrw = std::sqrt(urw*urw + vrw *vrw);
				double sin2 = std::sin(2*Yrw);


				double CDlaf = CDlaf0;
				if (Yrw >= 1.57)
					CDlaf = CDlafPI;
				double CDl = CDlaf * Afw / Alw;
				if (Alw < 0.01)
					CDl = 0.0;
				double denominator = 1-0.5*tetha*(1-CDl/CDt)*sin2;
				double Cx = - CDl * (Alw/Afw) * std::cos(Yrw)/ denominator;
				if (Afw < 0.01)
					Cx = 0.0;
				double Cy = CDt * std::sin(Yrw) / denominator;
				//double Cz = 0; // as Fossen said: "only Cx, Cy and Cn are needed"
				double Ck = k*Cy;
				//double Cm;
				double Cn = (Sl/Loa - 0.18*(Yrw-3.1415/2.0))*Cy;
				double Xw = 0.5 * Pa * Vrw * Vrw * Cx * Afw;
				double Yw = 0.5 * Pa * Vrw * Vrw * Cy * Alw;
				//double Zw = 0.5 * Pa * Vrw * Vrw * Cz * Afw;
				double Zw = 0.0;
				double Kw = 0.5 * Pa * Vrw * Vrw * Ck * Alw * Hlw;
				//double Kw = 0.0;
				//double Mw = 0.5 * Pa * Vrw * Vrw * Cm * Afw * Hfw;
				double Mw = 0.0;
				double Nw = 0.5 * Pa * Vrw * Vrw * Cn * Alw * Loa;


				//std::cerr<<"\n "<<(*link_it)->model_name<<" mult: "<<mult<<" signed_distance_to_surface: "<<signed_distance_to_surface<< " w: "<< surface_plane_.z<<" z: "<< cob_position.z;
				//std::cerr<<"\n "<<(*link_it)->model_name<<"["<<(*link_it)->link->GetName()<<"] F: ("<<Xw<<", "<<Yw<<", "<<Zw<<") Vw ("<<Vw.x<<", "<<Vw.y<<", "<<Vw.z<<") M ("<<Kw<<", "<<Mw<<", "<<Nw<<") CN: "<<Cn<<" CY: "<<Cy<<" Yrw: "<<Yrw<<" speed ("<<urw<<", "<<vrw<<") linSpeed: "<<(*link_it)->link->GetWorldLinearVel ().x<<", "<<(*link_it)->link->GetWorldLinearVel ().y<<") mult: "<<mult;
				//std::cerr<<"\n "<<(*link_it)->model_name<<"["<<(*link_it)->link->GetName()<<"] lateral:" <<Alw<<" frontal: "<<Afw;
				(*link_it)->link->AddRelativeTorque (ignition::math::Vector3<double>(Kw, Mw, Nw) );
				//(*link_it)->link->AddForceAtWorldPosition (ignition::math::Vector3<double>(Xw, Yw, Zw), cob_position);
				(*link_it)->link->AddRelativeForce (ignition::math::Vector3<double>(Xw, Yw, Zw));
			}

			// get velocity damping
			// linear velocity difference in the link frame
			if ((*link_it)->usingLocalWaterVelocity)
			{
				//std::cerr<<"\n local: ";
				velocity_difference = (*link_it)->link->WorldPose ().Rot().RotateVectorReverse (
				    (*link_it)->link->WorldLinearVel () - (*link_it)->water_velocity_);
			}
			else if ((*link_it)->usingNoneWaterVelocity)
			{
				velocity_difference = (*link_it)->link->WorldPose ().Rot().RotateVectorReverse (
								    (*link_it)->link->WorldLinearVel () );
				//std::cerr<<"\n none worldLinearVel: "<<(*link_it)->link->GetWorldLinearVel ();
			}
			else
			{
				//std::cerr<<"\n global fluid_velocity_("<<(*link_it)->link->GetName()<<"): "<<fluid_velocity_;
				velocity_difference = (*link_it)->link->WorldPose ().Rot().RotateVectorReverse (
				    (*link_it)->link->WorldLinearVel () - fluid_velocity_);
			}

			// to square
			velocity_difference.X() *= fabs (velocity_difference.X());
			velocity_difference.Y() *= fabs (velocity_difference.Y());
			velocity_difference.Z() *= fabs (velocity_difference.Z());
			// apply damping coefficients
			//std::cerr<<"\n ###actual_force: "<<actual_force;
			actual_force -= (*link_it)->link->WorldPose ().Rot().RotateVector (
			    (*link_it)->linear_damping * velocity_difference);

			//link_it->link->AddForceAtRelativePosition(link_it->link->WorldPose().Rot().RotateVectorReverse(link_it->buoyant_force),
			//                                          link_it->buoyancy_center);
			//std::cerr<<"\n linear_damping["<<(*link_it)->link->GetName()<<"]: "<<(*link_it)->linear_damping<<" vel diff: "<<velocity_difference;
			//std::cerr<<"\n subtract["<<(*link_it)->link->GetName()<<"]: "<<(*link_it)->link->WorldPose ().Rot().RotateVector (	    (*link_it)->linear_damping * velocity_difference);
		//	std::cerr<<"\n bouyancy["<<(*link_it)->link->GetName()<<"]: "<<actual_force;
			//std::cerr<<"\n position["<<(*link_it)->link->GetName()<<"]: "<<(*link_it)->link->WorldPose ();
			//std::cerr<<"\n cob_position["<<(*link_it)->link->GetName()<<"]: "<<cob_position;
			//std::cerr<<"\n world cob["<<(*link_it)->link->GetName()<<"]: "<<(*link_it)->link->GetWorldCoGPose();
			(*link_it)->link->AddForceAtWorldPosition (actual_force, cob_position);

			// same for angular damping
			velocity_difference = (*link_it)->link->RelativeAngularVel ();
			velocity_difference.X() *= fabs (velocity_difference.X());
			velocity_difference.Y() *= fabs (velocity_difference.Y());
			velocity_difference.Z() *= fabs (velocity_difference.Z());
			(*link_it)->link->AddRelativeTorque (-(*link_it)->angular_damping * velocity_difference);
			//std::cerr<<"\n angular damping["<<(*link_it)->link->GetName()<<"]: "<<(*link_it)->angular_damping<<" velocity_difference: "<<velocity_difference;
			//std::cerr<<"\n torque: "<<(-(*link_it)->angular_damping * velocity_difference);

			// publish states as odometry message
			double delta = (ros::Time::now () - this->prevUpdateTime).toSec();
			//std::cerr<<"\n delta: "<<delta << " >= " <<param_update_rate;
			if (delta >= this->param_update_rate)
			{
				this->prevUpdateTime = ros::Time::now ();
				nav_msgs::Odometry state;
				state.header.frame_id = "world";
				state.header.stamp = ros::Time::now ();
				ignition::math::Vector3<double> vec;
				ignition::math::Pose3<double> pose;
				for (model_it = parsed_models_.begin (); model_it != parsed_models_.end (); ++model_it)
				{
					// which link
					state.child_frame_id = "base_link";
					// write absolute pose
					pose = (*model_it)->model_ptr->WorldPose ();
					state.pose.pose.position.x = pose.Pos().X();
					state.pose.pose.position.y = pose.Pos().Y();
					state.pose.pose.position.z = pose.Pos().Z();
					state.pose.pose.orientation.x = pose.Rot().X();
					state.pose.pose.orientation.y = pose.Rot().Y();
					state.pose.pose.orientation.z = pose.Rot().Z();
					state.pose.pose.orientation.w = pose.Rot().W();

					// write relative linear velocity
					vec = (*model_it)->model_ptr->RelativeLinearVel ();
					state.twist.twist.linear.x = vec.X();
					state.twist.twist.linear.y = vec.Y();
					state.twist.twist.linear.z = vec.Z();
					// write relative angular velocity
					vec = (*model_it)->model_ptr->RelativeAngularVel ();
					state.twist.twist.angular.x = vec.X();
					state.twist.twist.angular.y = vec.Y();
					state.twist.twist.angular.z = vec.Z();

					// publish
					(*model_it)->state_publisher.publish (state);
				}
			}


			//  ROS_INFO("Link %s: Applying buoyancy force (%.01f, %.01f, %.01f)", link.name.c_str(), link.buoyant_force.x, link.buoyant_force.y, link.buoyant_force.z);
		}
		//std::cerr<<"\n FreeFloatingFluidPlugin::Update finisheed";
/*		double t2 = ros::Time::now ().toNSec ();
		std::cerr << std::fixed;
		std::cerr<<"\n delta: "<<(t2-t1)<<" t: "<<t2;*/
	}

	void
	FreeFloatingFluidPlugin::ParseNewModel (const physics::ModelPtr &_model)
	{

		//std::cerr << "\n ############################################### START FreeFloatingFluidPlugin::ParseNewModel";
		//std::cerr << "\n ############################################### START FreeFloatingFluidPlugin::ParseNewModel";
		//std::cerr << "\n ############################################### START FreeFloatingFluidPlugin::ParseNewModel";
		//std::cerr << "\n ############################################### START FreeFloatingFluidPlugin::ParseNewModel";
		// define new model structure: name / pointer / publisher to odometry
		model_st* new_model = new model_st ();
		new_model->name = _model->GetName ();
		new_model->model_ptr = _model;
		new_model->state_publisher = rosnode_->advertise<nav_msgs::Odometry> ("/" + _model->GetName () + "/state", 1);
		//std::string topic = "/" + _model->GetName() + "/Surface";

		/*ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
		 topic, 1,
		 boost::bind(&model_st::processSurfaceData, new_model, _1),
		 ros::VoidPtr(), &callback_queue_);*/
		//new_model->createSubscriber(rosnode_, topic);
		// tells this model has been parsed
		parsed_models_.push_back (new_model);

		// get robot description from model name
		// we cannot do anything without the robot_description, as a custom parsing is required to get buoyancy tags
		if (!rosnode_->hasParam ("/" + _model->GetName () + "/" + description_))
			return;

		const unsigned int previous_link_number = buoyant_links_.size ();
		std::string urdf_content;
		rosnode_->getParam ("/" + _model->GetName () + "/" + description_, urdf_content);
		// parse actual URDF as XML (that's ugly) to get custom buoyancy tags

		// links from urdf
		TiXmlDocument urdf_doc;
		urdf_doc.Parse (urdf_content.c_str (), 0);

		const ignition::math::Vector3<double> WORLD_GRAVITY = world_->Physics ()->World()->Gravity ();

		TiXmlElement* urdf_root = urdf_doc.FirstChildElement ();
		TiXmlNode* urdf_node, *link_node, *buoy_node;
		double compensation;
		unsigned int link_index, joint_index;
		physics::LinkPtr sdf_link;
		bool found;

		for (urdf_node = urdf_root->FirstChild (); urdf_node != 0; urdf_node = urdf_node->NextSibling ())
		{
			if (urdf_node->ValueStr () == "link")
			{
				//ROS_INFO("++++++++++++++++++++++++++++++++++++NODE : %s", urdf_node->ToElement()->Attribute("name"));

				// find corresponding sdf model link if any
				found = false;
				for (link_index = 0; link_index < _model->GetLinks ().size (); ++link_index)
				{
					//ROS_INFO("match %s", _model->GetLinks()[link_index]->GetName().c_str());
					ROS_WARN(" LINK: %s", _model->GetLinks ()[link_index]->GetName ().c_str ());
					if (urdf_node->ToElement ()->Attribute ("name") == _model->GetLinks ()[link_index]->GetName ())
					{
						found = true;
						sdf_link = _model->GetLinks ()[link_index];
						break;
					}
				}

				if (found)
				{
					for (link_node = urdf_node->FirstChild (); link_node != 0; link_node = link_node->NextSibling ())
					{
						if (link_node->ValueStr () == "buoyancy")
						{
							// this link is subject to buoyancy, create an instance
							link_st* new_buoy_link = new link_st ();
							new_buoy_link->model_name = _model->GetName (); // in case this model is deleted
							new_buoy_link->link = sdf_link;    // to apply forces
							new_buoy_link->limit = .1;
							std::string topic = "/" + _model->GetName () + "/Surface/" + urdf_node->ToElement ()->Attribute ("name");
							new_buoy_link->frontal_area = 0.04;
							new_buoy_link->lateral_area = 0.06;
							new_buoy_link->lateral_length = 1.2;

							new_buoy_link->createSubscriberWaterSurface (rosnode_, topic);
							// get data from urdf
							// default values
							new_buoy_link->buoyancy_center = sdf_link->GetInertial ()->CoG ();
							new_buoy_link->linear_damping = new_buoy_link->angular_damping = 5 * ignition::math::Vector3<double>::One
							    * sdf_link->GetInertial ()->Mass ();

							compensation = 0;
							for (buoy_node = link_node->FirstChild (); buoy_node != 0; buoy_node = buoy_node->NextSibling ())
							{
								if (buoy_node->ValueStr () == "origin")
									ReadVector3 ((buoy_node->ToElement ()->Attribute ("xyz")), new_buoy_link->buoyancy_center);
								else if (buoy_node->ValueStr () == "compensation")
									compensation = atof (buoy_node->ToElement ()->GetText ());
								else if (buoy_node->ValueStr () == "limit")
								{
									std::stringstream ss (buoy_node->ToElement ()->Attribute ("radius"));
									ss >> new_buoy_link->limit;
								}
								else if (buoy_node->ValueStr () == "frontal_area")
								{
									std::stringstream ss (buoy_node->ToElement ()->GetText ());
									ss >> new_buoy_link->frontal_area;
								}
								else if (buoy_node->ValueStr () == "lateral_area")
								{
									std::stringstream ss (buoy_node->ToElement ()->GetText ());
									ss >> new_buoy_link->lateral_area;
								}
								else if (buoy_node->ValueStr () == "lateral_length")
								{
									std::stringstream ss (buoy_node->ToElement ()->GetText ());
									ss >> new_buoy_link->lateral_length;
								}
								else if (buoy_node->ValueStr () == "waterVelocity")
								{
									std::string nameLocal = "local";
									std::string nameGlobal = "global";
									std::string nameNone = "none";
									//std::cerr<<"\n waterVelocity type("<<urdf_node->ToElement()->Attribute("name")<<"): "<<buoy_node->ToElement ()->GetText ();
									if (nameLocal.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWaterVelocity = true;
										new_buoy_link->usingNoneWaterVelocity = false;
										new_buoy_link->initServiceClient (rosnode_);

										//new_buoy_link->Start ();
									}
									else if (nameNone.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWaterVelocity = false;
										new_buoy_link->usingNoneWaterVelocity = true;
										fluid_velocity_.Set (0, 0, 0);
									}
									else if (nameGlobal.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWaterVelocity = false;
										new_buoy_link->usingNoneWaterVelocity = false;
										new_buoy_link->createSubscriberWaterCurrent(rosnode_, "\gazebo\current");
									}
								}
								else if (buoy_node->ValueStr () == "windVelocity")
								{
									std::string nameLocal = "local";
									std::string nameGlobal = "global";
									std::string nameNone = "none";

									//std::cerr<<"\n windVelocity type("<<urdf_node->ToElement()->Attribute("name")<<"): "<<buoy_node->ToElement ()->GetText ();
									if (nameLocal.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWindVelocity = true;
										new_buoy_link->usingNoneWindVelocity = false;
										new_buoy_link->initWindServiceClient (rosnode_);

										//new_buoy_link->Start ();
									}
									else if (nameNone.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWindVelocity = false;
										new_buoy_link->usingNoneWindVelocity = true;
										new_buoy_link->wind_velocity_.Set (0, 0, 0);
									}
									else if (nameGlobal.compare (buoy_node->ToElement ()->GetText ()) == 0)
									{
										new_buoy_link->usingLocalWindVelocity = false;
										new_buoy_link->usingNoneWindVelocity = false;
										double x, y;
										rosnode_->getParam ("/uwsim/wind/x", x);
										rosnode_->getParam ("/uwsim/wind/y", y);
										new_buoy_link->wind_velocity_.Set (x, y, 0);
										//std::cerr<<"\n\n wind: "<<x<<", "<<y;
									}
								}
								else if (buoy_node->ValueStr () == "damping")
								{
									if (buoy_node->ToElement ()->Attribute ("xyz") != NULL)
									{
										ReadVector3 ((buoy_node->ToElement ()->Attribute ("xyz")), new_buoy_link->linear_damping);
									}
									if (buoy_node->ToElement ()->Attribute ("rpy") != NULL)
									{
										ReadVector3 ((buoy_node->ToElement ()->Attribute ("rpy")), new_buoy_link->angular_damping);
									}
								}
								else
									ROS_WARN("Unknown tag <%s/> in buoyancy node for model %s", buoy_node->ValueStr ().c_str (),
									         _model->GetName ().c_str ());
							}
							if (new_buoy_link->usingLocalWaterVelocity || new_buoy_link->usingLocalWindVelocity)
								new_buoy_link->Start ();
							new_buoy_link->buoyant_force = -compensation * sdf_link->GetInertial ()->Mass () * WORLD_GRAVITY;

							// store this link
							buoyant_links_.push_back (new_buoy_link);
						}
					}   // out of loop: buoyancy-related nodes
				}       // out of condition: in sdf
			}           // out of loop: links
		}               // out of loop: all urdf nodes
		if (previous_link_number == buoyant_links_.size ())
			ROS_INFO_NAMED("Buoyancy plugin", "No links subject to buoyancy inside %s", _model->GetName ().c_str ());
		else
			ROS_INFO_NAMED("Buoyancy plugin", "Added %i buoy links from %s",
			               (int ) buoyant_links_.size () - previous_link_number, _model->GetName ().c_str ());
		//std::cerr << "\n ### FINISHED FreeFloatingFluidPlugin::ParseNewModel";
	}

	void
	FreeFloatingFluidPlugin::RemoveDeletedModel (std::vector<model_st*>::iterator &_model_it)
	{
		ROS_INFO("Removing deleted model: %s", (*_model_it)->name.c_str ());

		// remove model stored links
		std::vector<link_st*>::iterator link_it = buoyant_links_.begin ();
		while (link_it != buoyant_links_.end ())
		{
			if ((*link_it)->model_name == (*_model_it)->name)
			{
				delete *link_it;
				link_it = buoyant_links_.erase (link_it);
			}
			else
				++link_it;
		}
		delete *_model_it;
		// remove it from the list
		_model_it = parsed_models_.erase (_model_it);
	}

	void
	FreeFloatingFluidPlugin::FluidVelocityCallBack (const geometry_msgs::Vector3ConstPtr &_msg)
	{
		// store fluid velocity
		fluid_velocity_.Set (_msg->x, _msg->y, _msg->z);
	}

}

