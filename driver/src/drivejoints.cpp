#include "driver/driver.h"

Posdriver::Posdriver()
{
    PosJoints_actual = nh.subscribe("/joint_states", 1, &Posdriver::DataCallback, this);
    PosPiece = nh.subscribe("piece_number", 1, &Posdriver::ArduinoCallback, this);
    PosPlace = nh.advertise<std_msgs::Int8>("position_place", 1);
    // PosJoints = nh.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 1);
    PosJoints = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/ur_rtde/controllers/joint_space_controller/command", 1);
    gripper_client = nh.serviceClient<ur_rtde_controller::RobotiQGripperControl>("/ur_rtde/robotiq_gripper/command");

    inverse = nh.serviceClient<ur_kdl::IK>("inverse_kinematics_lma");
    for(int i=0; i<6; i++)
    {
        srv.request.actual_joint_state.position.push_back(0.0);
        pos.push_back(0.0);
        reachpos.push_back(0.0);
        actualpos.push_back(0.0);
        diffpos.push_back(0.0);
    }

    
    // posdr.joint_names.resize(6);
    // posdr.points.resize(1);
    // posdr.points[0].positions.resize(6);
    posdr.positions.resize(6);

    PosHomex = 0.32644;
	PosHomey = 0.50205;
	PosHomez = 0.35175;

    // TODO: Posizione Tappetino
	PosTappx = 0.03897;
	PosTappy = 0.97584;
    PosTappz = 0.24315;

    // TODO: Posizione Z di Ogni Formina ? 
	Posz = 0.27852;

    vel = 0.0215;
    actpositionx = PosHomex;
    actpositiony = PosHomey;
    actpositionz = PosHomez;
 
 // PxT fa riferimento alla casella i-esima (T sta per temporanea)
    //Px sta ad indicare la coordinata x dell’i-esima formina.
    
    for (int i=1 ; i<41 ; i++)
    {
        Px.push_back(0.0);
        Py.push_back(0.0);
    }
   
    for (int i=1 ; i<81 ; i++)
    {
        PxT.push_back(0.0);
        PyT.push_back(0.0);
    }

    //Definizione posizioni temporanee per definire la posizione dei pezzi

    // TODO: Posizioni Per Il Calcolo Iniziale 
    PxT[1] = 0.17521;  
    PyT[1] = 0.59657;
    PxT[8] = 0.46819;
    PyT[8] = 0.30004;
    
    
    alfa = atan((PyT[8]-PyT[1])/(PxT[8]-PxT[1])) * 180 / Pi_G;
    beta = alfa + 90;
                            //0.3643583, 0.931033, -0.009754, -0.0180392
    quatx = 0.3643583;
    quaty = 0.931033;
    quatz = -0.009754;
	quatw = -0.0180392;

    //0.0389005, 0.8494443, 0.5197352, 0.0825012

    PxT[73] = PxT[1] + 0.540 * cos (beta * Pi_G / 180);
    PyT[73] = PyT[1] + 0.540 * sin (beta * Pi_G / 180);
    PxT[80] = PxT[73] + 0.420 * cos (alfa * Pi_G / 180);
    PyT[80] = PyT[73] + 0.420 * sin (alfa * Pi_G / 180);

    PxT[9] = (PxT[73] - PxT[1])/9 + PxT[1];         // Definizione posizione temporanea colonna sinistra
    PyT[9] = (PyT[73] - PyT[1])/9 + PyT[1];
    PxT[17] = 2*(PxT[73] - PxT[1])/9 + PxT[1];
    PyT[17] = 2*(PyT[73] - PyT[1])/9 + PyT[1];
    PxT[25] = 3*(PxT[73] - PxT[1])/9 + PxT[1];
    PyT[25] = 3*(PyT[73] - PyT[1])/9 + PyT[1];
    PxT[33] = 4*(PxT[73] - PxT[1])/9 + PxT[1];
    PyT[33] = 4*(PyT[73] - PyT[1])/9 + PyT[1];
    PxT[41] = 5*(PxT[73] - PxT[1])/9 + PxT[1];
    PyT[41] = 5*(PyT[73] - PyT[1])/9 + PyT[1];
    PxT[49] = 6*(PxT[73] - PxT[1])/9 + PxT[1];
    PyT[49] = 6*(PyT[73] - PyT[1])/9 + PyT[1];
    PxT[57] = 7*(PxT[73] - PxT[1])/9 + PxT[1];
    PyT[57] = 7*(PyT[73] - PyT[1])/9 + PyT[1];
    PxT[65] = 8*(PxT[73] - PxT[1])/9 + PxT[1];
    PyT[65] = 8*(PyT[73] - PyT[1])/9 + PyT[1];

    PxT[16] = (PxT[80] - PxT[8])/9 + PxT[8];        // Definizione posizione temporanea colonna destra
    PyT[16] = (PyT[80] - PyT[8])/9 + PyT[8];
    PxT[24] = 2*(PxT[80] - PxT[8])/9 + PxT[8];
    PyT[24] = 2*(PyT[80] - PyT[8])/9 + PyT[8];
    PxT[32] = 3*(PxT[80] - PxT[8])/9 + PxT[8];
    PyT[32] = 3*(PyT[80] - PyT[8])/9 + PyT[8];
    PxT[40] = 4*(PxT[80] - PxT[8])/9 + PxT[8];
    PyT[40] = 4*(PyT[80] - PyT[8])/9 + PyT[8];
    PxT[48] = 5*(PxT[80] - PxT[8])/9 + PxT[8];
    PyT[48] = 5*(PyT[80] - PyT[8])/9 + PyT[8];
    PxT[56] = 6*(PxT[80] - PxT[8])/9 + PxT[8];
    PyT[56] = 6*(PyT[80] - PyT[8])/9 + PyT[8];
    PxT[64] = 7*(PxT[80] - PxT[8])/9 + PxT[8];
    PyT[64] = 7*(PyT[80] - PyT[8])/9 + PyT[8];
    PxT[72] = 8*(PxT[80] - PxT[8])/9 + PxT[8];
    PyT[72] = 8*(PyT[80] - PyT[8])/9 + PyT[8];

    //Definizione posizione pezzi

    Px[1] = PxT[1];
    Py[1] = PyT[1];
    Px[2] = 2*(PxT[8] - PxT[1])/7 + PxT[1];
    Py[2] = 2*(PyT[8] - PyT[1])/7 + PyT[1];
    Px[3] = 4*(PxT[8] - PxT[1])/7 + PxT[1];
    Py[3] = 4*(PyT[8] - PyT[1])/7 + PyT[1];
    Px[4] = 6*(PxT[8] - PxT[1])/7 + PxT[1];
    Py[4] = 6*(PyT[8] - PyT[1])/7 + PyT[1];

    Px[5] = (PxT[16] - PxT[9])/7 + PxT[9];
    Py[5] = (PyT[16] - PyT[9])/7 + PyT[9];
    Px[6] = 3*(PxT[16] - PxT[9])/7 + PxT[9];
    Py[6] = 3*(PyT[16] - PyT[9])/7 + PyT[9];
    Px[7] = 5*(PxT[16] - PxT[9])/7 + PxT[9]; 
    Py[7] = 5*(PyT[16] - PyT[9])/7 + PyT[9];
    Px[8] = PxT[16];
    Py[8] = PyT[16];

    Px[9] = PxT[17];
    Py[9] = PyT[17];
    Px[10] = 2*(PxT[24] - PxT[17])/7 + PxT[17];
    Py[10] = 2*(PyT[24] - PyT[17])/7 + PyT[17];
    Px[11] = 4*(PxT[24] - PxT[17])/7 + PxT[17];
    Py[11] = 4*(PyT[24] - PyT[17])/7 + PyT[17];
    Px[12] = 6*(PxT[24] - PxT[17])/7 + PxT[17];
    Py[12] = 6*(PyT[24] - PyT[17])/7 + PyT[17];

    Px[13] = (PxT[32] - PxT[25])/7 + PxT[25];
    Py[13] = (PyT[32] - PyT[25])/7 + PyT[25];
    Px[14] = 3*(PxT[32] - PxT[25])/7 + PxT[25];
    Py[14] = 3*(PyT[32] - PyT[25])/7 + PyT[25];
    Px[15] = 5*(PxT[32] - PxT[25])/7 + PxT[25];
    Py[15] = 5*(PyT[32] - PyT[25])/7 + PyT[25];
    Px[16] = PxT[32];
    Py[16] = PyT[32];

    Px[17] = PxT[33];
    Py[17] = PyT[33];
    Px[18] = 2*(PxT[40] - PxT[33])/7 + PxT[33];
    Py[18] = 2*(PyT[40] - PyT[33])/7 + PyT[33];
    Px[19] = 4*(PxT[40] - PxT[33])/7 + PxT[33];
    Py[19] = 4*(PyT[40] - PyT[33])/7 + PyT[33];
    Px[20] = 6*(PxT[40] - PxT[33])/7 + PxT[33];
    Py[20] = 6*(PyT[40] - PyT[33])/7 + PyT[33];

    Px[21] = (PxT[48] - PxT[41])/7 + PxT[41];
    Py[21] = (PyT[48] - PyT[41])/7 + PyT[41];
    Px[22] = 3*(PxT[48] - PxT[41])/7 + PxT[41];
    Py[22] = 3*(PyT[48] - PyT[41])/7 + PyT[41];
    Px[23] = 5*(PxT[48] - PxT[41])/7 + PxT[41];
    Py[23] = 5*(PyT[48] - PyT[41])/7 + PyT[41];
    Px[24] = PxT[48];
    Py[24] = PyT[48];

    Px[25] = PxT[49];
    Py[25] = PyT[49];
    Px[26] = 2*(PxT[56] - PxT[49])/7 + PxT[49];
    Py[26] = 2*(PyT[56] - PyT[49])/7 + PyT[49];
    Px[27] = 4*(PxT[56] - PxT[49])/7 + PxT[49];
    Py[27] = 4*(PyT[56] - PyT[49])/7 + PyT[49];
    Px[28] = 6*(PxT[56] - PxT[49])/7 + PxT[49];
    Py[28] = 6*(PyT[56] - PyT[49])/7 + PyT[49];

    Px[29] = (PxT[64] - PxT[57])/7 + PxT[57];
    Py[29] = (PyT[64] - PyT[57])/7 + PyT[57];
    Px[30] = 3*(PxT[64] - PxT[57])/7 + PxT[57];
    Py[30] = 3*(PyT[64] - PyT[57])/7 + PyT[57];
    Px[31] = 5*(PxT[64] - PxT[57])/7 + PxT[57];
    Py[31] = 5*(PyT[64] - PyT[57])/7 + PyT[57];
    Px[32] = PxT[64];
    Py[32] = PyT[64];

    Px[33] = PxT[65];
    Py[33] = PyT[65];
    Px[34] = 2*(PxT[72] - PxT[65])/7 + PxT[65];
    Py[34] = 2*(PyT[72] - PyT[65])/7 + PyT[65];
    Px[35] = 4*(PxT[72] - PxT[65])/7 + PxT[65];
    Py[35] = 4*(PyT[72] - PyT[65])/7 + PyT[65];
    Px[36] = 6*(PxT[72] - PxT[65])/7 + PxT[65];
    Py[36] = 6*(PyT[72] - PyT[65])/7 + PyT[65];

    Px[37] = (PxT[80] - PxT[73])/7 + PxT[73];
    Py[37] = (PyT[80] - PyT[73])/7 + PyT[73];
    Px[38] = 3*(PxT[80] - PxT[73])/7 + PxT[73];
    Py[38] = 3*(PyT[80] - PyT[73])/7 + PyT[73];
    Px[39] = 5*(PxT[80] - PxT[73])/7 + PxT[73];
    Py[39] = 5*(PyT[80] - PyT[73])/7 + PyT[73];
    Px[40] = PxT[80];
    Py[40] = PyT[80];

    pos_pla.data = 1;

    // Move to Home
    moveHome(PosHomex, PosHomey, 0.75, quatx, quaty, quatz, quatw);

}

void Posdriver::moveHome(float x, float y, float z, float ox, float oy, float oz, float ow)
{
    // Home Coordinates
    srv.request.ee_pose.position.x = x;
    srv.request.ee_pose.position.y = y;
    srv.request.ee_pose.position.z = z;
    srv.request.ee_pose.orientation.x = ox;
    srv.request.ee_pose.orientation.y = oy;
    srv.request.ee_pose.orientation.z = oz;
    srv.request.ee_pose.orientation.w = ow;

    // Call IK
    if (inverse.call(srv))
        {
            for(int i=0; i<6; i++)
            {
                pos[i]=srv.response.joint_values.position[i];
                reachpos[i] = round(srv.response.joint_values.position[i]*100)/100;
            }

            posdr.positions = {pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]};
            posdr.time_from_start = ros::Duration(5);
            PosJoints.publish(posdr);
            ros::Duration(5).sleep();
        }
    else
        {
            ROS_ERROR("Failed to call service");
        }

    ROS_WARN("Reached Home Position");

}

void Posdriver::moveGripper(float position, float speed, float force)
{
    ur_rtde_controller::RobotiQGripperControl gripper_service;
    gripper_service.request.position = position;
    gripper_service.request.speed = speed;
    gripper_service.request.force = force;

    gripper_client.call(gripper_service);
}

void Posdriver::DataCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(int i=0; i<6; i++)
    {
        // if (i == 0)
        // {
        //     srv.request.actual_joint_state.position[2] = msg->position[0];
        //     actualpos[2] = round(msg->position[0]*100)/100;
        // }
        // else if (i == 2)
        // {
        //     srv.request.actual_joint_state.position[0] = msg->position[2];
        //     actualpos[0] = round(msg->position[2]*100)/100;
        // }
        // else
        // {
            srv.request.actual_joint_state.position[i] = msg->position[i];
            actualpos[i] = round(msg->position[i]*100)/100;
        // }
    }

    if (newmess == 1 && phase == 0)
    {
        j=1;
        newmess=0;
    }

    if (j==1)
    {
        if (k==0)
        {
            if (phase == 0)
            {
                srv.request.ee_pose.position.z = 0.75;
                srv.request.ee_pose.orientation.x = quatx;
                srv.request.ee_pose.orientation.y = quaty;
                srv.request.ee_pose.orientation.z = quatz;
                srv.request.ee_pose.orientation.w = quatw;
                despositionz = 0.75;

                for (int i=1 ; i<41 ; i++)
                {
                    if (numbp == i)
                    {
                        srv.request.ee_pose.position.x = Px[i];
                        srv.request.ee_pose.position.y = Py[i];
                        despositionx = Px[i];
                        despositiony = Py[i];
                    }
                }
                k=1;
            }

            if (phase == 1)
            {
                srv.request.ee_pose.position.z = Posz;
                srv.request.ee_pose.orientation.x = quatx;
                srv.request.ee_pose.orientation.y = quaty;
                srv.request.ee_pose.orientation.z = quatz;
                srv.request.ee_pose.orientation.w = quatw;
                despositionz = Posz;

                for (int i=1 ; i<41 ; i++)
                {
                    if (numbp == i)
                    {
                        srv.request.ee_pose.position.x = Px[i];
                        srv.request.ee_pose.position.y = Py[i];
                        despositionx = Px[i];
                        despositiony = Py[i];
                    }
                }
                k=1;
            }

            if (phase == 2)
            {
                srv.request.ee_pose.position.z = 0.75;
                srv.request.ee_pose.orientation.x = quatx;
                srv.request.ee_pose.orientation.y = quaty;
                srv.request.ee_pose.orientation.z = quatz;
                srv.request.ee_pose.orientation.w = quatw;
                despositionz = 0.75;

                for (int i=1 ; i<41 ; i++)
                {
                    if (numbp == i)
                    {
                        srv.request.ee_pose.position.x = Px[i];
                        srv.request.ee_pose.position.y = Py[i];
                        despositionx = Px[i];
                        despositiony = Py[i];
                    }
                }
                k=1;
            }

            if (phase == 3)
            {
                srv.request.ee_pose.position.x = PosTappx;
                srv.request.ee_pose.position.y = PosTappy;
                srv.request.ee_pose.position.z = 0.75;
                srv.request.ee_pose.orientation.x = quatx;
                srv.request.ee_pose.orientation.y = quaty;
                srv.request.ee_pose.orientation.z = quatz;
                srv.request.ee_pose.orientation.w = quatw;
                despositionx = PosTappx;
                despositiony = PosTappy;
                despositionz = 0.75;
                k=1;
            }

            if (phase == 4)
            {
                srv.request.ee_pose.position.x = PosTappx;
                srv.request.ee_pose.position.y = PosTappy;
                srv.request.ee_pose.position.z = Posz+0.01;
                srv.request.ee_pose.orientation.x = quatx;
                srv.request.ee_pose.orientation.y = quaty;
                srv.request.ee_pose.orientation.z = quatz;
                srv.request.ee_pose.orientation.w = quatw;
                despositionx = PosTappx;
                despositiony = PosTappy;
                despositionz = PosTappz;
                k=1;
            }

            if (phase == 5)
            {
                srv.request.ee_pose.position.x = PosTappx;
                srv.request.ee_pose.position.y = PosTappy;
                srv.request.ee_pose.position.z = 0.75;
                srv.request.ee_pose.orientation.x = quatx;
                srv.request.ee_pose.orientation.y = quaty;
                srv.request.ee_pose.orientation.z = quatz;
                srv.request.ee_pose.orientation.w = quatw;
                despositionx = PosTappx;
                despositiony = PosTappy;
                despositionz = 0.75;
                k=1;
            }

            if (phase == 6)
            {
                srv.request.ee_pose.position.x = PosHomex;
                srv.request.ee_pose.position.y = PosHomey;
                srv.request.ee_pose.position.z = 0.75;
                srv.request.ee_pose.orientation.x = quatx;
                srv.request.ee_pose.orientation.y = quaty;
                srv.request.ee_pose.orientation.z = quatz;
                srv.request.ee_pose.orientation.w = quatw;
                despositionx = PosHomex;
                despositiony = PosHomey;
                despositionz = 0.75;
                k=1;
            }

            if (inverse.call(srv))
                {
                    for(int i=0; i<6; i++)
                    {
                        pos[i]=srv.response.joint_values.position[i];
                        reachpos[i] = round(srv.response.joint_values.position[i]*100)/100;
                    }
                    // posdr.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
                    // posdr.points[0].positions = {pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]};
                    posdr.positions = {pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]};
                    timeto = (sqrt(((despositionx-actpositionx)*(despositionx-actpositionx))+((despositiony-actpositiony)*(despositiony-actpositiony))+((despositionz-actpositionz)*(despositionz-actpositionz))))/vel;
                    // posdr.points[0].time_from_start = ros::Duration(timeto);
                    posdr.time_from_start = ros::Duration(timeto);
                    //ROS_INFO("tempo di esecuzione %f",timeto);
                    PosJoints.publish(posdr);
                    ros::Duration(timeto).sleep();
                }
            else
                {
                    ROS_ERROR("Failed to call service");
                }
        }

        for (int i = 0 ; i < 6 ; i++)
        {
            diffpos[i] = abs(actualpos[i]-reachpos[i]);
        }

                                    //while(robot non arrivato){ros::spinOnce();}
        if (diffpos[0]<0.001 && diffpos[1]<0.001 && diffpos[2]<0.001 && diffpos[3]<0.001 && diffpos[4]<0.001 && diffpos[5]<0.001)
        {
            actpositionx = srv.request.ee_pose.position.x;
            actpositiony = srv.request.ee_pose.position.y;
            actpositionz = srv.request.ee_pose.position.z;
            if (phase == 1) 
            {
                // gripper.close();
                moveGripper(0, 100, 100);
                ros::Duration(0.2).sleep();
            }
            // if (phase == 4) gripper.open();
            if (phase == 4) moveGripper(100, 100, 100);
            ROS_INFO("fase%d",phase);

            phase++;
            if (phase==7)
            {
                PosPlace.publish(pos_pla);
                phase=0;
                j=0;
            }
            k=0;
        }
    }
}

void Posdriver::ArduinoCallback(const std_msgs::Int8::ConstPtr& piece)
{
    numbp = piece->data;
    newmess=1;
}

void Posdriver::spinner()
{
    ros::spinOnce();
}
