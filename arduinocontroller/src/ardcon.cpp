#include "arduinocontroller/arduinocontroller.h"

Arduino::Arduino()
{
    PieceFind = nh.subscribe("piece_find", 1, &Arduino::Callback, this);
    NextPieceSub = nh.subscribe("position_place", 1, &Arduino::NextPieceCallback, this);
    PieceNumber = nh.advertise<std_msgs::Int8>("piece_number", 1);
    ros::Duration(1).sleep();
    srand (time(NULL));
    

    for (int l = 0 ; l < 40 ; l++)
    {
        randpz.push_back(0.0);
    }

    ROS_INFO("Numero prova (da 1 a 4):");
    std::cin >> n;
    
    if(n==1)
    {
        myfile.open ("/home/matilda/AppuntiROS/prova1.txt");
        myfile << "Prova 1:" << "\n";
        myfile << "pezzo da inserire\t" << "tempo pos tapp\t\t" << "tempo prel tapp\t\t" << "tempo posizionamento\t" << "posizione\n";
        randpz[0] = 9;
        randpz[1] = 2;
        randpz[2] = 21;
        randpz[3] = 14;
        randpz[4] = 40;
        randpz[5] = 12;
        randpz[6] = 7;
        randpz[7] = 35;
        randpz[8] = 33;
        randpz[9] = 26;
        randpz[10] = 37;
        randpz[11] = 24;
        randpz[12] = 38;
        randpz[13] = 8;
        randpz[14] = 17;
        randpz[15] = 13;
        randpz[16] = 36;
        randpz[17] = 22;
        randpz[18] = 25;
        randpz[19] = 1;
        randpz[20] = 6;
        randpz[21] = 4;
        randpz[22] = 27;
        randpz[23] = 5;
        randpz[24] = 29;
        randpz[25] = 20;
        randpz[26] = 32;
        randpz[27] = 19;
        randpz[28] = 31;
        randpz[29] = 10;
        randpz[30] = 30;
        randpz[31] = 16;
        randpz[32] = 18;
        randpz[33] = 39;
        randpz[34] = 15;
        randpz[35] = 34;
        randpz[36] = 3;
        randpz[37] = 11;
        randpz[38] = 28;
        randpz[39] = 23;
    }

    if(n==2)
    {
        myfile.open ("/home/matilda/AppuntiROS/prova2.txt");
        myfile << "Prova 2:" << "\n";
        myfile << "pezzo da inserire\t" << "tempo pos tapp\t\t" << "tempo prel tapp\t\t" << "tempo posizionamento\t" << "posizione\n";
        randpz[0] = 11;
        randpz[1] = 4;
        randpz[2] = 14;
        randpz[3] = 12;
        randpz[4] = 7;
        randpz[5] = 38;
        randpz[6] = 6;
        randpz[7] = 17;
        randpz[8] = 29;
        randpz[9] = 22;
        randpz[10] = 20;
        randpz[11] = 35;
        randpz[12] = 31;
        randpz[13] = 16;
        randpz[14] = 5;
        randpz[15] = 21;
        randpz[16] = 3;
        randpz[17] = 10;
        randpz[18] = 32;
        randpz[19] = 39;
        randpz[20] = 30;
        randpz[21] = 9;
        randpz[22] = 28;
        randpz[23] = 15;
        randpz[24] = 36;
        randpz[25] = 1;
        randpz[26] = 2;
        randpz[27] = 34;
        randpz[28] = 24;
        randpz[29] = 40;
        randpz[30] = 8;
        randpz[31] = 26;
        randpz[32] = 23;
        randpz[33] = 18;
        randpz[34] = 37;
        randpz[35] = 19;
        randpz[36] = 25;
        randpz[37] = 33;
        randpz[38] = 13;
        randpz[39] = 27;
    }

    if(n==3)
    {
        myfile.open ("/home/matilda/AppuntiROS/prova3.txt");
        myfile << "Prova 3:" << "\n";
        myfile << "pezzo da inserire\t" << "tempo pos tapp\t\t" << "tempo prel tapp\t\t" << "tempo posizionamento\t" << "posizione\n";
        randpz[0] = 25;
        randpz[1] = 14;
        randpz[2] = 11;
        randpz[3] = 28;
        randpz[4] = 24;
        randpz[5] = 10;
        randpz[6] = 32;
        randpz[7] = 23;
        randpz[8] = 7;
        randpz[9] = 15;
        randpz[10] = 12;
        randpz[11] = 39;
        randpz[12] = 31;
        randpz[13] = 17;
        randpz[14] = 21;
        randpz[15] = 27;
        randpz[16] = 18;
        randpz[17] = 35;
        randpz[18] = 30;
        randpz[19] = 16;
        randpz[20] = 38;
        randpz[21] = 9;
        randpz[22] = 26;
        randpz[23] = 13;
        randpz[24] = 19;
        randpz[25] = 3;
        randpz[26] = 37;
        randpz[27] = 6;
        randpz[28] = 33;
        randpz[29] = 1;
        randpz[30] = 4;
        randpz[31] = 20;
        randpz[32] = 36;
        randpz[33] = 22;
        randpz[34] = 29;
        randpz[35] = 8;
        randpz[36] = 5;
        randpz[37] = 34;
        randpz[38] = 2;
        randpz[39] = 40;
    }

    if(n==4)
    {
        myfile.open ("/home/matilda/AppuntiROS/prova4.txt");
        myfile << "Prova 4:" << "\n";
        myfile << "pezzo da inserire\t" << "tempo pos tapp\t\t" << "tempo prel tapp\t\t" << "tempo posizionamento\t" << "posizione\n";
        randpz[0] = 11;
        randpz[1] = 22;
        randpz[2] = 4;
        randpz[3] = 36;
        randpz[4] = 18;
        randpz[5] = 9;
        randpz[6] = 37;
        randpz[7] = 13;
        randpz[8] = 34;
        randpz[9] = 12;
        randpz[10] = 29;
        randpz[11] = 40;
        randpz[12] = 25;
        randpz[13] = 24;
        randpz[14] = 32;
        randpz[15] = 10;
        randpz[16] = 30;
        randpz[17] = 28;
        randpz[18] = 15;
        randpz[19] = 1;
        randpz[20] = 20;
        randpz[21] = 39;
        randpz[22] = 19;
        randpz[23] = 35;
        randpz[24] = 23;
        randpz[25] = 6;
        randpz[26] = 17;
        randpz[27] = 3;
        randpz[28] = 27;
        randpz[29] = 16;
        randpz[30] = 7;
        randpz[31] = 38;
        randpz[32] = 14;
        randpz[33] = 21;
        randpz[34] = 26;
        randpz[35] = 8;
        randpz[36] = 33;
        randpz[37] = 31;
        randpz[38] = 2;
        randpz[39] = 5;
    }

    for (int l = 0 ; l < 40 ; l++)
    {
        ROS_INFO("vettore:[%i]" , randpz[l] );
    }

    k = 1;
}

void Arduino::Callback(const std_msgs::Int8::ConstPtr& Corr)
{

    if (Corr->data < 31)
    {
        pe = Corr->data;
        Cor = 1;
        pospez = ros::Time::now();
        timepos = pospez - starttime; 
        ROS_INFO("pos");
    }

    if (Corr->data == 50)
    {
        tappDown = ros::Time::now();
        timeTapDown = tappDown - starttime;
        ROS_INFO("tappdown");
    }

    if (Corr->data == 51)
    {
        Pos = 1;
        tappUp = ros::Time::now();
        timeTapUp = tappUp - starttime;
        ROS_INFO("tapup");
    }
}

void Arduino::NextPieceCallback(const std_msgs::Int8::ConstPtr& data)
{
    send_next_piece = true;
}

void Arduino::spinner()
{
    if (Cor == 1 && Pos == 1 && k == 40)
    {
        Pos = 0;
        Cor = 0;
        ROS_INFO("Posizione scelta: %i",pe);
        ROS_INFO("Tempo posizionamento su piattaforma: %d [sec] %d [nsec]", timeTapDown.sec , timeTapDown.nsec);
        ROS_INFO("Tempo prelevamento da piattaforma: %d [sec] %d [nsec]", timeTapUp.sec , timeTapUp.nsec);
        ROS_INFO("Tempo posizione pezzo: %d [sec] %d [nsec]", timepos.sec , timepos.nsec);
        ROS_INFO("Fine dell'esperimento");

        myfile << pez << "\t\t\t" << timeTapDown.sec << "." << timeTapDown.nsec << "\t\t" << timeTapUp.sec << "." << timeTapUp.nsec << "\t\t" << timepos.sec << "." << timepos.nsec << "\t\t" << pe << "\n\n\n";
    }
    
    if (Cor == 1 && Pos == 1 && k < 40 && send_next_piece == true)
    {
        Pos = 0;
        Cor = 0;
        ROS_INFO("Posizione scelta: %i",pe);
        ROS_INFO("Tempo posizionamento su piattaforma: %d [sec] %d [nsec]", timeTapDown.sec , timeTapDown.nsec);
        ROS_INFO("Tempo prelevamento da piattaforma: %d [sec] %d [nsec]", timeTapUp.sec , timeTapUp.nsec);
        ROS_INFO("Tempo posizione pezzo: %d [sec] %d [nsec]", timepos.sec , timepos.nsec);
        ROS_INFO(" ");

        myfile << pez << "\t\t\t" << timeTapDown.sec << "." << timeTapDown.nsec << "\t\t" << timeTapUp.sec << "." << timeTapUp.nsec << "\t\t" << timepos.sec << "." << timepos.nsec << "\t\t" << pe << "\n";

        pnumb.data = randpz[k];
        pez = randpz[k];
        ROS_INFO("Inserisci pezzo: %i", pnumb.data);
        //starttime = ros::Time::now();
        PieceNumber.publish(pnumb);
        send_next_piece = false;
        k++;
    }

    if (j == 0 && send_next_piece == true)
    {
        pnumb.data = randpz[0];
        pez = randpz[0];
        ROS_INFO("Inserisci pezzo: %i", pnumb.data);
        starttime = ros::Time::now();
        PieceNumber.publish(pnumb);
        send_next_piece = false;
        j=1;
    }
    ros::spinOnce();
}