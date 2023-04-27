
#include "mycobot_main.h"
#include "mycobot_socket.h"

#include "DATA.Struct.pb.h"


#include<iostream>


#define DEBUG /*调试模式*/


int main(int argc, const char* args[])
{
	int fd_car, fd_sensor, fd_network;

	int fd_arm;

	unsigned char tempbuff[1024];/*临时缓存*/
	unsigned char databuff[18];

	int recv_long = 0;



	int judge = 30;

	if ((fd_arm = mysys_arm_init()) < 0)
	{
		return -1;//机械臂设备初始化失败，系统直接退出
	}
	
	
	//if ((fd_sensor = mysys_sensor_init()) < 0)
	//{
	//	return -1;//传感器设备初始化失败，系统直接退出
	//}

	/*
	if ((fd_car = mysys_car_init()) < 0)
	{
		return -1;//机械臂设备初始化失败，系统直接退出
	}
	*/
	/*
	if (fd_network = mysys_network_init() < 0)
	{
		return -1;
	}
	*/

	/*机械臂复位*/
	mysys_resetMycobot(fd_arm);





	int server_socket_fd = -1;//服务器网络文件描述符



	PDataPack_RecvQUEUE pdatapack_recvqueue = NULL;/*数据包接收队列*/



	pthread_t recv_threadID = -1;/*网络接收线程ID */


	MYSYSY_Thread_ARG mysys_datarecvthread_arg;/*这个参数传递到子线程*/

	int client_fd = -1;/*存储客户端文件描述符*/
	struct sockaddr_in* addr = (struct sockaddr_in*)malloc(sizeof(struct sockaddr_in));/*存储客户端网络地址*/
	socklen_t addr_len = (socklen_t)sizeof(*addr);
	memset(addr, 0, sizeof(*addr));



	/*
	初始化TCP 服务器
	队头指向队尾指向一个新开辟的地址空间
	队头指向空
	队头的内容指向空
	*/
	if (mysys_initSocket(&server_socket_fd, &pdatapack_recvqueue) != 0)
	{
		free(addr);
		return -1;
	}


	//printf("accept start..\n");
	client_fd = accept(server_socket_fd, (struct sockaddr*)addr, &addr_len);/*阻塞接收客户端*/
	//printf("accpet end..\n");
	if (client_fd < 0)
	{
		perror("Socket accept failed！\n");
		close(server_socket_fd);/*关闭服务器Socke*/
		free(addr);
		mysys_freeDataPack_recvQ(pdatapack_recvqueue);/*释放接收队列*/
		return -1;
	}
	else {
		printf("收到来自%s:%d 客户端的连接...\n", inet_ntoa(addr->sin_addr), ntohs(addr->sin_port));
		//printf("printf end..\n");
	}

	mysys_datarecvthread_arg.fd = client_fd;
	mysys_datarecvthread_arg.pdatapack_recvqueue = pdatapack_recvqueue;





	/*
	阻塞读取数据
	验证START包
	启动通信，打开接收队列的锁
	*/

	printf("aaaaaaaaaaaaaaaaaa\n");
	//aaa(client_fd, pdatapack_recvqueue);
	printf("qqqqqqqqqqqqqqqqqq\n");


	while (true)
	{
		recv_long = recv(client_fd, tempbuff, 1024, 0);/*阻塞读取数据*/

		//memset(databuff, 0, );

		/*
		printf("收到数据：%c\n", tempbuff[0]);
		printf("收到数据：%c\n", tempbuff[1]);
		printf("收到数据：%c\n", tempbuff[2]);
		printf("收到数据：%c\n", tempbuff[3]);
		printf("收到数据：%c\n", tempbuff[4]);
		printf("收到数据：%c\n", tempbuff[5]);
		printf("收到数据：%c\n", tempbuff[6]);
		printf("收到数据：%c\n", tempbuff[7]);
		*/

		printf("recv_long = : %d\n", recv_long);

		for (int j = 0; j < recv_long; j++)
		{
			databuff[j] = tempbuff[j];
			printf("%02X ", databuff[j]);
		}
		printf("\n");

		write(fd_arm, databuff, recv_long);



		/*
		databuff[0] = 0xFE;
		//databuff[1] = 'F'*16+'E';

		if ('F' == 46) {
			printf("F = 46\n");
		}

		a = 'F' - '7';
		printf("%d", a);

		printf("\n9 = %c\n", '9');
		printf("databuff = : %d\n", databuff[0]);
		printf("databuff = : %d\n", databuff[1]);
		printf("databuff = : %d\n", 'F' - '7');
		printf("databuff = : %d\n", '0' + '1');
		*/


		/*
		judge--;

		if (judge <= 0)
		{
			break;
		}
		*/

	}













































































	/*
	上位机收到START帧后，会保持监听，以判断下一步进入什么模式
	*/

	/*
	下位机发送START帧后，会根据选择进入不同的模式，在进入的时候向上发送signal
	*/

	//while (mysys_menu_(fd_arm, fd_car, fd_sensor, client_fd) > -1)/*返回-1  正常退出 */
	//{
		//continue;
	//}

	/*数据包处理器：-》摘包，解析，执行  回传返回值*/
	//mysys_socketDataPackageHandle(robot_fd, client_fd, pdatapack_recvqueue);


	//close(client_fd);
	close(server_socket_fd);
	free(addr);
	mysys_freeDataPack_recvQ(pdatapack_recvqueue);

	mysys_exit_(fd_arm);/*关闭机械臂设备*/
	mysys_exit_(fd_sensor);
	return 0;
}





int mysys_menu_(int fd_arm, int fd_car, int fd_sensor, int client_fd)/*系统菜单与导航*/
{


	time_t timep;

	int ch, ret;

	DragAndDrop_DATA aa;
	Sensor_DATA s_d;

	unsigned char tmp_buf[512];
	unsigned char* recvbuff = tmp_buf;

	unsigned char tempbuff[1024];/*临时缓存*/
	unsigned char databuff[18];

	int int_recvbuff[10];

	unsigned char writebuff[18];


	int recv_long;/*阻塞读取数据*/
	int i = 0;
	int j = 0;
	int k = 0;
	int data_time = 0;//读取数据的次数

	int sp;
	int x_high, x_low, x;
	int y_high, y_low, y;
	int z_high, z_low, z;
	int rx_high, rx_low, rx;
	int ry_high, ry_low, ry;
	int rz_high, rz_low, rz;

	std::string strPb;


	//// 12 begin

	int fd_txt_1 = 0;/*打开文件用于存储*/
	struct tm* newtime;
	char tmpbuf_1[512];
	time_t lt1;






	DragAndDrop_DATA bb;



	/////12 end





	printf("\n\tmyCobot “ 坐标 ” 控制系统！\n\n\n");
	printf("\t\t\t1、数据上行伺教模式\n");
	printf("\t\t\t2、数据下行控制模式\n");
	printf("\t\t\t3、机械臂复位\n");
	printf("\t\t\t4、输出传感器数据\n");
	printf("\t\t\t5、protobuf_test ... \n");



	//printf("\t\t\t6、机械臂向上发送所有舵机的当前角度值 \n");
	//printf("\t\t\t7、机械臂向上发送夹角的当前角度值 \n");
	//printf("\t\t\t8、机械臂接收数据，并调整舵机角度 \n");
	//printf("\t\t\t9、机械臂接收数据，并调整夹角角度 \n");


	printf("\t\t\t6、直行 5s 后停止\n");
	printf("\t\t\t7、右拐 90 度\n");
	printf("\t\t\t8、左拐 90 度\n");
	printf("\t\t\t9、停止\n");
	printf("\t\t\t10、机械臂测试\n");
	printf("\t\t\t11、机械臂  +   底盘车\n");
	printf("\t\t\t12、尝试与上位机数据传输 \n");
	/*printf("\t%-30s\t\t%s\n", "1、本地拖拽伺教模式", "2、视觉控制模式（请遵循视觉控制协议）(NULL)");
	printf("\t%-30s\t%s\n","3、末端位置控制模式(NULL)","4、网络控制模式（按照通信API_v2.0.docx）");
	printf("\t%-30s\t%s\n","5、机械臂复位","6、机械臂进入自由模式");*/
	printf("\t-1、退出系统……\n\n");
	printf("\t请选择功能:");
	scanf("%d", &ch);

	unsigned char tmpbuf[1024 * 10];
	unsigned char* ss;
	int fd_txt = 0;
	int a = 0;

	switch (ch)
	{
	case 1:
		//数据上行伺教模式
		/*开始信号：0xff 0xff 0x0a 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x0a 0xfa*/


		/*
		开始伺教
		1、循环读角度，获取到数据文件fd_txt
		2、将fd_txt发回上位机
		3、发送结束信号
		4、返回(void *) 0
		*/
		aa = dragAndDropTeach_Mode(fd_arm);


		fd_txt = open(aa.datafile, O_RDONLY);/*打开文件，只读*/
		if (fd_txt < 0)
		{
			printf("文件%s打开失败！\n", aa.datafile);
			return 0;
		}


		printf("数据文件为:");
		memset(tmpbuf, 0, 1024 * 10);
		ret = read(fd_txt, tmpbuf, 1024 * 10);/*六个舵机，12字节 假设一次性可以正确读取所有角度 且没有出现丢失*/

		for (int i = 0; i < 1024 * 10; i++)
		{
			printf("%02X ", tmpbuf[i]);
			if (i % 17 == 0)
				printf("\n");
		}

		ss = tmpbuf;

		printf("开始写入!  \n");

		write(client_fd, ss, 1024);/*写到网络对端*/

		printf("写入完成！ \n");
		usleep(1000 * 500);

		//drag_Up_DropTeach_Mode(fd,client_fd); 
		break;
	case 2:
		//数据下行控制模式
		//1、阻塞接收文件
		//2、向机械臂写入数据文件


		recv_long = recv(client_fd, tempbuff, 1024, 0);/*阻塞读取数据*/

		//memset(databuff, 0, );


		printf("收到数据：%c\n", tempbuff[0]);
		printf("收到数据：%c\n", tempbuff[1]);

		databuff[0] = 0xFE;
		databuff[1] = 0xFE;
		databuff[2] = 0x10;
		databuff[3] = 0x25;


		for (i = 0; i <= recv_long; i++)
		{
			if (tempbuff[i] == 'F' &&
				tempbuff[i + 1] == 'E' &&
				tempbuff[i + 3] == 'F' &&
				tempbuff[i + 4] == 'E' &&
				tempbuff[i + 54] == 'F' &&
				tempbuff[i + 55] == 'A')
			{
				printf("找到了！\n");

				//if(tempbuff[i] >= 'A' && tempbuff[i]<='F')
					//printf("\ntempbuff[i] = %c,x_high = %d\n", tempbuff[i + 8], tempbuff[i + 8] - '7');
				//else
				//{
					//printf("\ntempbuff[i] = %c,x_high = %d\n", tempbuff[i + 8], tempbuff[i + 8]);
				//}


				databuff[0] = 0xFE;
				databuff[1] = 0xFE;
				databuff[2] = 0x10;
				databuff[3] = 0x25;

				x_high = str_to_int(tempbuff[i + 12]) * 16 + str_to_int(tempbuff[i + 13]);
				x_low = str_to_int(tempbuff[i + 15]) * 16 + str_to_int(tempbuff[i + 16]);

				y_high = str_to_int(tempbuff[i + 18]) * 16 + str_to_int(tempbuff[i + 19]);
				y_low = str_to_int(tempbuff[i + 21]) * 16 + str_to_int(tempbuff[i + 22]);

				z_high = str_to_int(tempbuff[i + 24]) * 16 + str_to_int(tempbuff[i + 25]);
				z_low = str_to_int(tempbuff[i + 27]) * 16 + str_to_int(tempbuff[i + 28]);

				rx_high = str_to_int(tempbuff[i + 30]) * 16 + str_to_int(tempbuff[i + 31]);
				rx_low = str_to_int(tempbuff[i + 33]) * 16 + str_to_int(tempbuff[i + 34]);

				ry_high = str_to_int(tempbuff[i + 36]) * 16 + str_to_int(tempbuff[i + 37]);
				ry_low = str_to_int(tempbuff[i + 39]) * 16 + str_to_int(tempbuff[i + 40]);

				rz_high = str_to_int(tempbuff[i + 42]) * 16 + str_to_int(tempbuff[i + 43]);
				rz_low = str_to_int(tempbuff[i + 45]) * 16 + str_to_int(tempbuff[i + 46]);

				databuff[4] = x_high;
				databuff[5] = x_low;
				databuff[6] = y_high;
				databuff[7] = y_low;
				databuff[8] = z_high;
				databuff[9] = z_low;
				databuff[10] = rx_high;
				databuff[11] = rx_low;
				databuff[12] = ry_high;
				databuff[13] = ry_low;
				databuff[14] = rz_high;
				databuff[15] = rz_low;

				databuff[16] = str_to_int(tempbuff[i + 48]) * 16 + str_to_int(tempbuff[i + 49]);;//sp
				databuff[17] = 0x01;
				databuff[18] = 0xFA;

				for (j = 0; j < 19; j++)
				{
					printf("%02X ", databuff[j]);
				}

				write(fd_arm, databuff, 19);

				usleep(1000 * 500 * 10);




				printf("\nx_high = %d,x_low = %d\n", x_high, x_low);
				printf("\ny_high = %d,y_low = %d\n", y_high, y_low);
				printf("\nz_high = %d,z_low = %d\n", z_high, z_low);
				printf("\nrx_high = %d,rx_low = %d\n", rx_high, rx_low);
				printf("\nry_high = %d,ry_low = %d\n", ry_high, ry_low);
				printf("\nrz_high = %d,rz_low = %d\n", rz_high, rz_low);


				printf("\ntempbuff[i+0] = %c,x_high = %d\n", tempbuff[i + 0], (tempbuff[i + 0]));
				printf("\ntempbuff[i+1] = %c,x_high = %d\n", tempbuff[i + 1], (tempbuff[i + 1]));

				printf("\ntempbuff[i+3] = %c,x_high = %d\n", tempbuff[i + 3], (tempbuff[i + 3]));
				printf("\ntempbuff[i+4] = %c,x_high = %d\n", tempbuff[i + 4], (tempbuff[i + 4]));

				printf("\ntempbuff[i+6] = %c,x_high = %d\n", tempbuff[i + 6], (tempbuff[i + 6]));
				printf("\ntempbuff[i+7] = %c,x_high = %d\n", tempbuff[i + 7], (tempbuff[i + 7]));

				printf("\ntempbuff[i+9] = %c,x_high = %d\n", tempbuff[i + 9], (tempbuff[i + 9]));
				printf("\ntempbuff[i+10] = %c,x_high = %d\n", tempbuff[i + 10], (tempbuff[i + 10]));

				printf("\ntempbuff[i+12] = %c,x_high = %d\n", tempbuff[i + 12], (tempbuff[i + 12]));
				printf("\ntempbuff[i+13] = %c,x_high = %d\n", tempbuff[i + 13], (tempbuff[i + 13]));

				printf("\ntempbuff[i+15] = %c,x_high = %d\n", tempbuff[i + 15], (tempbuff[i + 15]));
				printf("\ntempbuff[i+16] = %c,x_high = %d\n", tempbuff[i + 16], (tempbuff[i + 16]));

				printf("\ntempbuff[i+18] = %c,x_high = %d\n", tempbuff[i + 18], (tempbuff[i + 18]));
				printf("\ntempbuff[i+19] = %c,x_high = %d\n", tempbuff[i + 19], (tempbuff[i + 19]));

				printf("\ntempbuff[i+21] = %c,x_high = %d\n", tempbuff[i + 21], (tempbuff[i + 21]));
				printf("\ntempbuff[i+22] = %c,x_high = %d\n", tempbuff[i + 22], (tempbuff[i + 22]));
			}
		}





		databuff[0] = 0xFE;
		//databuff[1] = 'F'*16+'E';

		if ('F' == 46) {
			printf("F = 46\n");
		}

		a = 'F' - '7';
		printf("%d", a);

		printf("\n9 = %c\n", '9');
		printf("databuff = : %d\n", databuff[0]);
		printf("databuff = : %d\n", databuff[1]);
		printf("databuff = : %d\n", 'F' - '7');
		printf("databuff = : %d\n", '0' + '1');



		break;
	case 3:
		printf("1_数据包发送完毕！");

		mysys_resetMycobot(fd_arm);

		printf("4_数据包发送完毕！");
		break;
		//case 2:break;
		//case 3:break;
		//case 4:mysys_socketMain(fd);break;
		//case 5:mysys_resetMycobot(fd); break;
		//case 6:mysys_Serial_TX(fd, FREEMODE, NULL); break;
	case 4:
		/*if (recvbuff == NULL)
		{
			return -1;
		}*/





		i = 0, j = 0, ret = 0;
		//memset(recvbuff, 0, 6);//内存初始化


		printf("fd_sensor = %d ！\n", fd_sensor);

		//for(data_time = 0 ; data_time < 5 ; data_time++)
		while (1)
		{


			tcflush(fd_sensor, TCIOFLUSH);         //刷串口清缓存


			memset(recvbuff, 0, 512);//内存初始化
			ret = read(fd_sensor, recvbuff, 18);/*假设可以一次性读完 且没有沾包现象*/
			printf("\n ");
			for (i = 0; i < 10; i++)
			{
				int_recvbuff[i] = (int)recvbuff[i];
				int_recvbuff[i] = int_recvbuff[i] % 16;

				printf("%d ", int_recvbuff[i]);
			}

			s_d.temp_high = int_recvbuff[0] * 10 + int_recvbuff[1];
			s_d.temp_low = int_recvbuff[2] * 10 + int_recvbuff[3];


			s_d.humi_high = int_recvbuff[4] * 10 + int_recvbuff[5];
			s_d.humi_low = int_recvbuff[6] * 10 + int_recvbuff[7];








			tcflush(fd_sensor, TCIOFLUSH);         //刷串口清缓存


			memset(recvbuff, 0, 512);//内存初始化
			ret = read(fd_sensor, recvbuff, 18);/*假设可以一次性读完 且没有沾包现象*/

			printf("\n ");


			for (i = 0; i < 10; i++)
			{
				int_recvbuff[i] = (int)recvbuff[i];
				int_recvbuff[i] = int_recvbuff[i] % 16;
				printf("%d ", int_recvbuff[i]);
			}

			s_d.co = int_recvbuff[0] * 10000 + int_recvbuff[1] * 1000 + int_recvbuff[2] * 100 + int_recvbuff[3] * 10 + int_recvbuff[4];
			s_d.ham_gas = int_recvbuff[5] * 10000 + int_recvbuff[6] * 1000 + int_recvbuff[7] * 100 + int_recvbuff[8] * 10 + int_recvbuff[9];








			time(&timep);
			s_d.p = gmtime(&timep);

			printf("%d-%d-%d %d:%d:%d\n", 1900 + s_d.p->tm_year, 1 + s_d.p->tm_mon, s_d.p->tm_mday, 8 + s_d.p->tm_hour, s_d.p->tm_min, s_d.p->tm_sec);


			printf("temp_high = %d ; temp_low = %d ; humi_high = %d humi_low = %d ; co = %d ; ham_gas = %d \n", s_d.temp_high, s_d.temp_low, s_d.humi_high, s_d.humi_low, s_d.co, s_d.ham_gas);




			//向网络对端发送



			if (ret == -1)
			{
				printf("Error: %s\n", strerror(errno));
			}
			/*在返回数据中寻找开始帧*/
			//i = 0;
			printf("\nret=%d\n", ret);



			usleep(1000);


		}

		break;

	case 5:


		//ProtobufEncode(strPb);  // 序列化后是二进制
		//std::cout << "ProtobufDecode, size: " << strPb.size() << std::endl;
		//ProtobufDecode(strPb);

		break;


	case 6:


		forword_one_step(fd_car, fd_arm);


		break;
	case 7:

		/*

		右转

		*/
		printf("\n11\n");
		displacement(2, fd_car);
		usSleep(7620000);
		displacement(4, fd_car);

		break;
	case 8:

		/*

		左转

		*/
		printf("\n12\n");
		displacement(3, fd_car);
		usSleep(7620000);
		displacement(4, fd_car);

		break;
	case 9:

		/*
		停止
		*/

		displacement(4, fd_car);

		break;
	case 10:

		writebuff[0] = 0xfe;
		writebuff[1] = 0xfe;
		writebuff[2] = 0x0f;
		writebuff[3] = 0x22;


		writebuff[16] = 0x1e;
		writebuff[17] = 0xfa;


		//第一档



		writebuff[4] = 0;//1
		writebuff[5] = 0;

		writebuff[6] = 0;//2
		writebuff[7] = 0;

		writebuff[8] = 0;//3
		writebuff[9] = 0;

		writebuff[10] = 0;//4
		writebuff[11] = 0;

		writebuff[12] = 0;//5
		writebuff[13] = 0;

		writebuff[14] = 0;//6
		writebuff[15] = 0;


		write(fd_arm, writebuff, 18);

		for (i = 0; i < 18; i++)
			printf("%02X ", writebuff[i]);
		printf("\n");

		printf("第一档数据包发送完毕！\n");
		usSleep(3000000);


		//第二档


		writebuff[4] = 0;//1
		writebuff[5] = 0;

		writebuff[6] = 0;//2
		writebuff[7] = 0;

		writebuff[8] = 0;//3
		writebuff[9] = 0;

		writebuff[10] = 0x23;//4
		writebuff[11] = 0x28;

		writebuff[12] = 0;//5
		writebuff[13] = 0;

		writebuff[14] = 0;//6
		writebuff[15] = 0;


		write(fd_arm, writebuff, 18);

		for (i = 0; i < 18; i++)
			printf("%02X ", writebuff[i]);
		printf("\n");

		printf("第二档数据包发送完毕！\n");
		usSleep(3000000);


		//第三档


		writebuff[4] = 0;//1
		writebuff[5] = 0;

		writebuff[6] = 0;//2
		writebuff[7] = 0;

		writebuff[8] = 0x23;//3
		writebuff[9] = 0x28;

		writebuff[10] = 0;//4
		writebuff[11] = 0;

		writebuff[12] = 0;//5
		writebuff[13] = 0;

		writebuff[14] = 0;//6
		writebuff[15] = 0;


		write(fd_arm, writebuff, 18);

		for (i = 0; i < 18; i++)
			printf("%02X ", writebuff[i]);
		printf("\n");

		printf("第三档数据包发送完毕！\n");
		usSleep(3000000);


		//第四档



		writebuff[4] = 0;//1
		writebuff[5] = 0;

		writebuff[6] = 0x23;//2
		writebuff[7] = 0x28;

		writebuff[8] = 0;//3
		writebuff[9] = 0;

		writebuff[10] = 0xdc;//4
		writebuff[11] = 0xd8;

		writebuff[12] = 0;//5
		writebuff[13] = 0;

		writebuff[14] = 0;//6
		writebuff[15] = 0;


		write(fd_arm, writebuff, 18);

		for (i = 0; i < 18; i++)
			printf("%02X ", writebuff[i]);
		printf("\n");

		printf("第四档数据包发送完毕！\n");
		usSleep(3000000);


		//第五档


		writebuff[4] = 0;//1
		writebuff[5] = 0;

		writebuff[6] = 0x23;//2
		writebuff[7] = 0x28;

		writebuff[8] = 0;//3
		writebuff[9] = 0;

		writebuff[10] = 0;//4
		writebuff[11] = 0;

		writebuff[12] = 0;//5
		writebuff[13] = 0;

		writebuff[14] = 0;//6
		writebuff[15] = 0;


		write(fd_arm, writebuff, 18);

		for (i = 0; i < 18; i++)
			printf("%02X ", writebuff[i]);
		printf("\n");

		printf("第五档数据包发送完毕！\n");
		usSleep(3000000);

		mysys_resetMycobot(fd_arm);
		break;

	case 11:

		printf("\n前进两步\n");
		forword_one_step(fd_car, fd_arm);
		forword_one_step(fd_car, fd_arm);

		printf("\n右转\n");
		displacement(2, fd_car);
		usSleep(7640000);
		displacement(4, fd_car);

		forword_one_step(fd_car, fd_arm);

		printf("\n右转\n");
		displacement(2, fd_car);
		usSleep(7640000);
		displacement(4, fd_car);

		printf("\n前进一步\n");
		forword_one_step(fd_car, fd_arm);
		forword_one_step(fd_car, fd_arm);

		break;
	case 12:



		fd_txt_1 = open("sensor_data.txt", O_RDWR | O_CREAT, 0777);/*打开文件，一般来说，新建文件的默认值是0666，新建目录的默认值是0777*/
		if (fd_txt_1 < 0)
		{
			printf("文件%s打开失败！\n", tmpbuf_1);
		}

		i = 0, j = 0, ret = 0;
		printf("fd_sensor = %d ！\n", fd_sensor);


		//往文件里传数据
		bb = sensor_data(fd_sensor, 0 , 1, fd_txt_1);

		close(fd_txt_1);//关闭文件








		//aa = dragAndDropTeach_Mode(fd_arm);


		fd_txt = open("sensor_data.txt", O_RDONLY);/*打开文件，只读*/
		if (fd_txt < 0)
		{
			perror("Error: ");
			printf("文件%s打开失败！\n fd_txt = %d\n", tmpbuf_1,fd_txt);
			//return 0;
		}


		printf("数据文件为:");
		memset(tmpbuf, 0, 1024 * 10);
		ret = read(fd_txt, tmpbuf, 1024 * 10);/*六个舵机，12字节 假设一次性可以正确读取所有角度 且没有出现丢失*/

		for (int i = 0; i < 750; i++)
		{
			//printf("%02X ", tmpbuf[i]);
			printf("%d ", tmpbuf[i]);
			if (i % 17 == 0)
				printf("\n");
		}

		ss = tmpbuf;

		printf("开始写入!  \n");

		write(client_fd, ss, 1024);/*写到网络对端*/

		printf("写入完成！ \n");







		//通过网络传输将文件发上去









		break;
	case -1:return -1;//退出系统
	default: break;
	}
	return ch;
}


DragAndDrop_DATA sensor_data(int fd_sensor, int x, int y,int fd_txt)
{
	time_t timep;

	unsigned char tmp_buf[15];


	unsigned char tmp_buf_1[18];
	unsigned char* recvbuff = tmp_buf_1;

	unsigned char tempbuff[1024];/*临时缓存*/
	//unsigned char databuff[18];

	int int_recvbuff[10];


	Sensor_DATA s_d;
	int sensor_time;
	int gear;//档位
	int i,j,ret;

	DragAndDrop_DATA bb;



	//五个档位
	for (gear = 1; gear <= 5; gear++)
	{
		printf("\n第%d档位!\n", gear);
		//每个档位10条数据
		for (sensor_time = 1; sensor_time <= 10; sensor_time++)
		{

			s_d.mod = gear;
			s_d.x = x;
			s_d.y = y;

			printf("\n第%d条数据!\n", sensor_time);



			tcflush(fd_sensor, TCIOFLUSH);         //刷串口清缓存
			memset(recvbuff, 0, sizeof(recvbuff));//内存初始化
			ret = read(fd_sensor, recvbuff, 18);/*假设可以一次性读完 且没有沾包现象*/
			printf("\n ");
			for (i = 0; i < 10; i++)
			{
				int_recvbuff[i] = (int)recvbuff[i];
				int_recvbuff[i] = int_recvbuff[i] % 16;

				printf("%d ", int_recvbuff[i]);
			}

			s_d.temp_high = int_recvbuff[0] * 10 + int_recvbuff[1];
			s_d.temp_low = int_recvbuff[2] * 10 + int_recvbuff[3];

			s_d.humi_high = int_recvbuff[4] * 10 + int_recvbuff[5];
			s_d.humi_low = int_recvbuff[6] * 10 + int_recvbuff[7];








			tcflush(fd_sensor, TCIOFLUSH);         //刷串口清缓存
			memset(recvbuff, 0, sizeof(recvbuff));//内存初始化
			ret = read(fd_sensor, recvbuff, 18);/*假设可以一次性读完 且没有沾包现象*/
			printf("\n ");
			for (i = 0; i < 10; i++)
			{
				int_recvbuff[i] = (int)recvbuff[i];
				int_recvbuff[i] = int_recvbuff[i] % 16;
				printf("%d ", int_recvbuff[i]);
			}

			s_d.co = int_recvbuff[0] * 10000 + int_recvbuff[1] * 1000 + int_recvbuff[2] * 100 + int_recvbuff[3] * 10 + int_recvbuff[4];
			s_d.ham_gas = int_recvbuff[5] * 10000 + int_recvbuff[6] * 1000 + int_recvbuff[7] * 100 + int_recvbuff[8] * 10 + int_recvbuff[9];

			time(&timep);
			s_d.p = gmtime(&timep);

			//printf("\n%d-%d-%d %d:%d:%d\n", 1900 + s_d.p->tm_year, 1 + s_d.p->tm_mon, s_d.p->tm_mday, 8 + s_d.p->tm_hour, s_d.p->tm_min, s_d.p->tm_sec);
			printf("\n%d-%d-%d %d:%d:%d\n", s_d.p->tm_year, 1 + s_d.p->tm_mon, s_d.p->tm_mday, 8 + s_d.p->tm_hour, s_d.p->tm_min, s_d.p->tm_sec);
			printf("x = %d ; y = %d ; mod = %d ; temp_high = %d ; temp_low = %d  \n", s_d.x, s_d.y, s_d.mod, s_d.temp_high, s_d.temp_low);
			printf("humi_high = %d humi_low = %d ; co = %d ; ham_gas = %d \n", s_d.humi_high, s_d.humi_low, s_d.co, s_d.ham_gas);

			//读到文件中去
			



			tmp_buf[0] = s_d.p->tm_year;
			tmp_buf[1] = 1 + s_d.p->tm_mon;
			tmp_buf[2] = s_d.p->tm_mday;

			tmp_buf[3] = 8 + s_d.p->tm_hour;
			tmp_buf[4] = s_d.p->tm_min;
			tmp_buf[5] = s_d.p->tm_sec;


			tmp_buf[6] = x;
			tmp_buf[7] = y;
			tmp_buf[8] = s_d.mod;//档位

			tmp_buf[9] = s_d.temp_high;//温度高位
			tmp_buf[10] = s_d.temp_low;//温度低位

			tmp_buf[11] = s_d.humi_high;//湿度高位
			tmp_buf[12] = s_d.humi_low;//湿度低位

			tmp_buf[13] = s_d.co;//一氧化碳
			tmp_buf[14] = s_d.ham_gas;//有害气体


			printf("\n写入文件前tmp_buf :\n");
			printf("tm_year = %d ; tm_mon = %d ; tm_mday = %d ; tm_hour = %d ; tm_min = %d ; tm_sec = %d\n", tmp_buf[0], tmp_buf[1], tmp_buf[2], tmp_buf[3], tmp_buf[4], tmp_buf[5]);
			printf("x = %d ; y = %d ; mod = %d ; temp_high = %d ; temp_low = %d\n", tmp_buf[6], tmp_buf[7], tmp_buf[8],tmp_buf[9],tmp_buf[10]);
			printf("humi_high = %d humi_low = %d ; co = %d ; ham_gas = %d \n", tmp_buf[11], tmp_buf[12], tmp_buf[13], tmp_buf[14]);

			write(fd_txt, tmp_buf, 15);/*存储tmpbuff到文件中，只写坐标值*/
			memset(tmp_buf, 0, sizeof(tmp_buf));

			printf("\n写入文件后tmp_buf :");
			printf("tm_year = %d ; tm_mon = %d ; tm_mday = %d ; tm_hour = %d ; tm_min = %d ; tm_sec = %d\n", tmp_buf[0], tmp_buf[1], tmp_buf[2], tmp_buf[3], tmp_buf[4], tmp_buf[5]);
			printf("x = %d ; y = %d ; mod = %d ; temp_high = %d ; temp_low = %d\n", tmp_buf[6], tmp_buf[7], tmp_buf[8], tmp_buf[9], tmp_buf[10]);
			printf("humi_high = %d humi_low = %d ; co = %d ; ham_gas = %d \n", tmp_buf[11], tmp_buf[12], tmp_buf[13], tmp_buf[14]);

			printf("\nret=%d\n", ret);
			usleep(1000);


		}

	}


}







/*sleep in us*/
void usSleep(unsigned int nusecs)
{
	struct timeval    tval;

	tval.tv_sec = nusecs / 1000000;
	tval.tv_usec = nusecs % 1000000;
	select(0, NULL, NULL, NULL, &tval);
}




void forword_one_step(int fd_car, int fd_arm)
{

	unsigned char writebuff[18];

	int i;
	/*
	前进 5s 后停止

	*/

	displacement(1, fd_car);
	usSleep(6000000);
	displacement(4, fd_car);


	/*
		FE FE 0F 22 00 00 00 00 00 00 00 00 00 00 00 00 1E FA

		FE FE 0F 22 00 00 00 00 00 00 DC D8 00 00 00 00 1E FA

		FE FE 0F 22 00 00 00 00 DC D8 00 00 00 00 00 00 1E FA

		FE FE 0F 22 00 00 DC D8 00 00 23 28 00 00 00 00 1E FA

		FE FE 0F 22 00 00 DC D8 00 00 00 00 00 00 00 00 1E FA

	*/




	//采集温度、湿度、气体数据并发送



	writebuff[0] = 0xfe;
	writebuff[1] = 0xfe;
	writebuff[2] = 0x0f;
	writebuff[3] = 0x22;


	writebuff[16] = 0x1e;
	writebuff[17] = 0xfa;


	//第一档



	writebuff[4] = 0;//1
	writebuff[5] = 0;

	writebuff[6] = 0;//2
	writebuff[7] = 0;

	writebuff[8] = 0;//3
	writebuff[9] = 0;

	writebuff[10] = 0;//4
	writebuff[11] = 0;

	writebuff[12] = 0;//5
	writebuff[13] = 0;

	writebuff[14] = 0;//6
	writebuff[15] = 0;


	write(fd_arm, writebuff, 18);

	for (i = 0; i < 18; i++)
		printf("%02X ", writebuff[i]);
	printf("\n");

	printf("第一档数据包发送完毕！\n");
	usSleep(3000000);


	//第二档


	writebuff[4] = 0;//1
	writebuff[5] = 0;

	writebuff[6] = 0;//2
	writebuff[7] = 0;

	writebuff[8] = 0;//3
	writebuff[9] = 0;

	writebuff[10] = 0x23;//4
	writebuff[11] = 0x28;

	writebuff[12] = 0;//5
	writebuff[13] = 0;

	writebuff[14] = 0;//6
	writebuff[15] = 0;


	write(fd_arm, writebuff, 18);

	for (i = 0; i < 18; i++)
		printf("%02X ", writebuff[i]);
	printf("\n");

	printf("第二档数据包发送完毕！\n");
	usSleep(3000000);


	//第三档


	writebuff[4] = 0;//1
	writebuff[5] = 0;

	writebuff[6] = 0;//2
	writebuff[7] = 0;

	writebuff[8] = 0x23;//3
	writebuff[9] = 0x28;

	writebuff[10] = 0;//4
	writebuff[11] = 0;

	writebuff[12] = 0;//5
	writebuff[13] = 0;

	writebuff[14] = 0;//6
	writebuff[15] = 0;


	write(fd_arm, writebuff, 18);

	for (i = 0; i < 18; i++)
		printf("%02X ", writebuff[i]);
	printf("\n");

	printf("第三档数据包发送完毕！\n");
	usSleep(3000000);


	//第四档



	writebuff[4] = 0;//1
	writebuff[5] = 0;

	writebuff[6] = 0x23;//2
	writebuff[7] = 0x28;

	writebuff[8] = 0;//3
	writebuff[9] = 0;

	writebuff[10] = 0xdc;//4
	writebuff[11] = 0xd8;

	writebuff[12] = 0;//5
	writebuff[13] = 0;

	writebuff[14] = 0;//6
	writebuff[15] = 0;


	write(fd_arm, writebuff, 18);

	for (i = 0; i < 18; i++)
		printf("%02X ", writebuff[i]);
	printf("\n");

	printf("第四档数据包发送完毕！\n");
	usSleep(3000000);


	//第五档


	writebuff[4] = 0;//1
	writebuff[5] = 0;

	writebuff[6] = 0x23;//2
	writebuff[7] = 0x28;

	writebuff[8] = 0;//3
	writebuff[9] = 0;

	writebuff[10] = 0;//4
	writebuff[11] = 0;

	writebuff[12] = 0;//5
	writebuff[13] = 0;

	writebuff[14] = 0;//6
	writebuff[15] = 0;


	write(fd_arm, writebuff, 18);

	for (i = 0; i < 18; i++)
		printf("%02X ", writebuff[i]);
	printf("\n");

	printf("第五档数据包发送完毕！\n");
	usSleep(3000000);

	mysys_resetMycobot(fd_arm);
}





int displacement(int jud, int fd_car)
{

	unsigned char writebuff[32] = { 0 };

	writebuff[0] = 0xff;
	writebuff[1] = 0xfe;

	int i = 0;



	switch (jud)
	{

	case 1:
		writebuff[2] = 0x17;
		writebuff[3] = 0x17;
		break;
	case 2:
		writebuff[2] = 0;
		writebuff[3] = 0x17;
		break;
	case 3:
		writebuff[2] = 0x17;
		writebuff[3] = 0;
		break;
	case 4:
		writebuff[2] = 0;
		writebuff[3] = 0;
		break;
	default:
		break;
	}



	writebuff[4] = 0;
	writebuff[5] = 0;

	writebuff[6] = 0;
	writebuff[7] = 0;
	writebuff[8] = 0;
	writebuff[9] = 0;


	write(fd_car, writebuff, 10);




	printf("Jetson Serial Sent to Robot:");

	for (i = 0; i < (10); i++)
		printf("%02X ", writebuff[i]);
	printf("\n");





}






















int str_to_int(char x) {

	if (x >= 'A' && x <= 'F')
	{
		return (int)x - 55;
	}
	else
	{
		return (int)x - 48;
	}
}


/*
开始伺教
1、循环读角度，获取到数据文件fd_txt
2、将fd_txt发回上位机
3、发送结束信号
4、返回(void *) 0
*/
void drag_Up_DropTeach_Mode(int fd, int client_fd) {

	char* tmpbuf;

	char recvbuff[32], ch;
	DragAndDrop_DATA draganddrop_data;/*相关数据结构体*/


	/*开始信号：0xff 0xff 0x0a 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x0a 0xfa*/
	unsigned char* mode_1;
	mode_1 = (unsigned char*)"0xff 0xff 0x0a 0x0a 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xfa";


	//pthread_t ptid, ptid2;/*储存子线程的ID ，创建子线程读取串口 同时存到文件中*/
	mysys_resetMycobot(fd);/*初始化机械臂*/
	draganddrop_data.fd = fd;

	/*将读取到的数据放到fd_txt文件里*/
	int fd_txt = 0;
	//int fd_txt = mysys_DumpSerial(&draganddrop_data);		

	printf("请输入xxx结束:");
	//scanf("%s",);
	while (1)
	{
		memset(recvbuff, 0, 32);
		fgets(recvbuff, 32, stdin);
		if (strstr(recvbuff, "xxx") != NULL)
			break;
	}
	printf("饲教结束...");

	usleep(1000 * 500);




	/*
	for (i = 1; i < 11; i += 2) {
		tembuf[i] = " ";
	}
	for (i = 8; i < 7; i += 2)
	{
		tembuf[i] = 0x00;
	}
	tembuf[0] = tembuf[2] = 0xff;
	tembuf[4] = tembuf[6] = 0x0a;

	printf("tembuf = :  ")
		for (i = 0; i < sizeof(tembuf); i++)
		{
			print("%c ", tembuf);
		}
	printf("\n");
	*/



	/*向上发送数据文件fd_txt*/
	memset(tmpbuf, 0, 1024 * 20);

	strcat(tmpbuf, "0xff 0xff 0x0a 0x0a 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xfa");
	printf("tmpbuf = : %s \n", tmpbuf);

	fd_txt = open(draganddrop_data.datafile, O_RDONLY);
	int retlen = read(fd_txt, tmpbuf + 23, 1024 * 20);/*六个舵机，12字节 假设一次性可以正确读取所有坐标 且没有出现丢失*/
	printf("读取到的字符串长度为：%d \n", retlen);
	printf("tmpbuf = : %s \n", tmpbuf);


	strcat(tmpbuf, "0xff 0xff 0x0a 0x0a 0x0b 0x0b 0x00 0x00 0x00 0x00 0x00 0xfa");
	printf("tmpbuf = : %s \n", tmpbuf);


	write(client_fd, tmpbuf, 1024 * 20);/*写到网络对端*/
	printf("机械臂执行完毕！");
	usleep(1000 * 500);

	return;
}

int data_Down_Mode(int robotfd) {

}
void signal_Test_Mode(int robotfd, int client_fd) {


	const char* tmpbuf = "dsadsadssads";

	printf("2_数据包发送完毕！");
	write(client_fd, tmpbuf, 20);/*写到网络对端*/
	printf("3_数据包发送完毕！");
	usleep(1000 * 500);

	return;
}




int post_All_Coord(int fd, unsigned char databuff[], unsigned char sp)
{
	databuff[12] = 0x1e;/*速度*/
	if (sp != 0)
	{
		databuff[12] = sp;/*设置速度值*/
	}
	mysys_Serial_TX(fd, POST_ALL_ANGLE, databuff);
	return 0;
}





void mysys_code_mode(int fd_main, int argc, const char* args[])/*无界面模式，未开发*/
{

	switch (atoi(args[2]))
	{
	case 1:dragAndDropTeach_Mode(fd_main); break;
	case 2:break;
	case 3:break;
	case 4:break;
	case -1:return;
	default: break;
	}
}
/*右臂自适应，仿真右臂的动作，即在源数据的基础上，给1-5号舵机反向处理，在发送之前和接收时候调用，将所有的舵机角度数组给我即可
databuff中存储的六个舵机的数据值 共计12个字节，可根据实际修改
*/
void mysys_RightArmAutoRealy(unsigned char* databuff)
{
	int i = 0;
	int new_angle = 0, temp = 0;//解析到原始的角度在取反后
	if (databuff == NULL)return;
	for (i = 1; i <= 5; i++)//只处理1-5号机位
	{

		/*
		joint_no取值范围: 1~6
		angle_high：数据类型byte
		计算方式：角度值乘以100 先转换成int形式 再取十六进制的高字节
		angle_low：数据类型byte
		计算方式：角度值乘以100 先转换成int形式 再取十六进制的低字节

		说明：上述的计算方式，是顺时针角度（从舵机的视角来说，3号舵机反向）；
		若设置逆时针角度，角度值乘100，然后用65536减去。转换成int形式 再取十六进制的高字节、低字节。

		如何得出关节最大角度
		temp = angle1_high*256 + angle1_low
		Angle1=（temp \ 33000 ?(temp – 65536) : temp）/10
		计算方式：角度值低位 + 角度高位值乘以256
		先判断是否大于33000 如果大于33000就再减去65536 最后除以10 如果小于33000就直接除以10
		*/
		temp = databuff[(i - 1) * 2] * 0x100 + databuff[(i - 1) * 2 + 1];//0x100 = 256
		new_angle = ((temp > 33000 ? (temp - 65536) : temp) / 100) * -1;//新角度值

		databuff[(i - 1) * 2] = (unsigned char)((((new_angle * 100) + 65536) % 65536) / 0x100);//新角度数据值的高字节
		databuff[(i - 1) * 2 + 1] = (unsigned char)((((new_angle * 100) + 65536) % 65536) % 0x100);//新角度数据值的低字节
	}
	return;
}

/*串口发送  ComuType为类型  databuff中存储数据*/
int mysys_Serial_TX(int fd, ComuType comutype, unsigned char databuff[])
{

	int ret = 0, i;
	unsigned char writebuff[32] = { 0 };

	writebuff[0] = writebuff[1] = FLAG_START;
	switch (comutype)
	{
		/*设置指令帧*/
	case FREEMODE:
		/*自由模式*/
		writebuff[2] = 0x02;/*数据长度帧*/
		writebuff[3] = comutype;
		break;
	case READANGLE:
		/*读取所有机械臂舵机角度*/
		writebuff[2] = 0x02;
		writebuff[3] = comutype;
		break;
	case POST_ALL_ANGLE:
		/*发送全部角度*/
		writebuff[2] = 0x0f;
		writebuff[3] = comutype;
		mysys_RightArmAutoRealy(databuff);/*右臂角度自适应*/
		memcpy(writebuff + 4, databuff, 13);
		break;
	case READ_Gripper_ANGLE:
		/*读取夹爪角度*/
		writebuff[2] = 0x02;/*数据长度帧，按照机械臂的协议来*/
		writebuff[3] = comutype;
		break;
	case SET_Gripper_Mode:
		/*设置夹爪模式与设置夹爪角度相同*/
	case SET_Gripper_ANGLE:
		/*设置夹爪角度*/
		writebuff[2] = 0x04;/*数据长度帧*/
		writebuff[3] = comutype;
		memcpy(writebuff + 4, databuff, 2);
		break;




		/*读取当前位置坐标*/
	case READ_ALL_COORD:
		writebuff[2] = 0x02;
		writebuff[3] = comutype;
		break;
		/*设置位置坐标*/

	case SET_ALL_COORD:
		writebuff[2] = 0x10;
		writebuff[3] = comutype;
		memcpy(writebuff + 4, databuff, 14);
		break;
	default:
		printf("发送类型未定义！\n");
		return -1;
	}
	writebuff[2 + writebuff[2]] = FLAG_END;/*根据协议中的数据长度帧填充结束帧*/
	write(fd, writebuff, (int)(writebuff[2] + 3));




	printf("Jetson Serial Sent to Robot:");
	if (writebuff[3] != SET_ALL_COORD)
	{
		for (i = 0; i < (writebuff[2] + 3); i++)
			printf("%02X ", writebuff[i]);
		printf("\n");
	}
	else {
		for (i = 0; i < 16; i++)
			printf("%02X ", writebuff[i]);
		printf("\n");
	}
	return ret;
}


/*网络控制模式
fd：机械臂文件描述符
线程（Main）：
	1、初始化服务器Socket()；
	2、阻塞监听客户端连接
	3、创建子线程负责网络通信
	4、从链式队列摘除数据包，
	5、解析、执行数据包
	6、回传执行状态数据包

子线程：
	1、监听数据，将每个数据包验证后入队；
	2、入队后，回传所接收数据包的确认
*/

int mysys_socketMain(int robot_fd)
{

	return 0;
}

/*释放数据包接收队列*/
void mysys_freeDataPack_recvQ(PDataPack_RecvQUEUE pdatapack_recvqueue)
{
	if (pdatapack_recvqueue == NULL)/*队列不存在*/
	{
		return;
	}
	else if (pdatapack_recvqueue->front == pdatapack_recvqueue->rear)/*数据包队列为空*/
	{
		free(pdatapack_recvqueue->front);
		pdatapack_recvqueue->front = pdatapack_recvqueue->rear = NULL;
		free(pdatapack_recvqueue);
		pdatapack_recvqueue = NULL;
		return;
	}
	else {
		/*数据包队列不为空，需要一个个节点释放*/
		PDataPack_Node pdatapacknode = NULL;
		while (pdatapack_recvqueue->front != NULL)
		{
			pdatapacknode = pdatapack_recvqueue->front;
			pdatapack_recvqueue->front = pdatapack_recvqueue->front->next;
			if (pdatapacknode->datapack != NULL)
				free(pdatapacknode->datapack);/*释放数据包节点中的数据包*/
			free(pdatapacknode);/*释放数据节点*/
			pdatapacknode = NULL;
		}
		free(pdatapack_recvqueue);/*释放数据包接收队列*/
		pdatapack_recvqueue = NULL;
		return;
	}

}

/*复位机械臂*/
int mysys_resetMycobot(int fd)
{
	unsigned char databuff[16] = { 0 };
	memset(databuff, 0, 16);
	databuff[12] = 0x1e;/*速度*/
	mysys_Serial_TX(fd, POST_ALL_ANGLE, databuff);
#ifdef DEBUG
	printf("机械臂复位成功！\n");
#endif
	return 0;
}

/*发送全部角度给机械臂，不包括机械爪*/
int mysys_PostAllAngle(int fd, unsigned char databuff[], unsigned char sp)
{
	databuff[12] = 0x1e;/*速度*/
	if (sp != 0)
	{
		databuff[12] = sp;/*设置速度值*/
	}
	mysys_Serial_TX(fd, POST_ALL_ANGLE, databuff);
	return 0;

}

/*线程销毁处理函数*/
void mysys_cleanhandle(void* argc)
{
	int fd_txt = *((int*)argc);

	if (!close(fd_txt))
	{
		printf("数据文件关闭成功！\n");
	}
	else
		printf("数据文件关闭失败！\n");
}


void* mysys_DumpSerial(void* p_draganddrop_data)/*拖拽伺教模式中的读取串口子线程*/
{
	/*DragAndDrop_DATA draganddrop_data;相关数据结构体*/
	int fd = ((DragAndDrop_DATA*)p_draganddrop_data)->fd;/*串口文件的文件描述符*/
	int ret, i;
	int fd_txt = 0;/*打开文件用于存储*/
	struct tm* newtime;
	char tmpbuf[512];
	time_t lt1;
	time(&lt1);/*获取时间*/
	newtime = localtime(&lt1);
	strftime(tmpbuf, 64, "%F-%X", newtime);/*格式化时间(2022-01-20 23:41:58)到tmpbuf*/
	strcat(tmpbuf, ".txt");//新建.txt文件用来存放当前时间
	strcpy(((DragAndDrop_DATA*)p_draganddrop_data)->datafile, tmpbuf);/*拷贝文件名到结构体中，便于主线程可以获取*/


	pthread_detach(pthread_self());/*线程自我分离，分离之后子线程是在创建的时候，就独立于父线程，其死亡时候资源也是由自己来释放*/
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);/*线程的取消类型设置为立即取消*/
	fd_txt = open(tmpbuf, O_RDWR | O_CREAT, 0777);/*打开文件，一般来说，新建文件的默认值是0666，新建目录的默认值是0777*/
	if (fd_txt < 0)
	{
		printf("文件%s打开失败！\n", tmpbuf);
		return (void*)0;
	}
	/*发送指令到机械臂，进入自由模式指令*/
	sleep(3);
	if (mysys_Serial_TX(fd, FREEMODE, NULL) < 0)
	{
		/*发送失败*/
		printf("进入自由模式失败！\n");
		close(fd_txt);
		return (void*)0;
	}
	printf("已经进入伺教模式...请拖拽电机\n");
	/*循环采集机械臂数据并存储*/
	pthread_cleanup_push(mysys_cleanhandle, (void*)(&fd_txt));/*注册销毁函数*/
	while (1)
	{

		//读取所有的机械臂坐标
		/*
			#define READANGLE 0x20 读取所有舵机的角度，不包括机械爪

			#define POST_ALL_ANGLE 0x22 发送全部角度

			#define READ_Gripper_ANGLE 0x65 读取夹爪角度

			#define SET_Gripper_Mode 0x66 设置夹爪模式，张开和闭合两种模式

			#define SET_Gripper_ANGLE 0x67 设置夹爪角度
		*/
		mysys_Serial_TX(fd, READANGLE, NULL);/*读取所有机械臂坐标*/
		if (mysys_Serial_RX(fd, (unsigned char*)tmpbuf) > 0)/*接收机械臂的数据到tmpbuf,包括了帧头帧尾什么的 */
		{
			//mysys_Serial_TX(fd, READANGLE, NULL);/*读取所有机械臂坐标*/
			write(fd_txt, tmpbuf, 17);/*存储tmpbuff到文件中，只写坐标值*/
		}





		usleep(1000 * 500);/*500ms*/
	}
	pthread_cleanup_pop(1);/*销毁清理函数*/
	return (void*)0;
}








/*接收机械臂串口数据的接收函数,recvbuff需要提供512空间的大小，数据将存储在此，返回值为接收数据的长度，接收失败返回-1*/
int mysys_Serial_RX(int robot_fd, unsigned char* recvbuff)
{
	if (recvbuff == NULL)
	{
		return -1;
	}
	int i = 0, j = 0, ret;
	memset(recvbuff, 0, 512);//内存初始化
	ret = read(robot_fd, recvbuff, 512);/*假设可以一次性读完 且没有沾包现象*/

	printf("读取到%d个字节的返回数据！\n", ret);

	/*在返回数据中寻找开始帧*/
	i = 0;
	if (ret < 0)return -1;
	while (i < ret - 1)
	{
		for (i; i < ret - 1; i++)/*寻找包头*/
		{
			/*假设找对包头*/
			if ((recvbuff[i] == recvbuff[i + 1]) && (recvbuff[i] == 0xfe))
				break;
		}
		if (i < ret - 1)/*找到帧头*/
		{
#ifdef DEBUG
			printf("找到帧头！\n");
#endif
			/*验证下包尾*/
			if (recvbuff[i + 2 + recvbuff[i + 2]] == 0xfa)
			{
#ifdef DEBUG
				printf("收到机械臂有效串口数据为:");
				for (j = 0; j < 3 + recvbuff[i + 2]; j++)
					printf("%02X ", recvbuff[i + j]);
				printf("\n");
#endif
				/*将数据放在数组的最前端*/
				int recvlen = 3 + recvbuff[i + 2];/*有效返回的长度，包括了帧头，帧尾*/
				for (j = 0; j < recvlen; j++, i++)
				{
					recvbuff[j] = recvbuff[i];
				}
				/*这里要做右臂的自适应*/
				/*判断是不是读取所有角度*/
				if (recvbuff[3] == READANGLE)
					mysys_RightArmAutoRealy(recvbuff + 4);/*右臂角度自适应*/

				return recvlen;/*接收成功,返回接收有效数据的长度，包括帧头和帧尾*/
			}
			else {
				continue;/*继续找*/
			}
		}
		else {
			/*没有包头*/
			return -1;
		}
	}
}

/*拖拽伺教模式中的机械臂演示执行子线程*/
void* mysys_teachMode_Play(void* p_draganddrop_data)
{
	/*DragAndDrop_DATA draganddrop_data;相关数据结构体*/
	int fd = ((DragAndDrop_DATA*)p_draganddrop_data)->fd;/*串口的文件描述符*/
	int ret, i;
	int fd_txt = 0;/*打开文件用于读取*/
	char tmpbuf[16];

	pthread_detach(pthread_self());/*线程自我分离*/
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);/*线程的取消类型设置为立即取消*/
	fd_txt = open(((DragAndDrop_DATA*)p_draganddrop_data)->datafile, O_RDONLY);/*打开文件，只读*/
	if (fd_txt < 0)
	{
		printf("文件%s打开失败！\n", ((DragAndDrop_DATA*)p_draganddrop_data)->datafile);
		return (void*)0;
	}

	printf("已经进入伺教执行模式...请注意远离机械臂\n");
	usleep(1000 * 40);
	/*循环读取文件数据并控制机械臂执行*/
	pthread_cleanup_push(mysys_cleanhandle, (void*)(&fd_txt));/*注册销毁函数*/
#ifdef _DEBUG
	while (1)
	{
		printf("数据文件为:");
		memset(tmpbuf, 0, 16);
		ret = read(fd_txt, tmpbuf, 12);/*六个舵机，12字节 假设一次性可以正确读取所有角度 且没有出现丢失*/

		for (i = 0; i < 12; i++)
			printf("%02X ", tmpbuf[i]);
		printf("\n");
		if (ret < 12)break;
	}
#endif
	while (1)
	{

		memset(tmpbuf, 0, 16);
		ret = read(fd_txt, tmpbuf, 12);/*六个舵机，12字节 假设一次性可以正确读取所有角度 且没有出现丢失*/
#ifdef DEBUG
		printf("从文件中读取到%d个字节的舵机数据！\n", ret);
#endif
		if (ret < 12)break;
		mysys_PostAllAngle(fd, (unsigned char*)tmpbuf, 0);/*控制机械臂,发送全部角度给机械臂，速度值默认*/
		usleep(1000 * 500);/*500ms*/
	}
	printf("机械臂执行完毕！请键入xxx结束执行模式.\n");
	pthread_cleanup_pop(1);/*销毁清理函数*/
	return (void*)0;
}

/*拖拽伺教模式*/
DragAndDrop_DATA dragAndDropTeach_Mode(int fd)
{
	char recvbuff[32], ch;
	char tmpbuf[16];
	int ret, i;
	DragAndDrop_DATA draganddrop_data;/*相关数据结构体*/
	pthread_t ptid, ptid2;/*储存子线程的ID ，创建子线程读取串口 同时存到文件中*/
	mysys_resetMycobot(fd);/*初始化机械臂*/
	draganddrop_data.fd = fd;
	//启用多线程读取串口并存储文件；*/
	pthread_create(&ptid, NULL, mysys_DumpSerial, (void*)(&draganddrop_data));/*属性默认，启动函数为mysys_DumpSerial 将fd作为参数*/
	//主线程等待用户结束并结束子线程；	
	printf("请输入xxx结束:");
	//scanf("%s",);
	while (1)
	{
		memset(recvbuff, 0, 32);
		fgets(recvbuff, 32, stdin);
		if (strstr(recvbuff, "xxx") != NULL)
			break;
	}
	/*终止子线程*/
	if (pthread_cancel(ptid) == 0)	printf("向子线程发送取消请求成功。\n");


	int fd_txt = open(draganddrop_data.datafile, O_RDONLY);/*打开文件，只读*/
	if (fd_txt < 0)
	{
		printf("文件%s打开失败！\n", draganddrop_data.datafile);
		return draganddrop_data;
	}

	while (1)
	{
		printf("数据文件为:");
		memset(tmpbuf, 0, 16);
		ret = read(fd_txt, tmpbuf, 17);/*六个舵机，12字节 假设一次性可以正确读取所有角度 且没有出现丢失*/

		for (i = 0; i < 17; i++)
			printf("%02X ", tmpbuf[i]);
		printf("\n");
		if (ret < 12)break;
	}

	printf("饲教结束，是否向上发送执行（y|Y play，other exit）：");
	ch = getchar();
	if ((ch == 'y') || (ch == 'Y'))
	{
		/*
			向上位机发送数据文件

		*/


		//pthread_create(&ptid2, NULL, ,(void *)(&draganddrop_data));/*属性默认，启动函数为mysys_teachMode_Play 将draganddrop_data作为参数*/








		/*启动新线程执行*/
		//pthread_create(&ptid2, NULL, mysys_teachMode_Play,(void *)(&draganddrop_data));/*属性默认，启动函数为mysys_teachMode_Play 将draganddrop_data作为参数*/
		//printf("请输入xxx结束演示:");
		//scanf("%s",);
		//while(1)
		//{
			//memset(recvbuff, 0, 32);
			//fgets(recvbuff,32,stdin);
			//if(strstr(recvbuff,"xxx")!=NULL)
			//	break;
		//}
		//如果成功，pthread_cancel返回0。如果不成功，pthread_cancel返回一个非零的错误码
		//if(pthread_cancel(ptid2)==0)	printf("向子线程2发送取消请求成功。\n");
	}
	usleep(1000);
	printf("所有子线程结束\n");
	return draganddrop_data;
}



void mysys_exit_(int fd_device)
{
	close(fd_device);
	printf("系统正常退出！\n");

}










//对传感器设备的初始化，返回值：传感器串口文件描述符；
int mysys_sensor_init()
{
	int fd_device;
	struct termios  newtio;
	tcgetattr(fd_device, &newtio);

	newtio.c_cflag &= ~CSIZE;/*数据位屏蔽 将c_cflag全部清零*/
	newtio.c_cflag = B115200;/*set bound*/
	newtio.c_cflag |= CS8;/*数据位8*/
	newtio.c_cflag |= CLOCAL | CREAD;/*使驱动程序启动接收字符装置，同时忽略串口信号线的状态*/

	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);/*禁用软件流控制*/

	newtio.c_oflag &= ~OPOST;/*使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。*/

	/*在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。要使串口设备工作在原始模式，需要关闭ICANON、ECHO、ECHOE和ISIG选项，其操作方法如下：*/
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);/*在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。*/

	/*4）当VMIN = 0，VTIME = 0时 如果有数据可用，则read最多返回所要求的字节数，如果无数据可用，则read立即返回0。MIN > 0 , TIME =0 READ 会等待,直到MIN字元可读
	*/
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;
	fd_device = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);/*读写方式打开串口*/


	//set UART 
	if (fd_device < 0)
	{
		printf("设备连接失败！fd_sensor = %d\n", fd_device);
		perror("sensor Serial open failed...");
		return -1;
	}
	printf("sensor 设备连接成功 fd_sensor = %d\n", fd_device);

	if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0)
	{
		printf("设备连接初始化失败！\n");
		perror("sensor Serial init failed...");
		return -2;
	}

	printf("sensor 设备连接初始化成功！\n");
	return fd_device;/*返回文件描述符*/
}


//对机械臂串口初始化
int mysys_arm_init()
{
	int fd_device;
	struct termios  newtio;
	tcgetattr(fd_device, &newtio);
	newtio.c_cflag &= ~CSIZE;/*数据位屏蔽 将c_cflag全部清零*/
	newtio.c_cflag = B115200;/*set bound*/
	newtio.c_cflag |= CS8;/*数据位8*/
	newtio.c_cflag |= CLOCAL | CREAD;/*使驱动程序启动接收字符装置，同时忽略串口信号线的状态*/

	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);/*禁用软件流控制*/

	newtio.c_oflag &= ~OPOST;/*使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。*/

	/*在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。要使串口设备工作在原始模式，需要关闭ICANON、ECHO、ECHOE和ISIG选项，其操作方法如下：*/
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);/*在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。*/

	/*4）当VMIN = 0，VTIME = 0时 如果有数据可用，则read最多返回所要求的字节数，如果无数据可用，则read立即返回0。MIN > 0 , TIME =0 READ 会等待,直到MIN字元可读
	*/
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;
	fd_device = open("/dev/ttyUSB0", O_RDWR);/*读写方式打开串口*/
	//set UART 
	if (fd_device < 0)
	{
		printf("设备连接失败！fd_robot_arm = %d\n", fd_device);
		perror("Serial open failed...");
		return -1;
	}
	printf("设备连接成功 fd_robot_arm = %d\n", fd_device);

	if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0)
	{
		printf("设备连接初始化失败！\n");
		perror("Serial init failed...");
		return -2;
	}
	printf("设备连接初始化成功！\n");
	return fd_device;/*返回文件描述符*/
}



//对底盘车串口初始化
int mysys_car_init()
{
	int fd_device;
	struct termios  newtio;
	tcgetattr(fd_device, &newtio);
	newtio.c_cflag &= ~CSIZE;/*数据位屏蔽 将c_cflag全部清零*/
	newtio.c_cflag = B115200;/*set bound*/
	newtio.c_cflag |= CS8;/*数据位8*/
	newtio.c_cflag |= CLOCAL | CREAD;/*使驱动程序启动接收字符装置，同时忽略串口信号线的状态*/

	newtio.c_iflag &= ~(IXON | IXOFF | IXANY);/*禁用软件流控制*/

	newtio.c_oflag &= ~OPOST;/*使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。*/

	/*在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。要使串口设备工作在原始模式，需要关闭ICANON、ECHO、ECHOE和ISIG选项，其操作方法如下：*/
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);/*在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。*/

	/*4）当VMIN = 0，VTIME = 0时 如果有数据可用，则read最多返回所要求的字节数，如果无数据可用，则read立即返回0。MIN > 0 , TIME =0 READ 会等待,直到MIN字元可读
	*/
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;
	fd_device = open("/dev/ttyUSB0", O_RDWR);/*读写方式打开串口*/
	//set UART 
	if (fd_device < 0)
	{
		printf("设备连接失败！fd_car = %d\n", fd_device);
		perror("Serial open failed...");
		return -1;
	}
	printf("设备连接成功 fd_car = %d\n", fd_device);

	if (tcsetattr(fd_device, TCSADRAIN, &newtio) != 0)
	{
		printf("设备连接初始化失败！\n");
		perror("Serial init failed...");
		return -2;
	}
	printf("设备连接初始化成功！\n");
	return fd_device;/*返回文件描述符*/
}








int mysys_network_init()
{
	return -1;
}








