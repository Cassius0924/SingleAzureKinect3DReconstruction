#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

//#include "Face.h"

#include "polygon.pb.h"

#include "Hello.h"

//using namespace my_package;

#include <chrono>
#include <thread>




int main(int argc, char** argv)
{


	/*

	// ∂¡»Îµ„‘∆ ˝æ›
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>("./11/1.pcd", *cloud);
	std::cout << "Loaded " << cloud->width * cloud->height << " data points." << std::endl;

	// Ωµ≤…—˘
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.01f, 0.01f, 0.01f); // …Ë÷√ÃÂÀÿ¥Û–°
	sor.filter(*cloud_filtered);
	std::cout << "Downsampled to " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// º∆À„∑®œﬂ
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(cloud_filtered);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03); // …Ë÷√À—À˜∞Îæ∂
	ne.compute(*normals);
	std::cout << "º∆À„∑®œﬂΩ· ¯ " << std::endl;

	// »˝Ω«ªØ
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::cout << "1 " << std::endl;
	pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);
	std::cout << "2 " << std::endl;
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	std::cout << "3 " << std::endl;
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;



	std::cout << "4 " << std::endl;
	gp3.setInputCloud(cloud_with_normals);
	std::cout << "5 " << std::endl;
	gp3.setSearchMethod(tree2);
	std::cout << "6 " << std::endl;
	gp3.setSearchRadius(0.1); // …Ë÷√À—À˜∞Îæ∂
	std::cout << "7 " << std::endl;
	gp3.setMu(2.5);
	std::cout << "8 " << std::endl;
	gp3.setMaximumNearestNeighbors(100);
	std::cout << "9 " << std::endl;
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	std::cout << "10 " << std::endl;
	gp3.setMinimumAngle(M_PI / 18);
	std::cout << "11 " << std::endl;
	gp3.setMaximumAngle(2 * M_PI / 3);
	std::cout << "12 " << std::endl;
	gp3.setNormalConsistency(false);
	std::cout << "13 " << std::endl;
	pcl::PolygonMesh triangles;
	std::cout << "14 " << std::endl;
	gp3.reconstruct(triangles);
	std::cout << "15 " << std::endl;

	*/

	pcl::PLYReader reader;

	//¥¥Ω® pcl::PolygonMesh ¿‡–Õµƒ∂‡±ﬂ–ŒÕ¯∏Ò ˝æ›£∫
	pcl::PolygonMesh triangles;
	//¥”Œƒº˛÷–º”‘ÿ∂‡±ﬂ–ŒÕ¯∏Ò ˝æ›£∫

	reader.read("./12/nn.ply", triangles);
	//pcl::io::loadPLYFile("./12/nn.ply", mesh);

	// ¥”PolygonMesh÷–ªÒ»°∂•µ„∫Õ—’…´ ˝æ›
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::fromPCLPointCloud2(triangles.cloud, *cloud);






	Hello hi;

	unsigned char tempbuff[1024];
	unsigned char databuff[18];

	int fd_network;
	int server_socket_fd = -1;

	int recv_long = 0;

	pthread_t recv_threadID = -1;

	int client_fd = -1;
	struct sockaddr_in* addr = (struct sockaddr_in*)malloc(sizeof(struct sockaddr_in));
	socklen_t addr_len = (socklen_t)sizeof(*addr);
	memset(addr, 0, sizeof(*addr));

	struct sockaddr_in sockaddr;
	memset(&sockaddr, 0, sizeof(sockaddr));
	sockaddr.sin_family = AF_INET;
	sockaddr.sin_port = htons(5001);

	char ip_local[32 + 1] = { 0 };

	if (hi.get_local_ip(ip_local) != 0)
	{
		printf("ªÒ»°±æµÿIP ß∞‹£°\n");
		return -1;
	}
	inet_aton(ip_local, &sockaddr.sin_addr);

	server_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (server_socket_fd < 0)
	{
		perror("Socket create failed!\n");
		return -1;
	}

	if (bind(server_socket_fd, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) != 0)//∞Û∂®µÿ÷∑∫Õ∂Àø⁄∫≈
	{
		perror("Socket bind failed!\n");
		close(server_socket_fd);//πÿ±’∑˛ŒÒ∆˜socket
		return -1;
	}
	if (listen(server_socket_fd, 1) != 0)//º‡Ã˝øÕªß∂À
	{
		perror("Socket listen failed!\n");
		close(server_socket_fd);
		return -1;
	}










	//printf("accept start..\n");
	client_fd = accept(server_socket_fd, (struct sockaddr*)addr, &addr_len);//◊Ë»˚Ω” ’øÕªß∂À
	//printf("accpet end..\n");
	if (client_fd < 0)
	{
		perror("Socket accept failed£°\n");
		close(server_socket_fd);//πÿ±’∑˛ŒÒ∆˜Socke
		free(addr);
		return -1;
	}
	else {
		printf(" ’µΩ¿¥◊‘%s:%d øÕªß∂Àµƒ¡¨Ω”...\n", inet_ntoa(addr->sin_addr), ntohs(addr->sin_port));
		//printf("printf end..\n");
	}




	printf("aaaaaaaaaaaaaaaaaa\n");

	printf("qqqqqqqqqqqqqqqqqq\n");












	my_package::pg pg_message;

	int  triangles_i = 0;



	// ±È¿˙»˝Ω«–Œ√Ê∆¨
	for (auto it = triangles.polygons.begin(); it != triangles.polygons.end(); ++it) {
		triangles_i++;

		// ªÒ»°µ±«∞»˝Ω«–Œ√Ê∆¨µƒ»˝∏ˆ∂•µ„µƒÀ˜“˝
		int v1_1 = it->vertices[0];
		int v2_2 = it->vertices[1];
		int v3_3 = it->vertices[2];

		// ªÒ»°»˝∏ˆ∂•µ„µƒ◊¯±Í÷µ
		float x1 = cloud->points[v1_1].x;
		float y1 = cloud->points[v1_1].y;
		float z1 = cloud->points[v1_1].z;

		float x2 = cloud->points[v2_2].x;
		float y2 = cloud->points[v2_2].y;
		float z2 = cloud->points[v2_2].z;

		float x3 = cloud->points[v3_3].x;
		float y3 = cloud->points[v3_3].y;
		float z3 = cloud->points[v3_3].z;




		my_package::V1* v1 = pg_message.add_v1();
		v1->set_x(x1);
		v1->set_y(y1);
		v1->set_z(z1);

		my_package::V2* v2 = pg_message.add_v2();
		v2->set_x(x2);
		v2->set_y(y2);
		v2->set_z(z2);

		my_package::V3* v3 = pg_message.add_v3();
		v3->set_x(x3);
		v3->set_y(y3);
		v3->set_z(z3);
		//  ‰≥ˆ»˝∏ˆ∂•µ„µƒ◊¯±Í÷µ∫Õ—’…´


		pcl::PointXYZRGB color;
		color.r = color.g = color.b = 0;
		color.r = (color.r + cloud->points[v1_1].r + cloud->points[v2_2].r + cloud->points[v3_3].r) / 3;
		color.g = (color.g + cloud->points[v1_1].g + cloud->points[v2_2].g + cloud->points[v3_3].g) / 3;
		color.b = (color.b + cloud->points[v1_1].b + cloud->points[v2_2].b + cloud->points[v3_3].b) / 3;




		pg_message.add_r(color.r);
		pg_message.add_g(color.g);
		pg_message.add_b(color.b);


		std::cout << "triangles_i : " << triangles_i << std::endl;
		std::cout << "Vertex 1: (" << x1 << ", " << y1 << ", " << z1 << ") Color: (" << (int)color.r << ", " << (int)color.g << ", " << (int)color.b << ")" << std::endl;
		std::cout << "Vertex 2: (" << x2 << ", " << y2 << ", " << z2 << ") Color: (" << (int)color.r << ", " << (int)color.g << ", " << (int)color.b << ")" << std::endl;
		std::cout << "Vertex 3: (" << x3 << ", " << y3 << ", " << z3 << ") Color: (" << (int)color.r << ", " << (int)color.g << ", " << (int)color.b << ")" << std::endl;





		if (triangles_i % 1 == 0)
		{
			std::string strPb="";


			uint32_t pbSize = pg_message.ByteSize();        // ªÒ»°–Ú¡–ªØ∫Ûµƒ¥Û–°

			std::cout << "2   uint32_t pbSize = pg_message.ByteSize(); = " << pbSize << std::endl;

			//strPb.clear();

			strPb.resize(pbSize);

			uint8_t* szData = (uint8_t*)strPb.c_str();//◊÷∑˚¥Æ◊™÷∏’Î


			pg_message.SerializeToArray(szData, pbSize);
			/*
			std::cout << "aaa" << std::endl;
			if (!pg_message.SerializeToArray(szData, pbSize))   // øΩ±¥–Ú¡–ªØ∫Ûµƒ ˝æ›
			{
				std::cout << "sensor pb msg SerializeToArray failed." << std::endl;
				//return false;
			}
			else
			{
				std::cout << "sensor pb msg SerializeToArray sucess." << std::endl;
			}

			printf("start write!  \n");
			write(client_fd, szData, pbSize);*//*–¥µΩÕ¯¬Á∂‘∂À*/
			printf("finish write£° \n");

			pg_message.Clear();



			//szData = null;

			std::chrono::milliseconds timespan(10); // or whatever

			std::this_thread::sleep_for(timespan);

			break;

		}

		/*
		if (triangles_i % 90000 == 0)
		{
			break;
		}
		*/











	}



	std::cout << "Loaded " << cloud->width * cloud->height << " data points." << std::endl;
	std::cout << "Saved " << triangles.polygons.size() << " triangles." << std::endl;
















	return 0;
}



/*
	my_package::pg pg_message;
	// …Ë÷√r◊÷∂Œµƒ÷µ
	pg_message.add_r(10);
	pg_message.add_r(20);
	pg_message.add_r(30);

	pg_message.add_g(10);
	pg_message.add_g(20);
	pg_message.add_g(30);

	pg_message.add_b(10);
	pg_message.add_b(20);
	pg_message.add_b(30);

	// ªÒ»°r◊÷∂Œµƒ÷µ
	for (int i = 0; i < pg_message.r_size(); i++) {
		int value = pg_message.r(i);
		std::cout << "r[" << i << "] = " << value << std::endl;
	}

	for (int i = 0; i < pg_message.g_size(); i++) {
		int value = pg_message.g(i);
		std::cout << "g[" << i << "] = " << value << std::endl;
	}

	for (int i = 0; i < pg_message.b_size(); i++) {
		int value = pg_message.b(i);
		std::cout << "b[" << i << "] = " << value << std::endl;
	}
	// …Ë÷√v1◊÷∂Œµƒ÷µ
	my_package::V1* v1 = pg_message.add_v1();
	v1->set_x(1.0);
	v1->set_y(2.0);
	v1->set_z(3.0);

	my_package::V1* v1_1 = pg_message.add_v1();
	v1_1->set_x(1.0);
	v1_1->set_y(2.0);
	v1_1->set_z(3.0);

	my_package::V2* v2 = pg_message.add_v2();
	v2->set_x(1.0);
	v2->set_y(2.0);
	v2->set_z(3.0);

	my_package::V3* v3 = pg_message.add_v3();
	v3->set_x(1.0);
	v3->set_y(2.0);
	v3->set_z(3.0);

	// ªÒ»°v1◊÷∂Œµƒ÷µ
	for (int i = 0; i < pg_message.v1_size(); i++) {
		const my_package::V1& v1_message = pg_message.v1(i);
		std::cout << "v1[" << i << "].x = " << v1_message.x() << std::endl;
		std::cout << "v1[" << i << "].y = " << v1_message.y() << std::endl;
		std::cout << "v1[" << i << "].z = " << v1_message.z() << std::endl;
	}

	for (int i = 0; i < pg_message.v2_size(); i++) {
		const my_package::V2& v2_message = pg_message.v2(i);
		std::cout << "v2[" << i << "].x = " << v2_message.x() << std::endl;
		std::cout << "v2[" << i << "].y = " << v2_message.y() << std::endl;
		std::cout << "v2[" << i << "].z = " << v2_message.z() << std::endl;
	}

	for (int i = 0; i < pg_message.v3_size(); i++) {
		const my_package::V3& v3_message = pg_message.v3(i);
		std::cout << "v3[" << i << "].x = " << v3_message.x() << std::endl;
		std::cout << "v3[" << i << "].y = " << v3_message.y() << std::endl;
		std::cout << "v3[" << i << "].z = " << v3_message.z() << std::endl;
	}
	*/






	/*
			for (const auto& index : polygon.vertices) {
				std::cout << index << " ";
				color.r += cloud->points[index].r;
				color.g += cloud->points[index].g;
				color.b += cloud->points[index].b;
			}
			std::cout << "Color: " << (int)(color.r / polygon.vertices.size())
				<< "," << (int)(color.g / polygon.vertices.size())
				<< "," << (int)(color.b / polygon.vertices.size())
				<< std::endl;
	*/