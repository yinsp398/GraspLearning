#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\imgcodecs.hpp>

/*********************************************************************/
//һЩ�����ĺ궨��

#define		GT_RES							int							//�����ķ���ֵ���ͣ���ʶ���Ƿ���ȷ�Լ���������ͣ�

#define		GT_RES_OK						1							//�������ܵķ���ֵ����ʾ����������
#define		GT_RES_NODEVICE					2							//�������ܵķ���ֵ����ʾû�з����豸��
#define		GT_RES_DEVICEERROR				3							//�������ܵķ���ֵ����ʾ�豸û���������У�
#define		GT_RES_DEVICENOTREADY			4
#define		GT_RES_ERROR					5

#define		COLORWIDTH						1920						//��ɫͼ����
#define		COLORHEIGHT						1080						//��ɫͼ��߶�
#define		DEPTHWIDTH						512							//���ͼ����
#define		DEPTHHEIGHT						424							//���ͼ��߶�

#define		IPADDR							"192.168.1.4"				//UR5 IP��ַ
#define		PORT1							"30000"						//�˿�port1
#define		PORT2							"30001"						//�˿�port2

#define		PI								3.14159265359
#define		POSE1							0.1,-0.2,0.25,PI,0,PI		//�ؼ�·���ڵ�1
#define		POSE2							0.2,-0.5,0.25,PI,0,PI		//�ؼ�·���ڵ�2
#define		TCPPOSE							0.0,0.0,0.16,0.0,0.0,0.0	//TCPλ��
#define		PAYLOAD							1.0,0.0,0.0,0.05			//���ش�С������λ��

#define		COUNT_TIMEOUT					100							//����NN�ĳ�ʱ����
#define		TIME_TOWAIT						150000						//����UR5��kinect�ĳ�ʱʱ��15000
#define		TIME_UPDATEPARAM				1000						//����caffemodel������ʱ��

#define		MODELFILE						"../model/Grasp_train_test_deploy.prototxt"								//caffeNet��ģ���ļ�
#define		TRAINEDFILE						"../model/Grasp_iter_1000.caffemodel"									//ѵ���õĲ����ļ�
#define		MEANFILE1						"../imageset/MeanFile/meandepth.binaryproto"						//colorƽ��ֵ�ļ�
#define		MEANFILE2						"../imageset/MeanFile/meancolor.binaryproto"						//cloudpointsƽ��ֵ�ļ�

#define		COLORSPACEUP					535							//��ɫ�ռ䣨��ά��Լ�����ϱ߽�
#define		COLORSPACEDOWN					760							//�±߽�
#define		COLORSPACELEFT					800							//��߽�
#define		COLORSPACERIGHT					1125						//�ұ߽�

#define		COLORGRAPHWIDTH					100							//�ֲ���ɫͼ����
#define		COLORGRAPHHEIGHT				100							//�ֲ���ɫͼ��߶�
#define		DEPTHGRAPHWIDTH					50							//�ֲ����ͼ����
#define		DEPTHGRAPHHEIGHT				50							//�ֲ����ͼ��߶�
#define		CLOUDGRAPHWIDTH					50							//�ֲ�����ͼ����
#define		CLOUDGRAPHHEIGHT				50							//�ֲ�����ͼ��߶�


#define		BATCHSIZE						30							//һ�����ݸ���
#define		COLORSCALE						1							//��ɫͼ��ķŴ���

#define		IMAGEPATH						"../imageset/Color/"		//���ڴ洢ͼ�����ݵ�·��
#define		DEPTHPATH						"../imageset/Depth/"		//���ڴ洢������ݵ�·��
#define		CLOUDPATH						"../imageset/CloudPoints/"	//���ڴ洢������ݵ�·��
#define		PREDICTPATH						"../imageset/Predict.txt"	//Ԥ��ÿ��ץȡ�ĳɹ��ʵ��ļ�·��
#define		RESULTPATH						"../imageset/Result.txt"	//ÿ��ץȡ������ļ�·��

#define		COLOR2ROBOT11					0.32133						//��������ϵ������������ϵ��ת������Ԫ��
#define		COLOR2ROBOT12					-0.93730
#define		COLOR2ROBOT13					-0.10239
#define		COLOR2ROBOT14					0.14761
#define		COLOR2ROBOT21					-0.95147	
#define		COLOR2ROBOT22					-0.30964
#define		COLOR2ROBOT23					-0.11571
#define		COLOR2ROBOT24					-0.45001
#define		COLOR2ROBOT31					0.06066
#define		COLOR2ROBOT32					0.14098
#define		COLOR2ROBOT33					-0.93716
#define		COLOR2ROBOT34					0.90480
#define		ANGLEBIAS						(PI*0.6)					//ץȡ�Ƕȴ�ͼ�񵽻���������ϵ��ƫ��

#define		COLORFORMAT						CV_8UC1						//ͼ����cv mat�еĴ洢��ʽ��8λ��һ��ͨ���ĻҶ�ͼ
#define		DEPTHFORMAT						CV_16UC1					//���ͼ����cv::mat�еĴ洢��ʽ��16λ��һ��ͨ���ĻҶ�ͼ
#define		CLOUDFORMAT						CV_8UC1						//����ͼ����cv mat �еĴ洢��ʽ��8λ��һ��ͨ���ĻҶ�ͼ
#define		CLOUDRESOLUTION					2							//����ͼ�ķֱ��ʾ��ȣ���λmm/pixel
#define		DEPTHMEANCNT					10							//���ͼȡƽ����ͼƬ����
#define		CLOUDDEPTHBIAS					0.85
#define		CLOUDDEPTHMUL					1.7

struct GraspPose
{
	unsigned int x;
	unsigned int y;
	double theta;
};

struct Graphics
{
	cv::Mat *ColorImg;
	cv::Mat *DepthImg;
	cv::Mat *CloudPointsImg;
};



