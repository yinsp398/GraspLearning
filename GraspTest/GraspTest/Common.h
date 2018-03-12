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
#define		POSE1							0.1,-0.1,0.5,PI,0,PI		//�ؼ�·���ڵ�1
#define		POSE2							0.2,-0.5,0.5,PI,0,PI		//�ؼ�·���ڵ�2

#define		COUNT_TIMEOUT					100							//����NN�ĳ�ʱ����
#define		TIME_TOWAIT						15000						//����UR5��kinect�ĳ�ʱʱ��15000

#define		MODELFILE						"test1"						//caffeNet��ģ���ļ�
#define		TRAINEDFILE						"test13"					//ѵ���õĲ����ļ�
#define		MEANFILE						"test2"						//ƽ��ֵ�ļ�

#define		COLORSPACEUP					123							//��ɫ�ռ䣨��ά��Լ�����ϱ߽�
#define		COLORSPACEDOWN					456							//�±߽�
#define		COLORSPACELEFT					456							//��߽�
#define		COLORSPACERIGHT					456							//�ұ߽�

#define		ROBOTSPACEUP					123							//�����˿ռ䣨��ά��Լ�����ϱ߽�
#define		ROBOTSPACEDOWN					456							//�±߽�
#define		ROBOTSPACELEFT					456							//��߽�
#define		ROBOTSPACERIGHT					456							//�ұ߽�

#define		COLORGRAPHWIDE					60							//�ֲ���ɫͼ����
#define		COLORGRAPHHEIGHT				60							//�ֲ���ɫͼ��߶�
#define		DEPTHGRAPHWIDE					20							//�ֲ����ͼ����
#define		DEPTHGRAPHHEIGHT				20							//�ֲ����ͼ��߶�


#define		BATCHSIZE						30							//һ�����ݸ���
#define		COLORSCALE						1							//��ɫͼ��ķŴ���

#define		IMAGEPATH						"../dataset/picture/Image"	//���ڴ洢ͼ�����ݵ�·��
#define		PREDICTPATH						"../dataset/Predict.txt"	//Ԥ��ÿ��ץȡ�ĳɹ��ʵ��ļ�·��
#define		RESULTPATH						"../dataset/Result.txt"		//ÿ��ץȡ������ļ�·��

#define		COLOR2ROBOT11					1							//��������ϵ������������ϵ��ת������Ԫ��
#define		COLOR2ROBOT12					1	
#define		COLOR2ROBOT13					1	
#define		COLOR2ROBOT14					1	
#define		COLOR2ROBOT21					1	
#define		COLOR2ROBOT22					1	
#define		COLOR2ROBOT23					1	
#define		COLOR2ROBOT24					1	
#define		COLOR2ROBOT31					1	
#define		COLOR2ROBOT32					1	
#define		COLOR2ROBOT33					1	
#define		COLOR2ROBOT34					1	
#define		ANGLEBIAS						0							//ץȡ�Ƕȴ�ͼ�񵽻���������ϵ��ƫ��

#define		IMAGEFORMAT						CV_8UC1						//ͼ����cv mat�еĴ洢��ʽ��0~255��һ��ͨ���ĻҶ�ͼ

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
	cv::Mat *DepthInColorImg;
};



