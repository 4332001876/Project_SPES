#pragma once
#include "basic_elements.h"
using namespace std;

#define G_FORCE 9.80
#define PD_FACTOR 5   //������������΢�ֿ�����(PD)�õĲ���


class SPACE
{
public:
	vector<class Circle> cir;
	vector<class Triangle> tri;
	vector<class Rectangle> rect;
	vector<class Plane> plane;
	vector<class terCircle> tercir;
	vector<class terTriangle> tertri;
	vector<class terRectangle> terrect;

	void init_cmd();
	template <typename T> void add_element(vector<T> &obj);
	template <typename T> void add_element_ter(vector<T> &obj);

	void cal_launch ();//��ʼ����ʱ�����㣬�������ٶ�����
	void cal_main(double time_step);
	void EulerCalculus(double time_step);//ר��������ŷ�����ֵģ�ΪRK4����
	void RK4(double time_step,class SPACE &space2);//RK4����

	void copy_status(class SPACE &space2);
	void copy_force(class SPACE &space2,int step);//RK4���ֵ�n��ʱ��������������

	void generate_graph();//�������SPACE��ͼ�λ�����

};

void circle_circle_detect_collision(class Circle &cir1,class Circle &cir2);
template <typename T> void circle_poly_detect_collision(class Circle &cir,T &poly);
template <typename T1,typename T2> void poly_poly_detect_collision(T1 &poly1,T2 &poly2);
template <typename T1,typename T2> void poly_poly_collision_respond(T1 &poly1,T2 &poly2,XY pt,double dep,XY vector_n);//pt����ײ��
void plane_circle_detect_collision(class Plane &plain,class Circle &cir);
template <typename T> void plane_poly_detect_collision(class Plane &plain,T &poly);


void clear(char *buffer, char *cmd, int lenth);//����ַ���


