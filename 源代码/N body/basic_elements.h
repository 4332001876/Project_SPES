#pragma once
#include <cstdio>
#include <cmath>
#include <graphics.h>
#include <vector>
#include <cstring>
#include <cctype>
#include <windows.h>


#define PI 3.141592653589793
#define WIDTH 1280
#define HEIGHT 720
#define EPSILON 0.0000001 
using namespace std;

extern double magnify_ratio; //�Ŵ���,��ʾʱ����Ϊԭ������������
extern double moveX,moveY;

int sgn(double num);//���ź���

enum material_set{metal,wood,rubber,concrete,glass,teflon};//�Ȳ��ܲ���
double friction_factor(enum material_set mat1,enum material_set mat2);//�Ȳ���,Ĭ��0.5



typedef struct coordinate
{
    double x;
    double y;
} XY;
XY operator+(XY xy1,XY xy2);
XY operator-(XY xy1,XY xy2);
XY operator-(XY xy);
XY operator*(double num,XY xy1);
XY operator*(XY xy1,double num);
XY operator*(int num,XY xy1);
XY operator*(XY xy1,int num);


inline double normal(XY vec);//������ģ��
inline XY vertical_vector(XY point1,XY point2);//�������ֱ�ߵķ�����
inline XY rotate_vector(XY vec,double rad);//����ʱ����תrad���Ⱥ�ķ�����
inline XY rotate_anticlockwise(XY vec);//����ʱ����ת90�Ⱥ�ķ�����,�������ò���˳ʱ���
inline double cross(XY vec1,XY vec2);//���������ģ��
double det_2(XY vec1,XY vec2);//2������ʽ�������������������
inline double dot(XY vec1,XY vec2);//�������
inline XY unit_vector(XY vec);//���ص�λ��������

inline double depth(XY pt1,XY pt2, XY pt);//�����Ƕ���ߵ����,��С��0
inline XY vertical_point(XY pt1,XY pt2, XY pt);//��pt��ֱ��pt1,pt2�ϵĴ�ֱ��
XY cross_point(XY pt1,XY pt2, XY pt3, XY pt4);//��ֱ��pt1,pt2��ֱ��pt3,pt4�Ľ���,�ô˺����뱣֤�н���

void equation(vector<double> &num,XY pt1,XY pt2);//�������ֱ�ߵķ���ϵ��,��Ax+By+C=0˳��
int is_cross(vector<XY> line1,vector<XY> line2);//�ж����߶��Ƿ��ཻ�����򷵻�1�����򷵻�0
inline int is_cross(XY line1_0,XY line1_1,XY line2_0,XY line2_1);//�ж����߶��Ƿ��ཻ�����򷵻�1�����򷵻�0
int is_circle_line_intersect(XY pt1,XY pt2,XY center,double r);//�ж��߶κ�Բ�Ƿ����������㣬���򷵻�1

class body
{
public:
	vector<XY> points;
	material_set material; //�Ȳ��ܲ���
	double youngs_modulus;//����ģ��

    double m;         //������ע�����Ǹ�
	double I;         //ת������
	double q;//��ɣ���������������ȷֲ�����Ч��������
	double r;//�뾶,�Զ�������Ǵ��°뾶��ֻ�豣֤ͬһ��������,����ι������ģ������ؼ���

    XY xy;            //������������
    XY v;             //�ٶ�,��XY������ʾ
    XY a;             //���ٶ�,��XY������ʾ,����ŷ�����֣�һ�ף���ʵ�ò���
    double omega;     //���ٶ�,��λrad/s������ʾ��ʱ�뷽�򣨲ο���ѧ������ϵ��
	double omega_a;     //���ٶȵļ��ٶ�,��λrad/s^2������ʾ��ʱ�뷽��
    double direction; //����,0ָ�����ҷ�x�᷽��,��Χ0-2*PI(��ʱ����Χû��ϵ����Ҫ��̫��),ע��Ҫ��reset_rad(double direction)�������䱣���ں���Χ�ڽ������

    XY f;          //����,��XY������ʾ����Ҫ�������
    double moment; //����,����������ʲ�Ҫ��Ӧ��������Ц�ޣ�����Ҫ������룬����ʾ��ʱ�뷽��
	XY temp_f[4];//��RK4�õ�
	double temp_moment[4];//��RK4�õ�

    void set_mass(double mass);
    void set_coordinate(double x, double y);
    void set_v(double x, double y);
    void set_a(double x, double y);
    void set_w(double rad);
    void set_direction(double rad);
    void set_all(double mass, double xy_x, double xy_y, double vx, double vy, double ax, double ay, double rad_w, double rad_direction);
	void copy_basic_var(class body &body2);//��������ֵ
	
    double getMass();
    void print();    
	void pointDisplay();
	double reset_rad();

	void reset_force();//��������
	void cal_acceleration(); 
	void EulerCalculus(double h); //hָʱ�䲽��
	void RK4_cal_force();//RK4���ĸ�����ȨΪһ����

	virtual int is_inside(XY pt);//�麯��,�൱��ռλ��,֮��ͬ��״�����������������
	virtual void init_cmd();//��cmd���ʼ��
	virtual void Display();

	//�������õ�
	virtual void terDisplay();
	virtual void init_cmd_ter();//��cmd���ʼ��
};


class Star:public body
{
public:
	void set(double mass,XY center,double radius);
	void Display();
	void init_cmd();

	int is_inside(XY pt);//�жϵ��Ƿ���ͼ������򷵻�1
	void copy_var_cir(class Star &cir2);//�������б���ֵ

};


class Circle:public body
{
public:
	

	void set(double mass,XY center,double radius);
	void init_cmd();
	void Display();

	int is_inside(XY pt);//�жϵ��Ƿ���ͼ������򷵻�1
	void copy_var_cir(class Circle &cir2);//�������б���ֵ

};
class terCircle:public Circle
{
public:
	void terDisplay();
	void init_cmd_ter();//��cmd���ʼ��
};
class Triangle:public body
{
public:
	vector<XY> ori_vectors;//���������ĵ���������
	vector<XY> normal_vectors;//�����θ��߷�����
	double max_r;//��ײԲ�㷨����

	void set(double mass,vector<XY> set_points);
	void init_cmd();
	void copy_var_tri(class Triangle &tri2);//�������б���ֵ
	

	void reset_points();//��תʱ�����������

	int is_inside(XY pt);//�жϵ��Ƿ���ͼ������򷵻�1

};
class terTriangle:public Triangle//������
{
public:
	void init_cmd_ter();//��cmd���ʼ��
};
class Rectangle:public body
{
public:
	double length;//��
	double width;//��
	double max_r;//��ײԲ�㷨����
	vector<XY> normal_vectors;//�����θ��߷�����

	void set(double mass,double length,double width,XY center,double direction);
	void init_cmd();
	void copy_var_rect(class Rectangle &rect2);//�������б���ֵ
	

	void reset_points();//��תʱ�����������

	int is_inside(XY pt);//�жϵ��Ƿ���ͼ������򷵻�1�����㷨��ȷ��������Ϊ͹������ҵ㼯��ʱ��˳������
};
class terRectangle:public Rectangle//������
{
public:
	void init_cmd_ter();//��cmd���ʼ��
};




class Terrain
{
public:
	material_set material;
	virtual void init_cmd()=0;
};

class Plane:public Terrain//ƽ����
{
public:
	double a;//��Ӧ����Ax+By+C=0
	double b;
	double c;
	vector<XY> points;
	double length;//����,����ι������ģ����ؼ���
	double youngs_modulus;//����ģ��

	int collision_detect(XY xy1,XY xy2);//������������Ƿ񴩹�ƽ��
	void set(XY pt1,XY pt2);
	void init_cmd();
	void Display();
};




//inline��������Ҫ����ͷ�ļ�
inline double normal(XY vec)//������ģ��
{
	double norm;
	norm=sqrt(vec.x*vec.x+vec.y*vec.y);
	return norm;
}
inline XY vertical_vector(XY point1,XY point2)//�������ֱ�ߵķ�����
{
	XY vec;
	vec.x=point1.y-point2.y;
	vec.y=point2.x-point1.x;
	return vec;
}
inline XY rotate_vector(XY vec,double rad)//����ʱ����תrad���Ⱥ�Ľ��ٶȵķ�����
{
	XY result;
	result.x=vec.x*cos(rad)-vec.y*sin(rad);
	result.y=vec.y*cos(rad)+vec.x*sin(rad);
	return result;
}
inline XY rotate_anticlockwise(XY vec)//����ʱ����ת90�Ⱥ�ķ�����,�������ò���˳ʱ���
{
	XY result;
	result.x=-vec.y;
	result.y=vec.x;
	return result;
}
inline double cross(XY vec1,XY vec2)//�������,��������
{
	double product;
	product=vec2.y*vec1.x-vec2.x*vec1.y;
	return product;
}
inline double dot(XY vec1,XY vec2)//�������
{
	double product;
	product=vec1.x*vec2.x+vec1.y*vec2.y;
	return product;
}
inline XY unit_vector(XY vec)//���ص�λ��������
{
	XY result;
	if(normal(vec)==0)
		result=vec;
	else
	{
		result=(1/normal(vec))*vec;
	}
	return result;

}


inline double depth(XY pt1, XY pt2, XY pt)//�����Ƕ���ߵ����,��С��0
{
	double result;
	XY n_vector=vertical_vector(pt1,pt2);
	result=abs(dot(pt-pt2,n_vector)/normal(n_vector));
	return result;
}
inline XY vertical_point(XY pt1,XY pt2, XY pt)//��pt��ֱ��pt1,pt2�ϵĴ�ֱ��
{
	XY result;
	result=pt2+(dot(pt1-pt2,pt-pt2)/ (normal(pt1-pt2)*normal(pt1-pt2)) )*(pt1-pt2);
	return result;
}


inline int is_cross(XY line1_0,XY line1_1,XY line2_0,XY line2_1)//�ж����߶��Ƿ��ཻ�����򷵻�1�����򷵻�0
{
	int flag1=0,flag2=0;
	vector<double> num;
	equation(num,line1_0,line1_1);
	if((num[0]*line2_0.x+num[1]*line2_0.y+num[2])*(num[0]*line2_1.x+num[1]*line2_1.y+num[2])<0)
		flag1=1;
	else
		return 0;
	equation(num,line2_0,line2_1);
	if((num[0]*line1_0.x+num[1]*line1_0.y+num[2])*(num[0]*line1_1.x+num[1]*line1_1.y+num[2])<0)
		flag2=1;
	if(flag1==1&&flag2==1)
		return 1;
	else
		return 0;
}





