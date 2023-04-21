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

extern double magnify_ratio; //放大倍数,显示时坐标为原坐标乘上这个数
extern double moveX,moveY;

int sgn(double num);//符号函数

enum material_set{metal,wood,rubber,concrete,glass,teflon};//先不管材料
double friction_factor(enum material_set mat1,enum material_set mat2);//先不管,默认0.5



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


inline double normal(XY vec);//求向量模长
inline XY vertical_vector(XY point1,XY point2);//求过两点直线的法向量
inline XY rotate_vector(XY vec,double rad);//求逆时针旋转rad弧度后的法向量
inline XY rotate_anticlockwise(XY vec);//求逆时针旋转90度后的法向量,计算中用不着顺时针的
inline double cross(XY vec1,XY vec2);//向量叉积的模长
double det_2(XY vec1,XY vec2);//2阶行列式，或考虑正负的向量叉积
inline double dot(XY vec1,XY vec2);//向量点积
inline XY unit_vector(XY vec);//返回单位化的向量

inline double depth(XY pt1,XY pt2, XY pt);//计算点嵌入线的深度,不小于0
inline XY vertical_point(XY pt1,XY pt2, XY pt);//作pt到直线pt1,pt2上的垂直点
XY cross_point(XY pt1,XY pt2, XY pt3, XY pt4);//作直线pt1,pt2与直线pt3,pt4的交点,用此函数请保证有交点

void equation(vector<double> &num,XY pt1,XY pt2);//求过两点直线的方程系数,按Ax+By+C=0顺序
int is_cross(vector<XY> line1,vector<XY> line2);//判断两线段是否相交，是则返回1，否则返回0
inline int is_cross(XY line1_0,XY line1_1,XY line2_0,XY line2_1);//判断两线段是否相交，是则返回1，否则返回0
int is_circle_line_intersect(XY pt1,XY pt2,XY center,double r);//判断线段和圆是否有两个交点，是则返回1

class body
{
public:
	vector<XY> points;
	material_set material; //先不管材料
	double youngs_modulus;//杨氏模量

    double m;         //质量，注意它非负
	double I;         //转动惯量
	double q;//电荷，有正负，假设均匀分布，等效在重心上
	double r;//半径,对多边形则是大致半径（只需保证同一数量级）,用来喂给杨氏模量的相关计算

    XY xy;            //刚体重心坐标
    XY v;             //速度,以XY分量表示
    XY a;             //加速度,以XY分量表示,若用欧拉积分（一阶）其实用不上
    double omega;     //角速度,单位rad/s，正表示逆时针方向（参考数学极坐标系）
	double omega_a;     //角速度的加速度,单位rad/s^2，正表示逆时针方向
    double direction; //朝向,0指向正右方x轴方向,范围0-2*PI(暂时超范围没关系，不要超太多),注意要用reset_rad(double direction)函数让其保持在合理范围内降低误差

    XY f;          //受力,以XY分量表示，需要外界输入
    double moment; //力矩,看到这个单词不要反应不过来（笑哭），需要外界输入，正表示逆时针方向
	XY temp_f[4];//给RK4用的
	double temp_moment[4];//给RK4用的

    void set_mass(double mass);
    void set_coordinate(double x, double y);
    void set_v(double x, double y);
    void set_a(double x, double y);
    void set_w(double rad);
    void set_direction(double rad);
    void set_all(double mass, double xy_x, double xy_y, double vx, double vy, double ax, double ay, double rad_w, double rad_direction);
	void copy_basic_var(class body &body2);//拷贝变量值
	
    double getMass();
    void print();    
	void pointDisplay();
	double reset_rad();

	void reset_force();//受力置零
	void cal_acceleration(); 
	void EulerCalculus(double h); //h指时间步长
	void RK4_cal_force();//RK4由四个力加权为一个力

	virtual int is_inside(XY pt);//虚函数,相当于占位符,之后不同形状都可以用这个函数名
	virtual void init_cmd();//在cmd里初始化
	virtual void Display();

	//给地形用的
	virtual void terDisplay();
	virtual void init_cmd_ter();//在cmd里初始化
};


class Circle:public body
{
public:
	

	void set(double mass,XY center,double radius);
	void init_cmd();
	void Display();

	int is_inside(XY pt);//判断点是否在图形里，是则返回1
	void copy_var_cir(class Circle &cir2);//拷贝所有变量值

};
class terCircle:public Circle
{
public:
	void terDisplay();
	void init_cmd_ter();//在cmd里初始化
};
class Triangle:public body
{
public:
	vector<XY> ori_vectors;//三角形重心到各点向量
	vector<XY> normal_vectors;//三角形各边法向量
	double max_r;//碰撞圆算法参数

	void set(double mass,vector<XY> set_points);
	void init_cmd();
	void copy_var_tri(class Triangle &tri2);//拷贝所有变量值
	

	void reset_points();//旋转时重新求点坐标

	int is_inside(XY pt);//判断点是否在图形里，是则返回1

};
class terTriangle:public Triangle//地形类
{
public:
	void init_cmd_ter();//在cmd里初始化
};
class Rectangle:public body
{
public:
	double length;//长
	double width;//宽
	double max_r;//碰撞圆算法参数
	vector<XY> normal_vectors;//三角形各边法向量

	void set(double mass,double length,double width,XY center,double direction);
	void init_cmd();
	void copy_var_rect(class Rectangle &rect2);//拷贝所有变量值
	

	void reset_points();//旋转时重新求点坐标

	int is_inside(XY pt);//判断点是否在图形里，是则返回1，该算法正确性依赖其为凸多边形且点集按时针顺序排列
};
class terRectangle:public Rectangle//地形类
{
public:
	void init_cmd_ter();//在cmd里初始化
};




class Terrain
{
public:
	material_set material;
	virtual void init_cmd()=0;
};

class Plane:public Terrain//平面类
{
public:
	double a;//对应方程Ax+By+C=0
	double b;
	double c;
	vector<XY> points;
	double length;//长度,用来喂给杨氏模量相关计算
	double youngs_modulus;//杨氏模量

	int collision_detect(XY xy1,XY xy2);//检测两点连线是否穿过平面
	void set(XY pt1,XY pt2);
	void init_cmd();
	void Display();
};




//inline函数定义要放在头文件
inline double normal(XY vec)//求向量模长
{
	double norm;
	norm=sqrt(vec.x*vec.x+vec.y*vec.y);
	return norm;
}
inline XY vertical_vector(XY point1,XY point2)//求过两点直线的法向量
{
	XY vec;
	vec.x=point1.y-point2.y;
	vec.y=point2.x-point1.x;
	return vec;
}
inline XY rotate_vector(XY vec,double rad)//求逆时针旋转rad弧度后的角速度的法向量
{
	XY result;
	result.x=vec.x*cos(rad)-vec.y*sin(rad);
	result.y=vec.y*cos(rad)+vec.x*sin(rad);
	return result;
}
inline XY rotate_anticlockwise(XY vec)//求逆时针旋转90度后的法向量,计算中用不着顺时针的
{
	XY result;
	result.x=-vec.y;
	result.y=vec.x;
	return result;
}
inline double cross(XY vec1,XY vec2)//向量叉积,考虑正负
{
	double product;
	product=vec2.y*vec1.x-vec2.x*vec1.y;
	return product;
}
inline double dot(XY vec1,XY vec2)//向量点积
{
	double product;
	product=vec1.x*vec2.x+vec1.y*vec2.y;
	return product;
}
inline XY unit_vector(XY vec)//返回单位化的向量
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


inline double depth(XY pt1, XY pt2, XY pt)//计算点嵌入线的深度,不小于0
{
	double result;
	XY n_vector=vertical_vector(pt1,pt2);
	result=abs(dot(pt-pt2,n_vector)/normal(n_vector));
	return result;
}
inline XY vertical_point(XY pt1,XY pt2, XY pt)//作pt到直线pt1,pt2上的垂直点
{
	XY result;
	result=pt2+(dot(pt1-pt2,pt-pt2)/ (normal(pt1-pt2)*normal(pt1-pt2)) )*(pt1-pt2);
	return result;
}


inline int is_cross(XY line1_0,XY line1_1,XY line2_0,XY line2_1)//判断两线段是否相交，是则返回1，否则返回0
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





