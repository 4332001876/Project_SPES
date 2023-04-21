#include "basic_elements.h"

double magnify_ratio=20; //�Ŵ���,��ʾʱ����Ϊԭ������������
double moveX=0,moveY=0;

double friction_factor(enum material_set mat1,enum material_set mat2)//�Ȳ���,Ĭ��0.5
{
	return 0.5;
}
int sgn(double num)//���ź���
{
	if(num>0)
		return 1;
	else
	{
		if(num==0)
			return 0;
		else
			return -1;
	}
}


XY operator+(XY xy1,XY xy2)
{
	XY xy;
	xy.x=xy1.x+xy2.x;
	xy.y=xy1.y+xy2.y;
	return xy;
}
XY operator-(XY xy1,XY xy2)
{
	XY xy;
	xy.x=xy1.x-xy2.x;
	xy.y=xy1.y-xy2.y;
	return xy;
}
XY operator-(XY xy)
{
	XY result;
	result.x=-xy.x;
	result.y=-xy.y;
	return result;
}
XY operator*(double num,XY xy1)
{
	XY xy;
	xy.x=xy1.x*num;
	xy.y=xy1.y*num;
	return xy;
}
XY operator*(XY xy1,double num)
{
	XY xy;
	xy.x=xy1.x*num;
	xy.y=xy1.y*num;
	return xy;
}

XY operator*(int num,XY xy1)
{
	XY xy;
	xy.x=xy1.x*num;
	xy.y=xy1.y*num;
	return xy;
}
XY operator*(XY xy1,int num)
{
	XY xy;
	xy.x=xy1.x*num;
	xy.y=xy1.y*num;
	return xy;
}


double det_2(XY vec1,XY vec2)//2������ʽ�����������������һ��
{
	double det;
	det=vec2.y*vec1.x-vec2.x*vec1.y;
	return det;
}



XY cross_point(XY pt1,XY pt2, XY pt3, XY pt4)//��ֱ��pt1,pt2��ֱ��pt3,pt4�Ľ���,�ô˺����뱣֤�н���
{
	double a1=pt3.y-pt4.y;
	double a2=pt1.y-pt2.y;
	double b1=pt4.x-pt3.x;
	double b2=pt2.x-pt1.x;
	double c1=pt3.x*pt4.y-pt4.x*pt3.y;
	double c2=pt1.x*pt2.y-pt2.x*pt1.y;
	XY result;
	if(a1*b2-a2*b1!=0)
	{
		result.x=-(b2*c1-b1*c2)/(a1*b2-a2*b1);
		result.y=-(a1*c2-a2*c1)/(a1*b2-a2*b1);
	}
	else
	{
		XY m1,m2;
		m1=(pt1+pt2)*0.5;
		m2=(pt3+pt4)*0.5;
		result=(m1+m2)*0.5;
	}
	return result;
}


void equation(vector<double> &num,XY pt1,XY pt2)//�������ֱ�ߵķ���ϵ��,��Ax+By+C=0˳��
{
	num.clear();//���ϵ����
	double a,b,c;
	a=pt1.y-pt2.y;
	b=pt2.x-pt1.x;
	c=pt1.x*pt2.y-pt2.x*pt1.y;
	num.push_back(a);
	num.push_back(b);
	num.push_back(c);
}

int is_cross(vector<XY> line1,vector<XY> line2)//�ж����߶��Ƿ��ཻ�����򷵻�1�����򷵻�0
{
	int flag=0;
	flag=is_cross(line1[0],line1[1],line2[0],line2[1]);
	return flag;
}
int is_circle_line_intersect(XY pt1,XY pt2,XY center,double r)//�ж��߶κ�Բ�Ƿ����������㣬���򷵻�1
{
	if(normal(center-pt1)>=r && normal(center-pt2)>=r)
	{
		XY ver_pt=vertical_point(pt1,pt2,center);
		if(normal(ver_pt-center)<r&&is_cross(pt1,pt2,center,center+10*(ver_pt-center)))
			return 1;
		else
			return 0;
	}
	else
		return 0;


}

int body::is_inside(XY pt)//�麯��,�൱��ռλ��,֮��ͬ��״�����������������
{
	return 0;
}
void body::init_cmd()//�麯��,�൱��ռλ��,��cmd���ʼ��
{

}
void body::init_cmd_ter()//�麯��,�൱��ռλ��,��cmd���ʼ��
{

}


void body::set_mass(double mass)//��ֹ����С�ڵ���0
{
    if (mass < 0)
    {
        printf("Mass must not be a negetive number!Reset to %lf",-mass);
		m=-mass;
		return;
    }
    else
	{
		if(mass==0)
			mass=EPSILON;
		else
			m = mass;
	}
}
void body::set_coordinate(double x, double y)
{
    xy.x = x;
    xy.y = y;
}
void body::set_v(double x, double y)
{
    v.x = x;
    v.y = y;
}
void body::set_a(double x, double y)
{
    a.x = x;
    a.y = y;
}
void body::set_w(double rad)
{
    omega = rad;
}
void body::set_direction(double rad)
{
    direction = rad;
}
void body::set_all(double mass, double xy_x, double xy_y, double vx, double vy, double ax, double ay, double rad_w, double rad_direction)
{
    set_mass(mass);
    set_coordinate(xy_x, xy_y);
    set_v(vx, vy);
    set_a(ax, ay);
    set_w(rad_w);
    set_direction(rad_direction);
}
void body::copy_basic_var(class body &body2)//��������ֵ
{
	//Pointsû�п���
	body2.material=material;
	body2.youngs_modulus=youngs_modulus;

	body2.m=m;
	body2.I=I;
	body2.q=q;
	body2.r=r;

	body2.xy=xy;
	body2.v=v;
	body2.a=a;

	body2.omega=omega;
	body2.omega_a=omega_a;
	body2.direction=direction;
	//�������������
}


double body::getMass()
{
    return m;
}
void body::print(void)
{
    printf("Position:(%lf,%lf)\tSpeed:(%lf,%lf)\tAcceleration:(%lf,%lf)\n", xy.x, xy.y, v.x, v.y, a.x, a.y);
    printf("Angular velocity:%lf\tDirection:%lf\n", omega, direction); //cout�ڸ�ʽ�����ʱ����printf
}
void body::pointDisplay()
{
	fillcircle(magnify_ratio*(xy.x+moveX),magnify_ratio*(xy.y+moveY),6);//y��������Ӧ�÷��Ϸ�����
	
}
void body::Display()
{
	unsigned int i;
	for(i=0;i < points.size();i++)
	{
		line(magnify_ratio*(points[i].x+moveX),magnify_ratio*(points[i].y+moveY),magnify_ratio*(points[(i+1)%points.size()].x+moveX),magnify_ratio*(points[(i+1)%points.size()].y+moveY));
	}
}
void body::terDisplay()
{
	int pts[8];
	unsigned int i;
	for(i=0;i < points.size();i++)
	{
		pts[i*2]=magnify_ratio*(points[i].x+moveX);
		pts[i*2+1]=magnify_ratio*(points[i].y+moveY);
		line(magnify_ratio*(points[i].x+moveX),magnify_ratio*(points[i].y+moveY),magnify_ratio*(points[(i+1)%points.size()].x+moveX),magnify_ratio*(points[(i+1)%points.size()].y+moveY));
	}
	//floodfill(xy.x,xy.y,GREEN,FLOODFILLBORDER);
	fillpolygon((POINT*)pts, points.size());
}


void body::reset_force()
{
	f.x=0;
	f.y=0;
	moment=0;
}
void body::cal_acceleration()
{
	a.x = f.x / m;
    a.y = f.y / m;
	omega_a=moment/I;
}
void body::EulerCalculus(double h)
{
    a.x = f.x / m;
    a.y = f.y / m;
    //M=J*beta,MΪ���أ�JΪת��������betaΪ�Ǽ��ٶ�
	omega_a = moment/I;
    v.x += a.x * h;
    v.y += a.y * h;
    xy.x += v.x * h;
    xy.y += v.y * h;
	omega += omega_a*h;
    direction += omega * h;
	this->reset_rad();
}
void body::RK4_cal_force()//RK4���ĸ�����ȨΪһ����
{
	f=(1.0/6.0)*(temp_f[0]+ 2*temp_f[1] + 2*temp_f[2] +temp_f[3]);
	moment=(1.0/6.0)*(temp_moment[0]+ 2*temp_moment[1] + 2*temp_moment[2] +temp_moment[3]);
}


double body::reset_rad() //��ֹdirection���������
{
    int n = (int)((fabs(direction) / (2 * PI))); //floor������ȡ��
    if (direction > 6.2832)                      //��������һ������֤�����������ѭ�����Է�ֹ��ѭ����³���ԣ���
    {
        return direction - 2 * n * PI;
    }
    else
    {
        if (direction < -6.2832)
        {
            return direction + 2 * n * PI;
        }
        else
            return direction;
    }
}


void Star::set(double mass,XY center,double radius)
{
	if(radius<0)
	{
		radius=-radius;
		printf("Radius<0!Reset to %lf",radius);
		
	}
	r=radius;
	set_mass(mass);//��ֹ����С�ڵ���0
	xy=center;
	direction=0;
	I=0.5*m*r*r;//Բת��������ʽ
}
void Star::init_cmd()
{
	youngs_modulus=1.8e11;
	f.x=0;
	f.y=0;
	a.x=0;
	a.y=0;
	omega_a=0;
	q=0;
	printf("Velocity(x and y)��");
	scanf("%lf%lf",&v.x,&v.y);
	omega=0;

	double mass;
	XY center;
	double radius;
	printf("Mass��");
	scanf("%lf",&mass);
	printf("Center(x and y)��");
	scanf("%lf%lf",&center.x,&center.y);
	radius=5;

	this->set(mass,center,radius);
	this->reset_force();
}
void Star::copy_var_cir(class Star &cir2)//�������б���ֵ
{
	this->copy_basic_var(cir2);

}
void Star::Display()
{
	

	fillcircle(magnify_ratio*(xy.x+moveX),magnify_ratio*(xy.y+moveY),5);//y��������Ӧ�÷��Ϸ�����
	

}
int Star::is_inside(XY pt)//�жϵ��Ƿ���ͼ������򷵻�1
{
	if(normal(pt-xy)<=r)
		return 1;
	else
		return 0;
}

void Circle::set(double mass,XY center,double radius)
{
	if(radius<0)
	{
		radius=-radius;
		printf("Radius<0!Reset to %lf",radius);
		
	}
	r=radius;
	set_mass(mass);//��ֹ����С�ڵ���0
	xy=center;
	direction=0;
	I=0.5*m*r*r;//Բת��������ʽ
}
int Circle::is_inside(XY pt)//�жϵ��Ƿ���ͼ������򷵻�1
{
	if(normal(pt-xy)<=r)
		return 1;
	else
		return 0;
}
void Circle::init_cmd()
{
	youngs_modulus=1.8e11;
	f.x=0;
	f.y=0;
	a.x=0;
	a.y=0;
	omega_a=0;
	printf("Electric charge(���)��");
	scanf("%lf",&q);
	printf("Velocity(x and y)��");
	scanf("%lf%lf",&v.x,&v.y);
	printf("Angular velocity��");
	scanf("%lf",&omega);

	double mass;
	XY center;
	double radius;
	printf("Mass��");
	scanf("%lf",&mass);
	printf("Center(x and y)��");
	scanf("%lf%lf",&center.x,&center.y);
	printf("Radius��");
	scanf("%lf",&radius);

	this->set(mass,center,radius);
	this->reset_force();
}
void Circle::copy_var_cir(class Circle &cir2)//�������б���ֵ
{
	this->copy_basic_var(cir2);

}
void Circle::Display()
{
	circle(magnify_ratio*(xy.x+moveX),magnify_ratio*(xy.y+moveY),magnify_ratio*r);
	line(magnify_ratio*(xy.x+cos(direction)*r+moveX),magnify_ratio*(xy.y+sin(direction)*r+moveY),magnify_ratio*(xy.x-cos(direction)*r+moveX),magnify_ratio*(xy.y-sin(direction)*r+moveY));
}


void terCircle::terDisplay()
{
	fillcircle(magnify_ratio*(xy.x+moveX),magnify_ratio*(xy.y+moveY),magnify_ratio*r);
}

void terCircle::init_cmd_ter()
{
	//��Ȼ��Щ�����Ե����޹ؽ�Ҫ����Ӱ����ײ�㷨����ȷ��
	youngs_modulus=1.8e11;
	f.x=0;
	f.y=0;
	a.x=0;
	a.y=0;
	omega_a=0;
	q=0;
	v.x=0;v.y=0;
	omega=0;

	double mass;
	XY center;
	double radius;
	
	printf("Center(x and y)��");
	scanf("%lf%lf",&center.x,&center.y);
	printf("Radius��");
	scanf("%lf",&radius);
	mass=4.0/3.0*PI*abs(radius*radius*radius)*7900;
	this->set(mass,center,radius);
}

void Triangle::set(double mass,vector<XY> set_points)
{
	int i;
	if(set_points.size()!=3)
	{
		printf("Set triangle failed!");
		return;
	}
	direction=0;
	xy=set_points[0]+set_points[1]+set_points[2];//���������Ĺ�ʽ(���滹Ҫ����3)
	xy.x=xy.x/3;
	xy.y=xy.y/3;
	for(i=0;i<3;i++)
	{
		normal_vectors.push_back(unit_vector(rotate_anticlockwise(set_points[i]-set_points[(i+1)%3])));
	}

	this->set_mass(mass);//��ֹ����С�ڵ���0
	
	vector<XY>::iterator it;
	for(it=set_points.begin();it<set_points.end();it++)
	{
		points.push_back(*it);
		ori_vectors.push_back(*it-xy);//���������ĵ���������
	}

	double len1,len2,len3;
	len1=normal(ori_vectors[0]);
	len2=normal(ori_vectors[1]);
	len3=normal(ori_vectors[2]);
	r=(len1+len2+len3)/3;
	max_r=max(max(len1,len2),len3);

	//��ת������
	double sum=0;
	
	for(i=0;i<3;i++)
	{
		sum=sum+points[i].x*points[i].x+points[i].y*points[i].y;
		sum=sum-points[i].x*points[(i+1)%3].x-points[i].y*points[(i+1)%3].y;
	}
	I=m/18*abs(sum);//������ת��������ʽ

}
int Triangle::is_inside(XY pt)//�жϵ��Ƿ���ͼ������򷵻�1
{
	if(normal(pt-xy)>=max_r)
		return 0;
	int i;
	int sgn_sum=0;
	vector<XY> side(3),pt_to_points(3);//sideΪ������
	for(i=0;i<3;i++)
	{
		side[i]=points[(i+1)%3]-points[i];
		pt_to_points[i]=pt-points[i];
		sgn_sum+=sgn(det_2(side[i],pt_to_points[i]));
	}
	if(sgn_sum==3||sgn_sum==-3)
		return 1;
	else
		return 0;
}
void Triangle::init_cmd()
{
	youngs_modulus=1.8e11;
	f.x=0;
	f.y=0;
	a.x=0;
	a.y=0;
	omega_a=0;
	printf("Electric charge(���)��");
	scanf("%lf",&q);
	printf("Velocity(x and y)��");
	scanf("%lf%lf",&v.x,&v.y);
	printf("Angular velocity��");
	scanf("%lf",&omega);

	double mass;
	printf("Mass��");
	scanf("%lf",&mass);
	
	XY temp;
	vector<XY> set_points;
	
	printf("Points One(x and y)��");
	scanf("%lf%lf",&temp.x,&temp.y);
	set_points.push_back(temp);
	printf("Points Two(x and y)��");
	scanf("%lf%lf",&temp.x,&temp.y);
	set_points.push_back(temp);
	printf("Points Three(x and y)��");
	scanf("%lf%lf",&temp.x,&temp.y);
	set_points.push_back(temp);
	

	this->set(mass,set_points);
	this->reset_force();
}
void Triangle::copy_var_tri(class Triangle &tri2)//�������б���ֵ
{
	this->copy_basic_var(tri2);
	int i;
	for(i=0;i<3;i++)
	{
		tri2.points[i]=points[i];
		tri2.ori_vectors[i]=ori_vectors[i];
		tri2.normal_vectors[i]=normal_vectors[i];
	}
	tri2.max_r=max_r;

}
void terTriangle::init_cmd_ter()
{
	//��Ȼ��Щ�����Ե����޹ؽ�Ҫ����Ӱ����ײ�㷨����ȷ��
	youngs_modulus=1.8e11;
	f.x=0;
	f.y=0;
	a.x=0;
	a.y=0;
	omega_a=0;
	q=0;
	v.x=0;v.y=0;
	omega=0;

	double mass=1;//��ʱ��
	XY temp;
	vector<XY> set_points;
	
	printf("Points One(x and y)��");
	scanf("%lf%lf",&temp.x,&temp.y);
	set_points.push_back(temp);
	printf("Points Two(x and y)��");
	scanf("%lf%lf",&temp.x,&temp.y);
	set_points.push_back(temp);
	printf("Points Three(x and y)��");
	scanf("%lf%lf",&temp.x,&temp.y);
	set_points.push_back(temp);
	
	this->set(mass,set_points);
	mass=1.0/6.0*PI*abs(r*r*r)*7900;
	this->set_mass(mass);//��ֹ����С�ڵ���0
	I=I*mass;
}


void Triangle::reset_points()
{
	int i;
	for(i=0;i<3;i++)
	{
		points[i]=rotate_vector(ori_vectors[i],direction)+xy;	
	}
	for(i=0;i<3;i++)
	{
		normal_vectors[i]=unit_vector(rotate_anticlockwise(points[i]-points[(i+1)%3]));
	}
}


void Rectangle::set(double mass,double len,double wid,XY center,double direct)
{	
	set_mass(mass);//��ֹ����С�ڵ���0
	if(len<0||wid<0)
	{
		len=abs(len);
		wid=abs(wid);
		printf("Warning!Reset:length=%lf,width=%lf",len,wid);
	}
	vector<XY> points_set;
	length=len;
	width=wid;
	xy=center;
	direction=direct;
	r=0.5*sqrt(len*len+wid*wid);//���°뾶
	max_r=r;

	int i;
	for(i=0;i<4;i++)
	{
		points.push_back(center);
	}
	XY normal_vector1,normal_vector2;
	normal_vector1.x=cos(direct);
	normal_vector1.y=sin(direct);
	normal_vector2.x=-sin(direct);
	normal_vector2.y=cos(direct);
	normal_vectors.push_back(normal_vector1);//��λ����
	normal_vectors.push_back(normal_vector2);//��λ����

	XY delta_xy1,delta_xy2;
	delta_xy1.x=0.5*cos(direct)*len;
	delta_xy1.y=0.5*sin(direct)*len;
	delta_xy2.x=-0.5*sin(direct)*wid;
	delta_xy2.y=0.5*cos(direct)*wid;
	//������Щ�㱣֤��˳��Ϊʱ��˳������ĳЩ�㷨��ȷ�Ե�ǰ��
	points[0]=points[0]+delta_xy1+delta_xy2;
	points[1]=points[1]+delta_xy1-delta_xy2;
	points[2]=points[2]-delta_xy1-delta_xy2;
	points[3]=points[3]-delta_xy1+delta_xy2;

	I=m*(len*len+wid*wid)/12;//����ת��������ʽ

}
int Rectangle::is_inside(XY pt)//�жϵ��Ƿ���ͼ������򷵻�1�����㷨��ȷ��������Ϊ͹������ҵ㼯��ʱ��˳������
{
	if(normal(pt-xy)>=max_r)
		return 0;
	int i;
	XY temp;
	vector<XY> line1(points.begin(),points.begin()+2),line2(points.begin()+2,points.begin()+4);//����㼯˳����ʱ��˳���򽻻���λ��ʹ֮����
	if(is_cross(line1,line2))
	{
		printf("Rectangle\'s points in bad order!\n");
		temp=points[1];
		points[1]=points[2];
		points[2]=temp;
	}

	int sgn_sum=0;
	vector<XY> side(4),pt_to_points(4);//sideΪ������
	for(i=0;i<4;i++)
	{
		side[i]=points[(i+1)%4]-points[i];
		pt_to_points[i]=pt-points[i];
		sgn_sum+=sgn(det_2(side[i],pt_to_points[i]));
	}
	if(sgn_sum==4||sgn_sum==-4)
		return 1;
	else
		return 0;
	

}
void Rectangle::init_cmd()
{
	youngs_modulus=1.8e11;
	f.x=0;
	f.y=0;
	a.x=0;
	a.y=0;
	omega_a=0;
	printf("Electric charge(���)��");
	scanf("%lf",&q);
	printf("Velocity(x and y)��");
	scanf("%lf%lf",&v.x,&v.y);
	printf("Angular velocity��");
	scanf("%lf",&omega);

	double mass;
	double len;
	double wid;
	XY center;
	double direct;

	printf("Mass��");
	scanf("%lf",&mass);
	printf("Lenth��");
	scanf("%lf",&len);
	printf("Width��");
	scanf("%lf",&wid);
	printf("Center(x and y)��");
	scanf("%lf%lf",&center.x,&center.y);
	printf("Direction(0-2*pi)��");
	scanf("%lf",&direct);

	this->set(mass,len,wid,center,direct);
	this->reset_force();
}
void Rectangle::copy_var_rect(class Rectangle &rect2)//�������б���ֵ
{
	this->copy_basic_var(rect2);
	int i;
	for(i=0;i<4;i++)
	{
		rect2.points[i]=points[i];
		
	}
	rect2.length=length;
	rect2.width=width;
	rect2.max_r=max_r;
	rect2.normal_vectors[0]=normal_vectors[0];
	rect2.normal_vectors[1]=normal_vectors[1];
}
void terRectangle::init_cmd_ter()
{
	youngs_modulus=1.8e11;
	f.x=0;
	f.y=0;
	a.x=0;
	a.y=0;
	omega_a=0;
	q=0;
	v.x=0;v.y=0;
	omega=0;

	double mass;
	double len;
	double wid;
	XY center;
	double direct;

	printf("Lenth��");
	scanf("%lf",&len);
	printf("Width��");
	scanf("%lf",&wid);
	printf("Center(x and y)��");
	scanf("%lf%lf",&center.x,&center.y);
	printf("Direction(0-2*Pi)��");
	scanf("%lf",&direct);
	mass=1.0/6.0*PI*sqrt(len*len+wid*wid)*sqrt(len*len+wid*wid)*sqrt(len*len+wid*wid)*7900;

	this->set(mass,len,wid,center,direct);
	
}


void Rectangle::reset_points()//������ŷ�����ֺ󣬷��������ǶԵ�
{
	
	int i;
	for(i=0;i<4;i++)
	{
		points[i]=xy;
	}
	XY delta_xy1,delta_xy2;
	delta_xy1.x=0.5*cos(direction)*length;
	delta_xy1.y=0.5*sin(direction)*length;
	delta_xy2.x=-0.5*sin(direction)*width;
	delta_xy2.y=0.5*cos(direction)*width;
	//������Щ�㱣֤��˳��Ϊʱ��˳������ĳЩ�㷨��ȷ�Ե�ǰ��
	points[0]=points[0]+delta_xy1+delta_xy2;
	points[1]=points[1]+delta_xy1-delta_xy2;
	points[2]=points[2]-delta_xy1-delta_xy2;
	points[3]=points[3]-delta_xy1+delta_xy2;
	normal_vectors[0].x=cos(direction);
	normal_vectors[0].y=sin(direction);
	normal_vectors[1].x=-sin(direction);
	normal_vectors[1].y=cos(direction);
}



int Plane::collision_detect(XY xy1,XY xy2)//������������Ƿ񴩹�ƽ��
{
	vector<XY> line;
	line.push_back(xy1);
	line.push_back(xy2);
	if(is_cross(points,line)==1)
		return 1;
	else
		return 0;
	/*if(abs(a*xy1.x+b*xy1.y+c)>sqrt(a*a+b*b)*EPSILON&&abs(a*xy2.x+b*xy2.y+c)<sqrt(a*a+b*b)*EPSILON)
		return 1;
	else
	{
		if((a*xy1.x+b*xy1.y+c)*(a*xy2.x+b*xy2.y+c)<-(a*a+b*b)*EPSILON*EPSILON)
			return 1;
	}
	return 0;*/ //����
}

void Plane::set(XY pt1,XY pt2)
{
	points.push_back(pt1);
	points.push_back(pt2);
	a=pt1.y-pt2.y;
	b=pt2.x-pt1.x;
	c=pt1.x*pt2.y-pt2.x*pt1.y;
	length=normal(pt2-pt1);
}
void Plane::init_cmd()
{
	youngs_modulus=1.8e11;//����ģ��
	XY pt1,pt2;
	printf("Points One(x and y)��");
	scanf("%lf%lf",&pt1.x,&pt1.y);
	printf("Points Two(x and y)��");
	scanf("%lf%lf",&pt2.x,&pt2.y);
	this->set(pt1,pt2);
}
void Plane::Display()
{
	line(magnify_ratio*(points[0].x+moveX),magnify_ratio*(points[0].y+moveY),magnify_ratio*(points[1].x+moveX),magnify_ratio*(points[1].y+moveY));
}