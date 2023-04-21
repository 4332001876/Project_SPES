#include "basic_elements.h"
#include "calculation.h"



void SPACE::init_cmd()
{
	printf("CMD:Group 1:star(add these elements)\n\tGroup 2:finish\n");
	char buffer[501] = {'\0'}, cmd[501] = {'\0'};
    while (1)
    {
		printf("cmd>");
		fgets(buffer, 500, stdin);
        if (isalnum(buffer[0]) == 0)
        {
            clear(buffer, cmd, 501);
            fgets(buffer, 500, stdin);
        }
        sscanf(buffer, "%s", cmd);
	
		if(strcmp(cmd,"star")==0)
		{
			this->add_element<class Star>(star);
		}
		
		if(strcmp(cmd,"finish")==0)
		{
			break;
		}
	}
}
template <typename T> void SPACE::add_element(vector<T> &obj)
{
	T temp;
	temp.init_cmd();
	obj.push_back(temp);
}
template <typename T> void SPACE::add_element_ter(vector<T> &obj)
{
	T temp;
	temp.init_cmd_ter();
	obj.push_back(temp);
}

void SPACE::cal_launch()//初始化新时步计算，受力加速度置零
{
	int i,j;
	for(i=0;i<star.size();i++)
	{
		star[i].reset_force();
	}
	for(i=0;i<star.size();i++)
	{
		
		for(j=i+1;j<star.size();j++)
		{
			star[i].f=star[i].f+1e-6*(star[j].xy-star[i].xy)*(6.67e-11*star[i].m*star[j].m/((normal(star[i].xy-star[j].xy))*(normal(star[i].xy-star[j].xy))*(normal(star[i].xy-star[j].xy))));
			star[j].f=star[j].f-1e-6*(star[j].xy-star[i].xy)*(6.67e-11*star[i].m*star[j].m/((normal(star[i].xy-star[j].xy))*(normal(star[i].xy-star[j].xy))*(normal(star[i].xy-star[j].xy))));
		}
	}
	/*
	for(i=0;i<cir.size();i++)
	{
		cir[i].reset_force();
		cir[i].f.y=cir[i].f.y-G_FORCE*cir[i].m;
	}
	for(i=0;i<tri.size();i++)
	{
		tri[i].reset_force();
		tri[i].f.y=tri[i].f.y-G_FORCE*tri[i].m;
	}
	for(i=0;i<rect.size();i++)
	{
		rect[i].reset_force();
		rect[i].f.y=rect[i].f.y-G_FORCE*rect[i].m;
	}*/
}
void SPACE::cal_main(double time_step)
{
	this->cal_launch();
	int i,j;
	/*for(i=0;i<cir.size();i++)
	{
		for(j=i+1;j<cir.size();j++)
			circle_circle_detect_collision(cir[i],cir[j]);
		for(j=0;j<tri.size();j++)
			circle_poly_detect_collision<class Triangle>(cir[i],tri[j]);
		for(j=0;j<rect.size();j++)
			circle_poly_detect_collision<class Rectangle>(cir[i],rect[j]);
		for(j=0;j<plane.size();j++)
			plane_circle_detect_collision(plane[j],cir[i]);
		//新地形
		for(j=0;j<tercir.size();j++)
			circle_circle_detect_collision(cir[i],tercir[j]);
		for(j=0;j<tertri.size();j++)
			circle_poly_detect_collision<class terTriangle>(cir[i],tertri[j]);
		for(j=0;j<terrect.size();j++)
			circle_poly_detect_collision<class terRectangle>(cir[i],terrect[j]);
	}
	for(i=0;i<tri.size();i++)
	{
		for(j=i+1;j<tri.size();j++)
		{
			poly_poly_detect_collision<class Triangle,class Triangle>(tri[i],tri[j]);
			//poly_poly_detect_collision<class Triangle,class Triangle>(tri[j],tri[i]);
		}
		for(j=0;j<rect.size();j++)
		{
			poly_poly_detect_collision<class Triangle,class Rectangle>(tri[i],rect[j]);
			//poly_poly_detect_collision<class Rectangle,class Triangle>(rect[j],tri[i]);
		}
		for(j=0;j<plane.size();j++)
			plane_poly_detect_collision<class Triangle>(plane[j],tri[i]);
		//新地形
		for(j=0;j<tercir.size();j++)
			circle_poly_detect_collision<class Triangle>(tercir[j],tri[i]);
		for(j=0;j<tertri.size();j++)
		{
			poly_poly_detect_collision<class Triangle,class terTriangle>(tri[i],tertri[j]);
			//poly_poly_detect_collision<class terTriangle,class Triangle>(tertri[j],tri[i]);
		}
		for(j=0;j<terrect.size();j++)
		{
			poly_poly_detect_collision<class Triangle,class terRectangle>(tri[i],terrect[j]);
			//poly_poly_detect_collision<class terRectangle,class Triangle>(terrect[j],tri[i]);
		}
	}
	for(i=0;i<rect.size();i++)
	{
		for(j=i+1;j<rect.size();j++)
		{
			poly_poly_detect_collision<class Rectangle,class Rectangle>(rect[i],rect[j]);
			//poly_poly_detect_collision<class Rectangle,class Rectangle>(rect[j],rect[i]);
		}
		for(j=0;j<plane.size();j++)
			plane_poly_detect_collision<class Rectangle>(plane[j],rect[i]);
		//新地形
		for(j=0;j<tercir.size();j++)
			circle_poly_detect_collision<class Rectangle>(tercir[j],rect[i]);
		for(j=0;j<tertri.size();j++)
		{
			poly_poly_detect_collision<class Rectangle,class terTriangle>(rect[i],tertri[j]);
			//poly_poly_detect_collision<class terTriangle,class Rectangle>(tertri[j],rect[i]);
		}
		for(j=0;j<terrect.size();j++)
		{
			poly_poly_detect_collision<class Rectangle,class terRectangle>(rect[i],terrect[j]);
			//poly_poly_detect_collision<class terRectangle,class Rectangle>(terrect[j],rect[i]);
		}
	}*/
	this->EulerCalculus(time_step);
}
void SPACE::EulerCalculus(double time_step)//专门拿来算欧拉积分的，为RK4服务
{
	
	int i;
	for(i=0;i<star.size();i++)
	{
		star[i].EulerCalculus(time_step);
	}
	for(i=0;i<cir.size();i++)
	{
		cir[i].EulerCalculus(time_step);
	}
	for(i=0;i<tri.size();i++)
	{
		tri[i].EulerCalculus(time_step);
		tri[i].reset_points();
	}
	for(i=0;i<rect.size();i++)
	{
		rect[i].EulerCalculus(time_step);
		rect[i].reset_points();
	}
}
void SPACE::RK4(double time_step,class SPACE &space2)//RK4积分
{
	this->cal_launch();
	int i,j;
	this->copy_status(space2);

	space2.cal_main(0.5*time_step);//受力K1，状态为K2输入
	this->copy_force(space2,1);//计算结果存进space里

	space2.cal_main(0.5*time_step);//受力K2
	this->copy_force(space2,2);
	this->copy_status(space2);
	space2.EulerCalculus(0.5*time_step);//状态为K3输入

	space2.cal_main(time_step);//受力K3
	this->copy_force(space2,3);
	this->copy_status(space2);
	space2.EulerCalculus(time_step);//状态为K4输入

	space2.cal_main(time_step);//受力K4
	this->copy_force(space2,4);

	for(i=0;i<star.size();i++)
	{
		star[i].RK4_cal_force();
	}

	for(i=0;i<cir.size();i++)
	{
		cir[i].RK4_cal_force();
	}
	for(i=0;i<tri.size();i++)
	{
		tri[i].RK4_cal_force();	
	}
	for(i=0;i<rect.size();i++)
	{
		rect[i].RK4_cal_force();
	}
	this->EulerCalculus(time_step);
}



void SPACE::copy_status(class SPACE &space2)
{
	int i;
	for(i=0;i<star.size();i++)
	{
		star[i].copy_var_cir(space2.star[i]);
		
	}

	for(i=0;i<cir.size();i++)
	{
		cir[i].copy_var_cir(space2.cir[i]);
		
	}
	for(i=0;i<tri.size();i++)
	{
		tri[i].copy_var_tri(space2.tri[i]);
	}
	for(i=0;i<rect.size();i++)
	{
		rect[i].copy_var_rect(space2.rect[i]);
	}
}
void SPACE::copy_force(class SPACE &space2,int step)//RK4积分第n步时复制力至缓存区
{
	int i;
	for(i=0;i<star.size();i++)
	{
		star[i].temp_f[step-1]=space2.star[i].f;
		star[i].temp_moment[step-1]=space2.star[i].moment;
	}
	for(i=0;i<cir.size();i++)
	{
		cir[i].temp_f[step-1]=space2.cir[i].f;
		cir[i].temp_moment[step-1]=space2.cir[i].moment;
	}
	for(i=0;i<tri.size();i++)
	{
		tri[i].temp_f[step-1]=space2.tri[i].f;
		tri[i].temp_moment[step-1]=space2.tri[i].moment;
	}
	for(i=0;i<rect.size();i++)
	{
		rect[i].temp_f[step-1]=space2.rect[i].f;
		rect[i].temp_moment[step-1]=space2.rect[i].moment;
	}
}


void SPACE::generate_graph()//输出整个SPACE的图形化界面
{
	int i,j;
	BeginBatchDraw();
	cleardevice();
	for(i=0;i<star.size();i++)
		star[i].Display();
	for(i=0;i<cir.size();i++)
		cir[i].Display();
	for(i=0;i<tri.size();i++)
		tri[i].Display();
	for(i=0;i<rect.size();i++)
		rect[i].Display();
	for(i=0;i<plane.size();i++)
		plane[i].Display();

	for(j=0;j<tercir.size();j++)
		tercir[j].terDisplay();
	for(j=0;j<tertri.size();j++)
		tertri[j].terDisplay();
	for(j=0;j<terrect.size();j++)
		terrect[j].terDisplay();

	EndBatchDraw();
}




void circle_circle_detect_collision(class Circle &cir1,class Circle &cir2)//改变输入物体的受力
{
	
	if(normal(cir1.xy-cir2.xy)<cir1.r+cir2.r)
	{
		

		XY fn1;
		double dep;//深度
		dep=cir1.r+cir2.r-normal(cir1.xy-cir2.xy);
		XY vector_n=(1/(normal(cir1.xy-cir2.xy)))*(cir1.xy-cir2.xy);//单位化后的法向量
		//弹力,法向力矩为0
		XY v1,v2;
		v1=cir1.v+cir1.omega*rotate_anticlockwise(-vector_n*cir1.r);
		v2=cir2.v+cir2.omega*rotate_anticlockwise(vector_n*cir2.r);
		//v1-v2为以v2代表物体为参考系的v1速度,v2-v1为以v1为参考系的v2速度
		double v_relative=normal(v2-v1);
		double a_limit=max(10000,v_relative*v_relative*0.5);//数据基于论文,限制加速度

		fn1=vector_n*((cir1.youngs_modulus*cir2.youngs_modulus/cir1.r/cir2.r)/(cir1.youngs_modulus/cir1.r+cir2.youngs_modulus/cir2.r)*dep*(4*dep*dep*PI));//压强*面积
		if(normal(fn1)>a_limit*cir1.m)
			fn1=a_limit*cir1.m*unit_vector(fn1);
		if(normal(fn1)>a_limit*cir2.m)
			fn1=a_limit*cir2.m*unit_vector(fn1);

		cir1.f=cir1.f+ fn1;
		cir2.f=cir2.f- fn1;//+=没有给向量重载，怕出BUG
		
		
		
		cir1.f=cir1.f-min(cir1.m,cir2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
		cir2.f=cir2.f-min(cir1.m,cir2.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n;

		//摩擦力，不考虑摩擦力小于最大摩擦力的情况（这种实现不会有问题，否则很难实现）
		double fn=normal(fn1-min(cir1.m,cir2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);//弹力大小
		XY unit_fric1=unit_vector((v2-v1)-dot(v2-v1,vector_n)*vector_n);//cir1受到的摩擦力的方向向量
		XY unit_fric2=-unit_fric1;
		cir1.f=cir1.f+friction_factor(cir1.material,cir2.material)*fn*unit_fric1;
		cir1.moment+=cross(-vector_n*cir1.r,friction_factor(cir1.material,cir2.material)*fn*unit_fric1);
		cir2.f=cir2.f+friction_factor(cir1.material,cir2.material)*fn*unit_fric2;
		cir2.moment+=cross(vector_n*cir2.r,friction_factor(cir1.material,cir2.material)*fn*unit_fric2);
	}

}


template <typename T> void circle_poly_detect_collision(class Circle &cir,T &poly)
{
	if(normal(cir.xy-poly.xy)>cir.r+poly.max_r)
		return;
	double dep;//深度
	int i;
	XY vector_n;//法向量
	XY v1,v2;//两物体速度
	XY fn1;//物体1杨氏模量弹力部分
	double fn;//弹力大小
	XY unit_fric1,unit_fric2;//摩擦力的方向向量

	

	//第一类，多边形点嵌入圆内
	for(i=0;i<poly.points.size();i++)
	{
		if(cir.is_inside(poly.points[i]))
		{
			dep=cir.r-normal(poly.points[i]-cir.xy);
			vector_n=unit_vector(poly.points[i]-cir.xy);//单位化后的法向量,由圆心指向嵌入点
			//弹力，沿径向（法向量方向），对圆力矩为0

			v1=cir.v+cir.omega*rotate_anticlockwise(vector_n*(cir.r-dep));
			v2=poly.v+poly.omega*rotate_anticlockwise(poly.points[i]-poly.xy);
			//v1-v2为以速度v2的物体为参考系的v1速度,v2-v1为以v1为参考系的v2速度
			double v_relative=normal(v2-v1);
			double a_limit=max(10000,v_relative*v_relative*0.5);//数据基于论文,限制加速度

			

			fn1=vector_n*((cir.youngs_modulus*poly.youngs_modulus/cir.r/poly.r)/(cir.youngs_modulus/cir.r+poly.youngs_modulus/poly.r)*dep*(4*dep*dep*PI));//压强*面积
			if(normal(fn1)>a_limit*cir.m)
				fn1=a_limit*cir.m*unit_vector(fn1);
			if(normal(fn1)>a_limit*poly.m)
				fn1=a_limit*poly.m*unit_vector(fn1);
			cir.f=cir.f - fn1;
			poly.f=poly.f + fn1;//+=没有给向量重载，怕出BUG
			poly.moment+=cross(poly.points[i]-poly.xy,fn1);

			
			
			cir.f=cir.f-min(cir.m,poly.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
			poly.f=poly.f-min(cir.m,poly.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n;
			poly.moment+=cross(poly.points[i]-poly.xy,-min(cir.m,poly.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n);

			//摩擦力，不考虑摩擦力小于最大摩擦力的情况（这种实现不会有问题，否则很难实现）
			fn=normal(-fn1-min(cir.m,poly.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);//弹力大小
			unit_fric1=unit_vector((v2-v1)-dot(v2-v1,vector_n)*vector_n);//cir受到的摩擦力的方向向量
			unit_fric2=-unit_fric1;
			cir.f=cir.f+friction_factor(cir.material,poly.material)*fn*unit_fric1;
			cir.moment+=cross(vector_n*(cir.r-dep),friction_factor(cir.material,poly.material)*fn*unit_fric1);
			poly.f=poly.f+friction_factor(cir.material,poly.material)*fn*unit_fric2;
			poly.moment+=cross(poly.points[i]-poly.xy,friction_factor(cir.material,poly.material)*fn*unit_fric2);


		}
	}
	XY ver_pt;//垂直点
	XY pt1,pt2;//临时存放点坐标的变量,以简化代码
	//第二类，多边形边（线段，不是直线！）与圆有两个交点
	for(i=0;i<poly.points.size();i++)
	{
		pt1=poly.points[i];
		pt2=poly.points[(i+1)%poly.points.size()];
		if(is_circle_line_intersect(pt1,pt2,cir.xy,cir.r))
		{
			
			ver_pt=vertical_point(pt1,pt2,cir.xy);//作垂直点
			dep=cir.r-normal(ver_pt-cir.xy);//深度
			vector_n=unit_vector(rotate_anticlockwise(pt2-pt1));//单位化后的法向量
			if(normal(ver_pt+ 0.5*normal(ver_pt-cir.xy) *vector_n-cir.xy)>normal(ver_pt-cir.xy))//让法向量由嵌入点指向圆心
			{
				vector_n=-vector_n;
			}

			//弹力，沿径向（法向量方向），对圆力矩为0
			v1=cir.v+cir.omega*rotate_anticlockwise((-vector_n)*(cir.r-dep));
			v2=poly.v+poly.omega*rotate_anticlockwise(ver_pt-poly.xy);
			//v1-v2为以速度v2的物体为参考系的v1速度,v2-v1为以v1为参考系的v2速度
			double v_relative=normal(v2-v1);
			double a_limit=max(10000,v_relative*v_relative*0.5);//数据基于论文,限制加速度

			

			fn1=vector_n*((cir.youngs_modulus*poly.youngs_modulus/cir.r/poly.r)/(cir.youngs_modulus/cir.r+poly.youngs_modulus/poly.r)*dep*(4*dep*dep*PI));//压强*面积
			if(normal(fn1)>a_limit*cir.m)
				fn1=a_limit*cir.m*unit_vector(fn1);
			if(normal(fn1)>a_limit*poly.m)
				fn1=a_limit*poly.m*unit_vector(fn1);
			cir.f=cir.f+ fn1;
			poly.f=poly.f - fn1;//+=没有给向量重载，怕出BUG
			poly.moment+=cross(ver_pt-poly.xy,-fn1);

			
			
			cir.f=cir.f-min(cir.m,poly.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
			poly.f=poly.f-min(cir.m,poly.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n;
			poly.moment+=cross(ver_pt-poly.xy,-min(cir.m,poly.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n);

			//摩擦力，不考虑摩擦力小于最大摩擦力的情况（这种实现不会有问题，否则很难实现）
			fn=normal(fn1-min(cir.m,poly.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);//弹力大小
			unit_fric1=unit_vector((v2-v1)-dot(v2-v1,vector_n)*vector_n);//cir受到的摩擦力的方向向量
			unit_fric2=-unit_fric1;
			cir.f=cir.f+friction_factor(cir.material,poly.material)*fn*unit_fric1;
			cir.moment+=cross((-vector_n)*(cir.r-dep),friction_factor(cir.material,poly.material)*fn*unit_fric1);
			poly.f=poly.f+friction_factor(cir.material,poly.material)*fn*unit_fric2;
			poly.moment+=cross(ver_pt-poly.xy,friction_factor(cir.material,poly.material)*fn*unit_fric2);


		}
	}

}

template <typename T1,typename T2> void poly_poly_detect_collision(T1 &poly1,T2 &poly2)//这个新poly_poly不需要成对出现
{
	if(normal(poly1.xy-poly2.xy)>poly1.max_r+poly2.max_r)
		return;
	//详情百度多边形碰撞检测（与响应）算法
	//拷贝poly1和poly2所有边法向量
	static XY side_n_vector[6];//不想建vector,时间开销太大
	int count_side_n_vector=poly1.normal_vectors.size()+poly2.normal_vectors.size();
	int i,j;

	for(i=0;i<poly1.normal_vectors.size();i++)
	{
		side_n_vector[i]=poly1.normal_vectors[i];
	}
	for(i=0;i<poly2.normal_vectors.size();i++)
	{
		side_n_vector[i+poly1.normal_vectors.size()]=poly2.normal_vectors[i];
	}
	//拷贝边法向量完成

	double dot_value;
	double projection[6][4]={{0}};//投影点一维（流形）坐标,以poly1.points[0]为基准坐标0，4列为poly1最大点与最小点坐标、poly2最大点与最小点坐标
	int max_min_point[6][4]={{0}};//4列为poly1最大点与最小点下标、poly2最大点与最小点下标
	for(i=0;i<count_side_n_vector;i++)
	{
		//poly1
		
		for(j=1;j<poly1.points.size();j++)
		{
			dot_value=dot(side_n_vector[i],poly1.points[j]-poly1.points[0]);
			if(dot_value>projection[i][0])
			{
				projection[i][0]=dot_value;
				max_min_point[i][0]=j;
			}
			if(dot_value<projection[i][1])
			{
				projection[i][1]=dot_value;
				max_min_point[i][1]=j;
			}
		}

		//poly2
		dot_value=dot(side_n_vector[i],poly2.points[0]-poly1.points[0]);
		projection[i][2]=dot_value;
		projection[i][3]=dot_value;
		for(j=1;j<poly2.points.size();j++)
		{
			dot_value=dot(side_n_vector[i],poly2.points[j]-poly1.points[0]);
			if(dot_value>projection[i][2])
			{
				projection[i][2]=dot_value;
				max_min_point[i][2]=j;
			}
			if(dot_value<projection[i][3])
			{
				projection[i][3]=dot_value;
				max_min_point[i][3]=j;
			}
		}
	}
	//已装完投影信息,开始判定
	double min_dep,temp_dep_for_min;
	int min_dep_vector=0;
	for(i=0;i<count_side_n_vector;i++)
	{
		if(projection[i][3]>=projection[i][0]||projection[i][1]>=projection[i][2])//当存在line1最小值大于等于line2最大值时,不重叠,即不碰撞
			return;
		temp_dep_for_min=min(projection[i][0]-projection[i][3],projection[i][2]-projection[i][1]);//必大于0
		if(i==0)
		{
			min_dep=temp_dep_for_min;
		}
		/*if(projection[i][0]>=projection[i][2]&&projection[i][1]<=projection[i][3])//线段包含线段,好像这种情况要一样处理
		{
			continue;//不处理这种情况
		}
		if(projection[i][0]<=projection[i][2]&&projection[i][1]>=projection[i][3])//线段包含线段
		{
			continue;//不处理这种情况
		}*/
		
		
		if(temp_dep_for_min<min_dep)
		{
			min_dep=temp_dep_for_min;
			min_dep_vector=i;
		}

	}
	//已获得最短投影长度min_dep及对应法向量下标min_dep_vector

	XY poly1_pt,poly2_pt;
	if(projection[min_dep_vector][0]-projection[min_dep_vector][3]>projection[min_dep_vector][2]-projection[min_dep_vector][1])
	{
		poly1_pt=poly1.points[max_min_point[min_dep_vector][1]];
		poly2_pt=poly2.points[max_min_point[min_dep_vector][2]];
	}
	else
	{
		poly1_pt=poly1.points[max_min_point[min_dep_vector][0]];
		poly2_pt=poly2.points[max_min_point[min_dep_vector][3]];
	}

	//开始碰撞响应
	
	double dep=min_dep;
	XY vector_n= side_n_vector[min_dep_vector];//已经是单位向量了
	int flag_inside=0;
	if(poly2.is_inside(poly1_pt))//多边形poly1点嵌入多边形poly2中
	{
		flag_inside=1;
		if(dot(vector_n,poly2_pt-poly1_pt)<0)
			vector_n=-vector_n;
		poly_poly_collision_respond<T1,T2>(poly1,poly2,poly1_pt,dep,vector_n);//vector_n为poly1受弹力方向
	}	
	if(poly1.is_inside(poly2_pt))//多边形poly2点嵌入多边形poly1中
	{
		flag_inside=1;
		if(dot(vector_n,poly1_pt-poly2_pt)<0)
			vector_n=-vector_n;
		poly_poly_collision_respond<T2,T1>(poly2,poly1,poly2_pt,dep,vector_n);//vector_n为poly2受弹力方向
	}	
	if(flag_inside==0)
	{
		static vector<XY> cross_points;
		cross_points.clear();
		for(i=0;i<poly1.points.size();i++)
		{
			for(j=0;j<poly2.points.size();j++)
			{
				if(is_cross(poly1.points[i],poly1.points[(i+1)%poly1.points.size()], poly2.points[j],poly2.points[(j+1)%poly2.points.size()]))
				{
					cross_points.push_back(cross_point(poly1.points[i],poly1.points[(i+1)%poly1.points.size()], poly2.points[j],poly2.points[(j+1)%poly2.points.size()]));
				}
			}
		}
		XY average_points;
		average_points.x=0;
		average_points.y=0;
		if(cross_points.size()!=0)
		{
			for(i=0;i<cross_points.size();i++)
			{
				average_points=average_points+cross_points[i];
			}
		
			average_points=average_points*(1.0/cross_points.size());
			if(dot(vector_n,poly2_pt-poly1_pt)<0)
				vector_n=-vector_n;
			poly_poly_collision_respond<T1,T2>(poly1,poly2,average_points,dep,vector_n);//vector_n为poly1受弹力方向
		}
		else
		{
			
			if(dot(vector_n,poly2_pt-poly1_pt)<0)//旧解决方案
				vector_n=-vector_n;
			poly_poly_collision_respond<T1,T2>(poly1,poly2,poly1_pt,dep,vector_n);//vector_n为poly1受弹力方向
			if(dot(vector_n,poly1_pt-poly2_pt)<0)
				vector_n=-vector_n;
			poly_poly_collision_respond<T2,T1>(poly2,poly1,poly2_pt,dep,vector_n);//vector_n为poly2受弹力方向
		}
		
	}
		
		
			
}
template <typename T1,typename T2> void poly_poly_collision_respond(T1 &poly1,T2 &poly2,XY pt,double dep,XY vector_n)//pt是碰撞点,vector_n为poly1受弹力方向
{		
		XY v1,v2;
		v1=poly1.v+poly1.omega*rotate_anticlockwise(pt-poly1.xy);
		v2=poly2.v+poly2.omega*rotate_anticlockwise(pt-poly2.xy);
		//v1-v2为以速度v2的物体为参考系的v1速度,v2-v1为以v1为参考系的v2速度
		double v_relative=normal(v2-v1);
		double a_limit=max(10000,v_relative*v_relative*0.5);//数据基于论文,限制加速度
		XY fn1;//弹力，沿径向（法向量方向）
		fn1=vector_n*((poly1.youngs_modulus*poly2.youngs_modulus/poly1.r/poly2.r)/(poly1.youngs_modulus/poly1.r+poly2.youngs_modulus/poly2.r)*dep*(4*dep*dep*PI));//压强*面积
		if(normal(fn1)>a_limit*poly1.m)
			fn1=a_limit*poly1.m*unit_vector(fn1);
		if(normal(fn1)>a_limit*poly2.m)
			fn1=a_limit*poly2.m*unit_vector(fn1);
		poly1.f=poly1.f+ fn1;
		poly2.f=poly2.f- fn1;//+=没有给向量重载，怕出BUG
		poly1.moment+=cross(pt-poly1.xy,fn1);
		poly2.moment+=cross(pt-poly2.xy,-fn1);

		
		poly1.f=poly1.f-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
		poly2.f=poly2.f-min(poly1.m,poly2.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n;
		poly1.moment+=cross(pt-poly1.xy,-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);
		poly2.moment+=cross(pt-poly2.xy,-min(poly1.m,poly2.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n);


		XY unit_fric1,unit_fric2;
		//摩擦力，不考虑摩擦力小于最大摩擦力的情况（这种实现不会有问题，否则很难实现）
		double fn=normal(fn1-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);//弹力大小
		unit_fric1=unit_vector((v2-v1)-dot(v2-v1,vector_n)*vector_n);//poly1受到的摩擦力的方向向量
		unit_fric2=-unit_fric1;
		poly1.f=poly1.f+friction_factor(poly1.material,poly2.material)*fn*unit_fric1;
		poly1.moment+=cross(pt-poly1.xy,friction_factor(poly1.material,poly2.material)*fn*unit_fric1);
		poly2.f=poly2.f+friction_factor(poly1.material,poly2.material)*fn*unit_fric2;
		poly2.moment+=cross(pt-poly2.xy,friction_factor(poly1.material,poly2.material)*fn*unit_fric2);
}



/*template <typename T1,typename T2> void old_poly_poly_detect_collision(T1 &poly1,T2 &poly2)//poly_poly必须成对出现！
{
	if(normal(poly1.xy-poly2.xy)>poly1.max_r+poly2.max_r)
		return;
	double dep,dep_temp;//深度
	int i,j,k;
	XY vector_n;//法向量
	XY v1,v2;//两物体速度
	XY fn1;//物体1弹力杨氏模量部分
	double fn;//弹力大小
	XY unit_fric1,unit_fric2;//摩擦力的方向向量
	XY ver_pt,ver_pt_temp;//垂直点
	XY pt,pt1,pt2,pt1_temp,pt2_temp,pt_side1,pt_side2;//临时存放点坐标的变量,以简化代码
	int flag_cross[4]={0};
	int count_inside=0;
	
	for(i=0;i<poly1.points.size();i++)
	{
		pt=poly1.points[i];
		pt_side1=poly1.points[(i+poly1.points.size()-1)%poly1.points.size()];
		pt_side2=poly1.points[(i+1)%poly1.points.size()];
		if(poly2.is_inside(pt))//多边形poly1点嵌入多边形poly2中
		{
			count_inside++;
			
			dep=poly2.r*2;
			dep_temp=0;
			for(j=0;j<poly2.points.size();j++)
			{
				pt1_temp=poly2.points[j];
				pt2_temp=poly2.points[(j+1)%poly2.points.size()];
				ver_pt_temp=vertical_point(pt1_temp,pt2_temp,pt);//作垂直点
				dep_temp=normal(pt-ver_pt_temp);
				if(is_cross(pt,pt_side1,pt1_temp,pt2_temp)&&is_cross(pt,pt_side2,pt1_temp,pt2_temp))
				{
					dep=dep_temp;
					ver_pt=ver_pt_temp;
					pt1=pt1_temp;
					pt2=pt2_temp;
					break;
				}
				if(dep_temp<=dep)
				{
					dep=dep_temp;
					ver_pt=ver_pt_temp;
					pt1=pt1_temp;
					pt2=pt2_temp;
				}
			}	
			
			vector_n=unit_vector(rotate_anticlockwise(pt2-pt1));//单位化后的法向量
			if(poly2.is_inside(ver_pt+ 0.5*dep *vector_n))//让法向量指向poly2外
			{
				vector_n=-vector_n;
			}
			//弹力，沿径向（法向量方向），对圆力矩为0
			fn1=vector_n*((poly1.youngs_modulus*poly2.youngs_modulus/poly1.r/poly2.r)/(poly1.youngs_modulus/poly1.r+poly2.youngs_modulus/poly2.r)*dep*(4*dep*dep*PI));//压强*面积
			poly1.f=poly1.f+ fn1;
			poly2.f=poly2.f- fn1;//+=没有给向量重载，怕出BUG
			poly1.moment+=cross(pt-poly1.xy,fn1);
			poly2.moment+=cross(pt-poly2.xy,-fn1);

			
			v1=poly1.v+poly1.omega*rotate_anticlockwise(pt-poly1.xy);
			v2=poly2.v+poly2.omega*rotate_anticlockwise(pt-poly2.xy);
			//v1-v2为以速度v2的物体为参考系的v1速度,v2-v1为以v1为参考系的v2速度
			poly1.f=poly1.f-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
			poly2.f=poly2.f-min(poly1.m,poly2.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n;
			poly1.moment+=cross(pt-poly1.xy,-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);
			poly2.moment+=cross(pt-poly2.xy,-min(poly1.m,poly2.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n);


			//摩擦力，不考虑摩擦力小于最大摩擦力的情况（这种实现不会有问题，否则很难实现）
			fn=normal(fn1-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);//弹力大小
			unit_fric1=unit_vector((v2-v1)-dot(v2-v1,vector_n)*vector_n);//poly1受到的摩擦力的方向向量
			unit_fric2=-unit_fric1;
			poly1.f=poly1.f+friction_factor(poly1.material,poly2.material)*fn*unit_fric1;
			poly1.moment+=cross(pt-poly1.xy,friction_factor(poly1.material,poly2.material)*fn*unit_fric1);
			poly2.f=poly2.f+friction_factor(poly1.material,poly2.material)*fn*unit_fric2;
			poly2.moment+=cross(pt-poly2.xy,friction_factor(poly1.material,poly2.material)*fn*unit_fric2);


		}
	}
	if(count_inside==0)
	{
		for(i=0;i<poly1.points.size();i++)
		{
			pt=poly1.points[i];
			pt_side1=poly1.points[(i+poly1.points.size()-1)%poly1.points.size()];
			pt_side2=poly1.points[(i+1)%poly1.points.size()];
	
			//pt没有嵌入多边形poly2中,但poly2的两临边与poly1的两临边相交
			{
				for(j=0;j<poly2.points.size();j++)
				{
					pt1_temp=poly2.points[j];
					pt2_temp=poly2.points[(j+1)%poly2.points.size()];
				
					if(is_cross(pt,pt_side1,pt1_temp,pt2_temp)&&is_cross(pt,pt_side2,pt1_temp,pt2_temp))
					{
						flag_cross[j]=1;
					}
				}
				for(j=0;j<poly2.points.size();j++)
				{
					if(flag_cross[j]&&flag_cross[(j+1)%poly2.points.size()])
					{
						for(k=0;k<2;k++)
						{
							pt1=poly2.points[(j+k)%poly2.points.size()];
							pt2=poly2.points[(j+k+1)%poly2.points.size()];
							ver_pt=vertical_point(pt1,pt2,pt);//作垂直点
							dep=normal(pt-ver_pt);
						

							vector_n=unit_vector(rotate_anticlockwise(pt2-pt1));//单位化后的法向量
							if(poly2.is_inside(ver_pt+ 0.5*dep *vector_n))//让法向量指向poly2外
							{
								vector_n=-vector_n;
							}
							//弹力，沿径向（法向量方向），对圆力矩为0
							fn1=vector_n*((poly1.youngs_modulus*poly2.youngs_modulus/poly1.r/poly2.r)/(poly1.youngs_modulus/poly1.r+poly2.youngs_modulus/poly2.r)*dep*(4*dep*dep*PI));//压强*面积
							poly1.f=poly1.f+ fn1;
							poly2.f=poly2.f- fn1;//+=没有给向量重载，怕出BUG
							poly1.moment+=cross(pt-poly1.xy,fn1);
							poly2.moment+=cross(pt-poly2.xy,-fn1);

			
							v1=poly1.v+poly1.omega*rotate_anticlockwise(pt-poly1.xy);
							v2=poly2.v+poly2.omega*rotate_anticlockwise(pt-poly2.xy);
							//v1-v2为以速度v2的物体为参考系的v1速度,v2-v1为以v1为参考系的v2速度
							poly1.f=poly1.f-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
							poly2.f=poly2.f-min(poly1.m,poly2.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n;
							poly1.moment+=cross(pt-poly1.xy,-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);
							poly2.moment+=cross(pt-poly2.xy,-min(poly1.m,poly2.m)*PD_FACTOR* dot(v2-v1,vector_n)*vector_n);


							//摩擦力，不考虑摩擦力小于最大摩擦力的情况（这种实现不会有问题，否则很难实现）
							fn=normal(fn1-min(poly1.m,poly2.m)*PD_FACTOR* dot(v1-v2,vector_n)*vector_n);//弹力大小
							unit_fric1=unit_vector((v2-v1)-dot(v2-v1,vector_n)*vector_n);//poly1受到的摩擦力的方向向量
							unit_fric2=-unit_fric1;
							poly1.f=poly1.f+friction_factor(poly1.material,poly2.material)*fn*unit_fric1;
							poly1.moment+=cross(pt-poly1.xy,friction_factor(poly1.material,poly2.material)*fn*unit_fric1);
							poly2.f=poly2.f+friction_factor(poly1.material,poly2.material)*fn*unit_fric2;
							poly2.moment+=cross(pt-poly2.xy,friction_factor(poly1.material,poly2.material)*fn*unit_fric2);
						}
					}
				}
			}
		}

	}
	

}*/

void plane_circle_detect_collision(class Plane &plain,class Circle &cir)
{
	XY ver_pt;
	int i;
	XY pt[2];
	pt[0]=plain.points[0];
	pt[1]=plain.points[1];
	ver_pt=vertical_point(pt[0],pt[1], cir.xy);
	if(normal(cir.xy-ver_pt)>cir.r)
		return;

	XY vector_n;//法向量
	
	double dep;//深度	
	XY v;//两物体速度
	XY fn1;//物体杨氏模量弹力部分
	double fn;//弹力大小
	XY unit_fric;//摩擦力的方向向量
	//第一类，地形边（线段，不是直线！）与圆有两个交点
	if(is_circle_line_intersect(pt[0],pt[1],cir.xy,cir.r))
	{
		
		vector_n=unit_vector(cir.xy-ver_pt);//单位化的法向量，方向向圆心
		dep=cir.r-normal(cir.xy-ver_pt);
		fn1=vector_n*((cir.youngs_modulus/cir.r)*dep*(4*dep*dep*PI));
		cir.f=cir.f+fn1;
		v=cir.v+cir.omega*rotate_anticlockwise(-vector_n*(cir.r-dep));
		cir.f=cir.f-cir.m*PD_FACTOR* dot(v,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
		fn=normal(fn1-cir.m*PD_FACTOR* dot(v,vector_n)*vector_n);//弹力大小
		unit_fric=-unit_vector(v-dot(v,vector_n)*vector_n);//cir受到的摩擦力的方向向量
		cir.f=cir.f+friction_factor(cir.material,plain.material)*fn*unit_fric;
		cir.moment+=cross(-vector_n*(cir.r-dep),friction_factor(cir.material,plain.material)*fn*unit_fric);
	}

	//第二类，地形点嵌入圆内
	for(i=0;i<2;i++)
	{
		if(cir.is_inside(pt[i]))
		{
			dep=cir.r-normal(cir.xy-pt[i]);
			vector_n=unit_vector(cir.xy-pt[i]);//单位化的法向量，方向向圆心
			fn1=vector_n*((cir.youngs_modulus/cir.r)*dep*(4*dep*dep*PI));
			cir.f=cir.f+fn1;
			v=cir.v+cir.omega*rotate_anticlockwise(-vector_n*(cir.r-dep));
			cir.f=cir.f-cir.m*PD_FACTOR* dot(v,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
			fn=normal(fn1-cir.m*PD_FACTOR* dot(v,vector_n)*vector_n);//弹力大小
			unit_fric=-unit_vector(v-dot(v,vector_n)*vector_n);//cir受到的摩擦力的方向向量
			cir.f=cir.f+friction_factor(cir.material,plain.material)*fn*unit_fric;
			cir.moment+=cross(-vector_n*(cir.r-dep),friction_factor(cir.material,plain.material)*fn*unit_fric);
		}
	}



}

template <typename T> void plane_poly_detect_collision(class Plane &plain,T &poly)
{
	XY ver_pt,ver_pt_temp;
	vector<XY> pt;
	pt.push_back(plain.points[0]);
	pt.push_back(plain.points[1]);
	ver_pt=vertical_point(pt[0],pt[1], poly.xy);//多边形至平面垂足
	if(normal(poly.xy-ver_pt)>poly.max_r)
		return;


	int i,j;
	vector<XY> line;
	int flag_cross[4]={0};
	int sum=0;
	for(i=0;i<poly.points.size();i++)
	{
		line.clear();
		line.push_back(poly.points[i]);
		line.push_back(poly.points[(i+1)%poly.points.size()]);
		if(is_cross(plain.points,line))
		{
			flag_cross[i]=1;
			sum++;
		}
		else
			flag_cross[i]=0;
		
	}

	

	XY vector_n;//法向量
	
	double dep,dep_temp;//深度	
	XY v;//两物体速度
	XY fn1;//物体杨氏模量弹力部分
	double fn;//弹力大小
	XY unit_fric;//摩擦力的方向向量
	XY pt_in;
	XY poly_pt1,poly_pt2;
	XY pt1_temp,pt2_temp;
	//第一类，地形边（线段，不是直线！）与多边形有两个交点
	if(sum==2)
	{
		for(i=0;i<poly.points.size();i++)
		{
			if(flag_cross[i]==1 && flag_cross[(i+1)%poly.points.size()]==1)
			{
				if(poly.points.size()==3 &&is_cross(poly.xy,poly.points[(i+1)%poly.points.size()],pt[0],pt[1])==0)
				{
					pt_in=poly.points[(i+2)%poly.points.size()];//直接复制过来的
						
					ver_pt=vertical_point(pt[0],pt[1], pt_in);
					vector_n=unit_vector(ver_pt-pt_in);//单位化的法向量，方向向反弹方向
					dep=normal(ver_pt-pt_in);
					fn1=vector_n*((poly.youngs_modulus/poly.r)*dep*(4*dep*dep*PI));
					poly.f=poly.f+fn1;
					v=poly.v+poly.omega*rotate_anticlockwise(pt_in-poly.xy);
					poly.f=poly.f-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
					fn=normal(fn1-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n);//弹力大小
					poly.moment+=cross(pt_in-poly.xy,fn*vector_n);
					unit_fric=-unit_vector(v-dot(v,vector_n)*vector_n);//poly受到的摩擦力的方向向量
					poly.f=poly.f+friction_factor(poly.material,plain.material)*fn*unit_fric;
					poly.moment+=cross(pt_in-poly.xy,friction_factor(poly.material,plain.material)*fn*unit_fric);

					pt_in=poly.points[i];//直接复制过来的
						
					ver_pt=vertical_point(pt[0],pt[1], pt_in);
					vector_n=unit_vector(ver_pt-pt_in);//单位化的法向量，方向向反弹方向
					dep=normal(ver_pt-pt_in);
					fn1=vector_n*((poly.youngs_modulus/poly.r)*dep*(4*dep*dep*PI));
					poly.f=poly.f+fn1;
					v=poly.v+poly.omega*rotate_anticlockwise(pt_in-poly.xy);
					poly.f=poly.f-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
					fn=normal(fn1-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n);//弹力大小
					poly.moment+=cross(pt_in-poly.xy,fn*vector_n);
					unit_fric=-unit_vector(v-dot(v,vector_n)*vector_n);//poly受到的摩擦力的方向向量
					poly.f=poly.f+friction_factor(poly.material,plain.material)*fn*unit_fric;
					poly.moment+=cross(pt_in-poly.xy,friction_factor(poly.material,plain.material)*fn*unit_fric);
				}
				else
				{
					pt_in=poly.points[(i+1)%poly.points.size()];
						
					ver_pt=vertical_point(pt[0],pt[1], pt_in);
					vector_n=unit_vector(ver_pt-pt_in);//单位化的法向量，方向向反弹方向
					dep=normal(ver_pt-pt_in);
					fn1=vector_n*((poly.youngs_modulus/poly.r)*dep*(4*dep*dep*PI));
					poly.f=poly.f+fn1;
					v=poly.v+poly.omega*rotate_anticlockwise(pt_in-poly.xy);
					poly.f=poly.f-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
					fn=normal(fn1-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n);//弹力大小
					poly.moment+=cross(pt_in-poly.xy,fn*vector_n);
					unit_fric=-unit_vector(v-dot(v,vector_n)*vector_n);//poly受到的摩擦力的方向向量
					poly.f=poly.f+friction_factor(poly.material,plain.material)*fn*unit_fric;
					poly.moment+=cross(pt_in-poly.xy,friction_factor(poly.material,plain.material)*fn*unit_fric);
				}
			}
		}
		vector <XY> center_to_pts;
		center_to_pts.push_back(poly.xy);
		center_to_pts.push_back(poly.points[0]);
		XY _pt_in[2];
		for(i=0;i<2;i++)
		{
			center_to_pts[1]=poly.points[i];
			if(poly.points.size()==4 && flag_cross[i]==1 && flag_cross[2+i]==1)
			{
				if(is_cross(center_to_pts,pt))
				{
					_pt_in[0]=poly.points[i];
					_pt_in[1]=poly.points[(i+3)%poly.points.size()];
				}
				else
				{
					_pt_in[0]=poly.points[i+1];
					_pt_in[1]=poly.points[i+2];
				}
				for(j=0;j<2;j++)
				{
					ver_pt=vertical_point(pt[0],pt[1], _pt_in[j]);
					vector_n=unit_vector(ver_pt-_pt_in[j]);//单位化的法向量，方向向反弹方向
					dep=normal(ver_pt-_pt_in[j]);
					fn1=vector_n*((poly.youngs_modulus/poly.r)*dep*(4*dep*dep*PI));
					poly.f=poly.f+fn1;
					v=poly.v+poly.omega*rotate_anticlockwise(_pt_in[j]-poly.xy);
					poly.f=poly.f-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
					fn=normal(fn1-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n);//弹力大小
					poly.moment+=cross(_pt_in[j]-poly.xy,fn*vector_n);
					unit_fric=-unit_vector(v-dot(v,vector_n)*vector_n);//poly受到的摩擦力的方向向量
					poly.f=poly.f+friction_factor(poly.material,plain.material)*fn*unit_fric;
					poly.moment+=cross(_pt_in[j]-poly.xy,friction_factor(poly.material,plain.material)*fn*unit_fric);
				}
			}

		}
	}
	
	//第二类，地形边（线段，不是直线！）与多边形只有一个交点，地形点嵌入多边形内
	
	if(sum==1)
	{
		for(i=0;i<2;i++)
		{
			if(poly.is_inside(pt[i]))
			{

				for(j=0;j<poly.points.size();j++)//找离点的距离最小的边
				{
					pt1_temp=poly.points[j];
					pt2_temp=poly.points[(j+1)%poly.points.size()];
					ver_pt_temp=vertical_point(pt1_temp,pt2_temp,pt[i]);//作垂直点
					dep_temp=normal(pt[i]-ver_pt_temp);
					if(j==0)
						dep=dep_temp;
					
					if(dep_temp<=dep)
					{
						dep=dep_temp;
						ver_pt=ver_pt_temp;
						poly_pt1=pt1_temp;
						poly_pt2=pt2_temp;
					}
				}	
				
				ver_pt= vertical_point(poly_pt1,poly_pt2,pt[i]);
				vector_n=unit_vector(pt[i]-ver_pt);//单位化的法向量，方向为弹力方向
				dep=normal(pt[i]-ver_pt);
				fn1=vector_n*((poly.youngs_modulus/poly.r)*dep*(4*dep*dep*PI));
				poly.f=poly.f+fn1;
				v=poly.v+poly.omega*rotate_anticlockwise(ver_pt-poly.xy);
				poly.f=poly.f-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n;//类似于Schmidt正交化的公式，加个速度阻尼
				fn=normal(fn1-poly.m*PD_FACTOR* dot(v,vector_n)*vector_n);//弹力大小
				poly.moment+=cross(ver_pt-poly.xy,fn*vector_n);

				unit_fric=-unit_vector(v-dot(v,vector_n)*vector_n);//poly受到的摩擦力的方向向量
				poly.f=poly.f+friction_factor(poly.material,plain.material)*fn*unit_fric;
				poly.moment+=cross(ver_pt-poly.xy,friction_factor(poly.material,plain.material)*fn*unit_fric);

				//if()//为了对付平面边物体翻落问题
			}
		}
	}
}








void clear(char *buffer, char *cmd, int lenth)
{
    int i;
    for (i = 0; i < lenth; i++)
    {
        buffer[i] = '\0';
        cmd[i] = '\0';
    }
}


