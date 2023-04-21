
using namespace std;
#include "calculation.h"
void delay(DWORD ms);

int main(void)
{
    double time_step = 0.001;

	class SPACE space,space_temp;
	space.init_cmd();
	space_temp=space;
	printf("Magnify_ratio?");
	scanf("%lf",&magnify_ratio);

	initgraph(WIDTH,HEIGHT,EW_SHOWCONSOLE);
	setorigin(WIDTH/2,HEIGHT/2);//重设坐标中心
	
	// 设置背景色
	setbkcolor(BLACK);
	
	// 用背景色清空屏幕
	cleardevice();
	
	// 设置绘图色
	setcolor(GREEN);
	setfillcolor(0x003800);
	// 画矩形
	
	setlinestyle(PS_SOLID,2);
	setaspectratio(1, -1);//设置当前缩放因子(拿它放大缩小是圆角的),如果缩放因子为负，可以实现坐标轴的翻转。如执行 setaspectratio(1, -1) 后，可使 y 轴向上为正。
    //magnify_ratio=10; //放大倍数,显示时坐标为原坐标乘上这个数



	int mouseX, mouseY;         // 当前鼠标坐标
	int mouseVX, mouseVY;       // 鼠标速度
	int prevMouseX, prevMouseY; // 上次鼠标坐标
	bool isMouseDown;           // 鼠标左键是否按下
	int originX=WIDTH/2,originY=HEIGHT/2;//原点坐标

	// 鼠标消息变量
    ExMessage msg;
	getmessage(&msg);
	prevMouseX = msg.x;
	prevMouseY = msg.y;
	int flag_first_left_button=0;

	double smooth_moveX=moveX,smooth_moveY=moveY;//让鼠标拖动操作光滑化
	double smooth_magnify_ratio=magnify_ratio;//让鼠标滚轮操作光滑化
	int batch=(int)(2*magnify_ratio);
	time_step*=20.0/batch;
	int i;
    for (i = 0;; i++)
    {
		
		//space.RK4(time_step,space_temp);//RK4积分
		space.cal_main(time_step);
		space.EulerCalculus(time_step);
		
		
        if (i % batch == 0)
        {
			
			while(peekmessage(&msg,EM_MOUSE))//实现鼠标拖动和滚轮
			{
				switch(msg.message)
				{
				case WM_LBUTTONDOWN:
				{
					isMouseDown = true;
					prevMouseX = msg.x;
					prevMouseY = msg.y;
					
					flag_first_left_button=1;
					break;
				}
				case WM_LBUTTONUP:
				{
					isMouseDown = false;

					if(flag_first_left_button)
					{
						smooth_moveX+=(msg.x-prevMouseX)/magnify_ratio;
						smooth_moveY-=(msg.y-prevMouseY)/magnify_ratio;
					
					}
					break;
				}
				case WM_MOUSEWHEEL:
				{
					if(msg.wheel==120)
						smooth_magnify_ratio*=2;
					if(msg.wheel==-120)
						smooth_magnify_ratio*=0.5;
				}
				}
			}
			if(abs(smooth_moveX-moveX)>=1)
				moveX=0.3*smooth_moveX+0.7*moveX;
			else
				moveX=smooth_moveX;
			if(abs(smooth_moveY-moveY)>=1)
				moveY=0.3*smooth_moveY+0.7*moveY;
			else
				moveY=smooth_moveY;
			if(abs(smooth_magnify_ratio/magnify_ratio)>0.95&&abs(smooth_magnify_ratio/magnify_ratio)<1.05)
				magnify_ratio=smooth_magnify_ratio;
			else
			{
				magnify_ratio=sqrt(smooth_magnify_ratio*magnify_ratio);
			}
			space.generate_graph();
			delay(20);
        }
    }
	
	// 按任意键退出
	getchar();
	
    return 0;
}
void delay(DWORD ms)
{
    static DWORD oldtime = GetTickCount();

    while (GetTickCount() - oldtime < ms)
        Sleep(1);

    oldtime = GetTickCount();
}