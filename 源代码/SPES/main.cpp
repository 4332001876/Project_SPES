
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
	setorigin(WIDTH/2,HEIGHT/2);//������������
	
	// ���ñ���ɫ
	setbkcolor(BLACK);
	
	// �ñ���ɫ�����Ļ
	cleardevice();
	
	// ���û�ͼɫ
	setcolor(GREEN);
	setfillcolor(0x003800);
	// ������
	
	setlinestyle(PS_SOLID,2);
	setaspectratio(1, -1);//���õ�ǰ��������(�����Ŵ���С��Բ�ǵ�),�����������Ϊ��������ʵ��������ķ�ת����ִ�� setaspectratio(1, -1) �󣬿�ʹ y ������Ϊ����
    //magnify_ratio=10; //�Ŵ���,��ʾʱ����Ϊԭ������������



	int mouseX, mouseY;         // ��ǰ�������
	int mouseVX, mouseVY;       // ����ٶ�
	int prevMouseX, prevMouseY; // �ϴ��������
	bool isMouseDown;           // �������Ƿ���
	int originX=WIDTH/2,originY=HEIGHT/2;//ԭ������

	// �����Ϣ����
    ExMessage msg;
	getmessage(&msg);
	prevMouseX = msg.x;
	prevMouseY = msg.y;
	int flag_first_left_button=0;

	double smooth_moveX=moveX,smooth_moveY=moveY;//������϶������⻬��
	double smooth_magnify_ratio=magnify_ratio;//�������ֲ����⻬��
	int batch=(int)(2*magnify_ratio);
	time_step*=20.0/batch;
	int i;
    for (i = 0;; i++)
    {
		
		space.RK4(time_step,space_temp);//RK4����
		
		
        if (i % batch == 0)
        {
			
			while(peekmessage(&msg,EM_MOUSE))//ʵ������϶��͹���
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
	
	// ��������˳�
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