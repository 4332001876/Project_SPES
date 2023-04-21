
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

	double eq_data_sec;
	FILE *eq_fp;
	printf("CMD:kobe,tohoku,wenchuan,chichi,kocaeli  \n");
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
	
		if(strcmp(cmd,"kobe")==0)
		{
			eq_fp=fopen("EQKobe.rec","r");
			eq_data_sec=0.01;
			fseek(eq_fp,0,SEEK_SET);
			fgets(buffer, 500, eq_fp);//����ǰ����
			fgets(buffer, 500, eq_fp);
			_stprintf(out, _T("��������:M7.2    1995-01-17    Kobe,Japan    Nishi-Akashi    KOBE/NIS090\0"));
			//strcpy(out,"7.2    1995    Kobe,Japan    Nishi-Akashi    KOBE/NIS090");
			break;
		}
		if(strcmp(cmd,"tohoku")==0)
		{
			eq_fp=fopen("EQTohoku.rec","r");
			eq_data_sec=0.05;
			fseek(eq_fp,0,SEEK_SET);
			fgets(buffer, 500, eq_fp);//����ǰ����
			fgets(buffer, 500, eq_fp);
			_stprintf(out, _T("��������:M9.0    2011-03-11    Tohoku,Japan"));
			
			break;
		}
		if(strcmp(cmd,"wenchuan")==0)
		{
			eq_fp=fopen("EQWenchuan.rec","r");
			eq_data_sec=0.01;
			fseek(eq_fp,0,SEEK_SET);
			fgets(buffer, 500, eq_fp);//����ǰ����
			fgets(buffer, 500, eq_fp);
			_stprintf(out, _T("��������:M8.0    2008-05-12    Wenchuan,China    051SFB ua0283"));
			
			break;
		}
		if(strcmp(cmd,"chichi")==0)
		{
			eq_fp=fopen("EQChichi.rec","r");
			eq_data_sec=0.01;
			fseek(eq_fp,0,SEEK_SET);
			fgets(buffer, 500, eq_fp);//����ǰ����
			fgets(buffer, 500, eq_fp);
			_stprintf(out, _T("��������:M7.6    1999-09-21    Chi-Chi,Taiwan Province    CHY101    CHICHI/CHY101-N"));
			
			break;
		}
		if(strcmp(cmd,"kocaeli")==0)
		{
			
			eq_fp=fopen("EQKocaeli.rec","r");
			eq_data_sec=0.01;
			fseek(eq_fp,0,SEEK_SET);
			fgets(buffer, 500, eq_fp);//����ǰ����
			fgets(buffer, 500, eq_fp);
			_stprintf(out, _T("��������:M7.5    1999-08-17    Kocaeli,Turkey    Duzce    KOCAELI/DZC270"));
			
			break;
			
		}
		if(strcmp(cmd,"tertri")==0)
		{
			
		}
		if(strcmp(cmd,"terrect")==0)
		{
			
		}
		if(strcmp(cmd,"finish")==0)
		{
			break;
		}
	}

	initgraph(WIDTH,HEIGHT,EW_SHOWCONSOLE);
	setorigin(WIDTH/2,HEIGHT/2);//������������
	
	// ���ñ���ɫ
	setbkcolor(BLACK);
	
	// �ñ���ɫ�����Ļ
	cleardevice();
	
	// ���û�ͼɫ
	setcolor(GREEN);
	setfillcolor(0x003800);
	
	setlinestyle(PS_SOLID,2);
	setaspectratio(1, -1);//���õ�ǰ��������(�����Ŵ���С��Բ�ǵ�),�����������Ϊ��������ʵ��������ķ�ת����ִ�� setaspectratio(1, -1) �󣬿�ʹ y ������Ϊ����
    //magnify_ratio=10; //�Ŵ���,��ʾʱ����Ϊԭ������������

	
	settextstyle(30,0,_T("Segoe UI"));
	LOGFONT f;
	gettextstyle(&f);						// ��ȡ��ǰ��������
	//f.lfHeight = 48;						// ��������߶�Ϊ 48
	//_tcscpy(f.lfFaceName, _T("����"));		// ��������Ϊ�����塱(�߰汾 VC �Ƽ�ʹ�� _tcscpy_s ����)
	f.lfQuality = ANTIALIASED_QUALITY;		// �������Ч��Ϊ�����  
	settextstyle(&f);

	int mouseX, mouseY;         // ��ǰ�������
	int mouseVX, mouseVY;       // ����ٶ�
	int prevMouseX, prevMouseY; // �ϴ��������
	bool isMouseDown;           // �������Ƿ���
	int originX=WIDTH/2,originY=HEIGHT/2;//ԭ������

	double move_speed=0;

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
	
	fscanf(eq_fp,"%e",&eq_a);
	fscanf(eq_fp,"%e",&eq_a);
	
    for (i = 0;; i++)
    {
		
		if(i*time_step>eq_count*eq_data_sec)
		{
			eq_a_old=eq_a;
			fscanf(eq_fp,"%lf",&eq_a);
			fscanf(eq_fp,"%lf",&eq_a);
			eq_a=eq_a*9.8;
			eq_count++;
			line(-WIDTH/2+30+(eq_count%3000)/12.5,-HEIGHT/2+120+eq_a*5,-WIDTH/2+30+(eq_count%3000)/12.5,-HEIGHT/2+120+eq_a_old*5);
			getimage(pDstImg,-WIDTH/2+30,-HEIGHT/2+70,240,100);
			if(eq_count%3000==0)
			{
				clearrectangle(-WIDTH/2+30,-HEIGHT/2+70,-WIDTH/2+270,-HEIGHT/2+170);
				rectangle(-WIDTH/2+30,-HEIGHT/2+70,-WIDTH/2+270,-HEIGHT/2+170);
				line(-WIDTH/2+30,-HEIGHT/2+120,-WIDTH/2+270,-HEIGHT/2+120);
				getimage(pDstImg,-WIDTH/2+30,-HEIGHT/2+70,240,100);
			}
		}
		_stprintf(count_sec_text, _T("%.2f\0"),eq_count*eq_data_sec);


		move_speed-=time_step*eq_a;
		moveX+=time_step*move_speed;
		smooth_moveX+=time_step*move_speed;
		
		space.RK4(time_step,space_temp);//RK4����
		
		
		flag_color_clear=0;
        if (i % batch == 0)
        {
			flag_color_clear=1;
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