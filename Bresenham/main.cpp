/*
    Author:Tomanlon

    Copyright:Tomanlon

    Date:2019-3-12

    Description: Simple demo for show bresenham draw line and obtain points set

    MailBox:taomanl@163.com
*/

#include <iostream>
#include <GL/glut.h>
#include <vector>

using namespace std;

int x0 = 500,y0 = 0;
int x1 = -500,y1 =500;

//define points
struct MapPoint
{
    int x;
    int y;
    MapPoint()
    {
        x=0;
        y=0;
    }
};

//draw points
void putpixel(int x,int y)
{
    glColor3f(0.0, 1.0, 1.0);
    glPointSize(3.0);
    glBegin(GL_POINTS);
    glVertex2f(x, y);
    glEnd();
}

//used for draw line
void bresenham_line(int x0, int y0, int x1, int y1) {
    int dx, dy, h, a, b, x, y, flag, t;
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    if (x1 > x0) a = 1; else a = -1;
    if (y1 > y0) b = 1; else b = -1;
    x = x0;
    y = y0;
    if (dx >= dy) {
        //0< |k| <=1
        flag = 0;
    } else {
        //|k|>1,exchange dx，dy
        t = dx;
        dx = dy;
        dy = t;
        flag = 1;
    }
    h = 2 * dy - dx;
    for (int i = 1; i <= dx; ++i) {
        putpixel(x, y);
        if (h >= 0) {
            if (flag == 0) y = y + b;
            else x = x + a;
            h = h - 2 * dx;
        }
        if (flag == 0) x = x + a;
        else y = y + b;
        h = h + 2 * dy;

    }

}

//used for obtain points set
vector<MapPoint> bresenham(int x0,int y0,int x1,int y1)
{
    vector<MapPoint> pp;
    MapPoint p;
    cout<<"init p is: "<<p.x<<" "<<p.y<<endl;
    int dx, dy, h, a, b, x, y, flag, t;
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    if (x1 > x0) a = 1; else a = -1;
    if (y1 > y0) b = 1; else b = -1;
    x = x0;
    y = y0;
    if (dx >= dy) {
        //0< |k| <=1
        flag = 0;
    } else {
        //|k|>1,exchange dx，dy
        t = dx;
        dx = dy;
        dy = t;
        flag = 1;
    }
    h = 2 * dy - dx;
    for (int i = 1; i <= dx; ++i) {
        p.x = x,p.y=y;
        pp.push_back(p);
        if (h >= 0) {
            if (flag == 0) y = y + b;
            else x = x + a;
            h = h - 2 * dx;
        }
        if (flag == 0) x = x + a;
        else y = y + b;
        h = h + 2 * dy;

    }
    return pp;
}

//gult init
void init()
{
	glutInitDisplayMode(GLUT_SINGLE|GLUT_RGB);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(600,600);
	glutCreateWindow("Bresenham");
    glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	gluOrtho2D(-500, 500, -500, 500);
}

//show result
void display()
{
    bresenham_line(x0,y0,x1,y1);
    glFlush();
}

int main(int argc,char** argv)
{
	glutInit(&argc, argv);
	init();
	vector<MapPoint> pset;
	pset = bresenham(x0,y0,x1,y1);
	// print points in pset
	for(vector<MapPoint>::iterator iter=pset.begin();
                             iter!=pset.end();iter++)
	{
        cout<<(*iter).x<<" "<<(*iter).y<<endl;
	}
	//display draw points
    glutDisplayFunc(display);
	glutMainLoop();
    return 0;
}
