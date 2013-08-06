 //# a

#include <stdio.h>
#include "Point2D.h"
#include <vector>
using namespace std;

//* 线段交叉检测
bool CheckDiffSide(const Point2D& p1,const Point2D& p2,const Point2D& q1,const Point2D& q2)
{
	Point2D u1=p1-q1,v1=p2-q1,u2=p1-q2,v2=p2-q2;

	double t1=u1%v1,t2=u2%v2;

	return t1>0&&t2<0||t1<0&&t2>0;
}

const double M_INFINITED=1e20;

class Edge2D
{
public:
	int _u;
	int _v;
	double _w;
	Edge2D(int u=0,int v=0):_u(u),_v(v){}

};

char g_start1_str[1024],g_start2_str[1024];
vector<vector<Point2D>> g_map;
vector<Point2D> g_mapV;

vector<Edge2D> g_mapE;
vector<Edge2D> g_mapEx;

vector<double> g_mapD;
vector<int> g_mapPre;

Point2D g_dest,g_start1,g_start2;// 0,1,2

//* 读取输入
void ReadMap(const char* file)
{
	char buffer[1024];
	char str[512];
    FILE* fp;


	fp=fopen(file,"rt");
	fgets(buffer,1024,fp);
	sscanf(buffer,"%*[^,]%*[, ](%lf, %lf)",&g_dest._x,&g_dest._y);
	fgets(buffer,1024,fp);
	sscanf(buffer,"%*[^,]%*[, ](%lf, %lf)%*[^,]%*[, ](%lf, %lf)",&g_start1._x,&g_start1._y,&g_start2._x,&g_start2._y);
	sscanf(buffer,"%[^/]/ %[^\n]",g_start1_str,g_start2_str);
	g_start2_str[strlen(g_start2_str)]='\0';
	g_mapV.push_back(g_dest);
	g_mapV.push_back(g_start1);
	g_mapV.push_back(g_start2);

	while(!feof(fp)){
		Point2D p;
		const char* t;
		int r;
		fgets(buffer,1024,fp);
		t=buffer;
		g_map.push_back(vector<Point2D>());

		vector<Point2D>& obstacle=g_map.back();
		r=g_mapV.size();
		while(1){
			if(sscanf(t,"%[^)]",str)==1){
				sscanf(str,"(%lf, %lf",&p._x,&p._y);
				obstacle.push_back(p);
				g_mapV.push_back(p);
			}
			t+=strlen(str);
			if(t[1]!=','){
				g_mapE.push_back(Edge2D(g_mapV.size()-1,r));
				break;
			}
			else{
				g_mapE.push_back(Edge2D(g_mapV.size()-1,g_mapV.size()));
				t+=3;
			}
		}
		if(g_mapV.size()-r>3){
			for(int i=r+1;i<g_mapV.size()-1;i++){
				g_mapE.push_back(Edge2D(i-1,i+1));
			}
			g_mapE.push_back(Edge2D(g_mapV.size()-1-1,r));
			g_mapE.push_back(Edge2D(g_mapV.size()-1,r+1));
		}


		// edge;
	}
}

//* 生成图权重
void ComputeWeight(void)
{
	for(int u=0;u<g_mapV.size();u++){
		const Point2D& up=g_mapV[u];
		for(int v=u+1;v<g_mapV.size();v++){
			Edge2D edge(u,v);
			const Point2D& vp=g_mapV[v];
			Point2D uv=vp-up;

			edge._w=uv.length();
			g_mapEx.push_back(edge);
			for(int e=0;e<g_mapE.size();e++){
				const Edge2D& obs=g_mapE[e];
				if(CheckDiffSide(g_mapV[obs._u],g_mapV[obs._v],g_mapV[u],g_mapV[v])
					&&CheckDiffSide(g_mapV[u],g_mapV[v],g_mapV[obs._u],g_mapV[obs._v])){
						g_mapEx.pop_back();
						break;
				}
			}
		}
	}
}

//* 最短路径
void findShortestWay(int s)
{
	g_mapD.resize(g_mapV.size());
	g_mapPre.resize(g_mapV.size());
	for(int i=0;i<g_mapD.size();i++){
		g_mapD[i]=M_INFINITED;
		g_mapPre[i]=-1;
	}
	g_mapD[s]=0;

	for(int i=1;i<g_mapV.size()-1;i++){
		for(int e=0;e<g_mapEx.size();e++){
		    const Edge2D& edge=g_mapEx[e];
			double w;
			
			w=g_mapD[edge._u]+edge._w;
			if(w<g_mapD[edge._v]){
				g_mapD[edge._v]=w;
				g_mapPre[edge._v]=edge._u;
			}
			w=g_mapD[edge._v]+edge._w;
			if(w<g_mapD[edge._u]){
				g_mapD[edge._u]=w;
				g_mapPre[edge._u]=edge._v;
			}
		}
	}   
}


//* 输出
const char* g_out="F:\\data\\map.out";
//* 输入
const char* g_in="F:\\data\\map.in";


int main(int argc, char* argv[])
{
	FILE* fp;
    
	ReadMap(g_in);
	ComputeWeight();
	findShortestWay(0);
	
	// 输出
	fp=fopen(g_out,"rt");

	if(g_mapD[1]<g_mapD[2]){
		int d=(int) g_mapD[1];
		fwrite(g_start1_str,sizeof(char),strlen(g_start1_str),fp);
		fwrite(&d,sizeof(int),1,fp);
		fprintf(fp,"%d",d);
	}
	else{
		int d=(int) g_mapD[2];
		fprintf(fp,"%s\n",g_start2_str);
		fprintf(fp,"%d",d);
	}
	fclose(fp);

	return 0;
}
