#include <queue>
#include <iostream>
#include <cstdio>
#include <cmath>

using namespace std;

const static unsigned char dif = 30;
const static unsigned level_1 = 127;
const static unsigned level_2 = 255;

class point{
  short x;
  short y;
public:
  point():x(0),y(0){}
  point(short _x,short _y):x(_x),y(_y){}
  void  show()
  {
    printf("%d %d\n",x,y);
  }
  short get_x()
  {
    return x;
  }
  short get_y()
  {
    return y;
  }
};

bool isPointLegal(int x, int y,int m, int n)
{
  if(x>=0 && x<m && y>=0 && y<n){
    return true;
  }
  else{
    return false;
  }
}

bool isEdge(unsigned char x, unsigned char y)
{
  if(x>=y && (x-y)>=dif || x<y &&(y-x)>=dif){
    return true;
  }
  else{
    return false;
  }
}

void sobel(unsigned char** mat, unsigned char** markMat, int m, int n)
{
  unsigned char a00, a01, a02;
  unsigned char a10, a11, a12;
  unsigned char a20, a21, a22;
  for(int i=1;i<m-1;i++){
    for(int j=1;j<n-1;j++){
      a00 = mat[i-1][j-1];
      a01 = mat[i-1][j];
      a02 = mat[i-1][j+1];
      a10 = mat[i][j-1];
      a11 = mat[i][j];
      a12 = mat[i][j+1];
      a20 = mat[i+1][j-1];
      a21 = mat[i+1][j];
      a22 = mat[i+1][j+1];
 
      //x方向上的近似导数
      double ux = a20 * (1) + a21 * (2) + a22 * (1)
	+ (a00 * (-1) + a01 * (-2) + a02 * (-1));
      // y方向上的近似导数
      double uy = a02 * (1) + a12 * (2) + a22 * (1)
	+ a00 * (-1) + a10 * (-2) + a20 * (-1);

      float tmp_f = sqrt(ux*ux + uy*uy);
      markMat[i][j] = tmp_f>255?255:(unsigned char)tmp_f;
    }
  }
  return;
}

void canny(unsigned char** mat, int m, int n,int thresh)
{
  return;
}

void solution(unsigned char** mat, int m, int n, int iter_times, point seed)
{
  //queue
  queue<point> q_point;
  //需要的矩阵
  //输出矩阵与标记矩阵
  unsigned char** outMat = new unsigned char*[m];
  unsigned char** markMat = new unsigned char*[m];
  for(int i=0;i<m;i++){
    outMat[i] = new unsigned char[n];
    markMat[i] = new unsigned char[n];
    for(int j=0;j<n;j++){
      outMat[i][j]=0;
      markMat[i][j]=0;
    }
  }
  //用于终止迭代过程
  bool signal_end = false;
  int iter = 0;
  //逻辑段
  q_point.push(seed);
  markMat[seed.get_x()][seed.get_y()] = level_1;
  while(!q_point.empty()&& iter<=iter_times && !signal_end){
    point tmp = q_point.front();
    q_point.pop();
    int x = tmp.get_x();
    int y = tmp.get_y();
    //计算八个方向上的梯度，并发展新的种子点
    //(-1,-1)direction
    if(isPointLegal(x-1, y-1,m ,n) && markMat[x-1][y-1]==0){
      if(isEdge(mat[x-1][y-1],mat[x][y])){
	//mark该点不可用
	markMat[x-1][y-1] = level_2;
      }
      else{
	//标出层级
	q_point.push(point(x-1,y-1));
	markMat[x-1][y-1] = level_1;
      }
    }
    //(-1,0)direction
    if(isPointLegal(x-1, y,m ,n) && markMat[x-1][y]==0){
      if(isEdge(mat[x-1][y],mat[x][y])){
	//mark该点不可用
	markMat[x-1][y] = level_2;
      }
      else{
	//标出层级
	q_point.push(point(x-1,y));
	markMat[x-1][y] = level_1;
      }
    }
    //(-1,1)direction
    if(isPointLegal(x-1, y+1,m ,n) && markMat[x-1][y+1]==0){
      if(isEdge(mat[x-1][y+1],mat[x][y])){
	//mark该点不可用
	markMat[x-1][y+1] = level_2;
      }
      else{
	//标出层级
	q_point.push(point(x-1,y+1));
	markMat[x-1][y+1] = level_1;
      }
    }
    //(0,-1)direction
    if(isPointLegal(x, y-1,m ,n) && markMat[x][y-1]==0){
      if(isEdge(mat[x][y-1],mat[x][y])){
	//mark该点不可用
	markMat[x][y-1] = level_2;
      }
      else{
	//标出层级
	q_point.push(point(x,y-1));
	markMat[x][y-1] = level_1;
      }
    }
    //(0,1)direction
    if(isPointLegal(x, y+1,m ,n) && markMat[x][y+1]==0){
      if(isEdge(mat[x][y+1],mat[x][y])){
	//mark该点不可用
	markMat[x][y+1] = level_2;
      }
      else{
	//标出层级
	q_point.push(point(x,y+1));
	markMat[x][y+1] = level_1;
      }
    }
    //(1,-1)direction
    if(isPointLegal(x+1, y-1,m ,n) && markMat[x+1][y-1]==0){
      if(isEdge(mat[x+1][y-1],mat[x][y])){
	//mark该点不可用
	markMat[x+1][y-1] = level_2;
      }
      else{
	//标出层级
	q_point.push(point(x+1,y-1));
	markMat[x+1][y-1] = level_1;
      }
    }
    //(1,0)direction
    if(isPointLegal(x+1, y,m ,n) && markMat[x+1][y]==0){
      if(isEdge(mat[x+1][y],mat[x][y])){
	//mark该点不可用
	markMat[x+1][y] = level_2;
      }
      else{
	//标出层级
	q_point.push(point(x+1,y));
	markMat[x+1][y] = level_1;
      }
    }
    //(1,1)direction
    if(isPointLegal(x+1, y+1,m ,n) && markMat[x+1][y+1]==0){
      if(isEdge(mat[x+1][y+1],mat[x][y])){
	//mark该点不可用
	markMat[x+1][y+1] = level_2;
      }
      else{
	//标出层级
	q_point.push(point(x+1,y+1));
	markMat[x+1][y+1] = level_1;
      }
    }

    iter++;
  }


  for(int i=0;i<m;i++){
    delete[] outMat[i];
    delete[] markMat[i];
  }
  delete[] outMat;
  delete[] markMat;
}

int main(int argc,char* argv[])
{
  point A(2,3);
  A.show();
  FILE *fp_input = fopen("./test.txt","r");
  FILE *fp_output = fopen("./result.txt","w+");
  
  int height =256,width =256;

  unsigned char** mat = new unsigned char*[height];
  for(int i=0;i<height;i++){
    mat[i] = new unsigned char[width];
    for(int j=0;j<width;j++)
      mat[i][j]=0;
  }
  //for test sobel
  unsigned char** out = new unsigned char*[height];
  for(int i=0;i<height;i++){
    out[i] = new unsigned char[width];
    for(int j=0;j<width;j++)
      mat[i][j]=0;
  }


  unsigned int tmp_i;
  for(int i=0;i<height;i++){
    for(int j=0;j<width;j++){
      fscanf(fp_input,"%u",&tmp_i);
      mat[i][j] = (unsigned char)(tmp_i);
    }
  }

  sobel(mat,out,height,width);

  for(int i=0;i<height;i++){
    for(int j=0;j<width;j++){
      fprintf(fp_output, "%d ", out[i][j]);
    }
    fprintf(fp_output, "\n");
  }

  fclose(fp_input);
  fclose(fp_output);
  for(int i=0;i<height;i++){
    delete[] mat[i];
    delete[] out[i];
  }
  delete[] mat;
  delete[] out;

}
