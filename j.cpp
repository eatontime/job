#include <queue>
#include <iostream>
#include <cstdio>
#include <cmath>

using namespace std;

const static unsigned char dif = 60;
const static unsigned char level_1 = 100;
const static unsigned char level_2 = 255;
const static int iter_times = 100000;

class point {
  short x;
  short y;
public:
  point() :x(0), y(0) {}
  point(short _x, short _y) :x(_x), y(_y) {}
  void  show()
  {
    printf("%d %d\n", x, y);
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

bool isInCircle(int x, int y, int radius);
bool isInImage(int x, int y, int m, int n, int radius);
bool isEdge(unsigned char p_src, unsigned char p_sobel);
void sobel(unsigned char** mat, unsigned char** markMat, int m, int n);
void saveToTxt(unsigned char** mat, const char* filename, int height, int width);
void solution(unsigned char** mat, int m, int n, point seed, int radius, const char* filename);

bool isInCircle(int x, int y, int radius)
{
  if ((x*x + y*y) > radius*radius)
    return false;
  else
    return true;
}

bool isInImage(int x, int y, int m, int n, int radius)
{
  if (x >= 0 && x<m && y >= 0 && y<n) {
    return true;
  }
  else {
    return false;
  }
}

bool isEdge(unsigned char p_src, unsigned char p_sobel)
{
  if (p_sobel>dif) {
    return true;
  }
  else {
    return false;
  }
}

void sobel(unsigned char** mat, unsigned char** markMat, int m, int n)
{
  unsigned char a00, a01, a02;
  unsigned char a10, a11, a12;
  unsigned char a20, a21, a22;
  for (int i = 1; i<m - 1; i++) {
    for (int j = 1; j<n - 1; j++) {
      a00 = mat[i - 1][j - 1];
      a01 = mat[i - 1][j];
      a02 = mat[i - 1][j + 1];
      a10 = mat[i][j - 1];
      a11 = mat[i][j];
      a12 = mat[i][j + 1];
      a20 = mat[i + 1][j - 1];
      a21 = mat[i + 1][j];
      a22 = mat[i + 1][j + 1];

      //x方向上的近似导数
      double ux = a20 * (1) + a21 * (2) + a22 * (1)
	+ (a00 * (-1) + a01 * (-2) + a02 * (-1));
      //y方向上的近似导数
      double uy = a02 * (1) + a12 * (2) + a22 * (1)
	+ a00 * (-1) + a10 * (-2) + a20 * (-1);

      float tmp_f = sqrt(ux*ux + uy*uy);
      markMat[i][j] = tmp_f>255 ? 255 : (unsigned char)tmp_f;
    }
  }
  return;
}

void saveToTxt(unsigned char** mat, const char* filename,int height, int width)
{
  FILE *fp;
  fp = fopen(filename, "w");
  for (int i = 0; i<height; i++) {
    for (int j = 0; j<width; j++) {
      fprintf(fp, "%d ", mat[i][j]);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return;
}

void solution(unsigned char** mat, int m, int n, point seed, int radius, const char* filename)
{
  //queue
  queue<point> q_point;

  //sobel矩阵sobelMat与输出矩阵markMat的创建与初始化
  unsigned char** sobelMat = new unsigned char*[m];
  unsigned char** markMat = new unsigned char*[m];
  for (int i = 0; i<m; i++) {
    sobelMat[i] = new unsigned char[n];
    markMat[i] = new unsigned char[n];
    for (int j = 0; j<n; j++) {
      sobelMat[i][j] = 0;
      markMat[i][j] = 0;
    }
  }

  sobel(mat, sobelMat, m, n);

  //用于终止迭代过程
  bool signal_end = false;
  int iter = 0;

  //逻辑段开始
  q_point.push(seed);
  point src_point = q_point.front();	//原始点
  int x_src = src_point.get_x();
  int y_src = src_point.get_y();
  markMat[seed.get_x()][seed.get_y()] = level_1;
  while (!q_point.empty() && iter <= iter_times && !signal_end) {
    point tmp = q_point.front();
    q_point.pop();
    int x = tmp.get_x();
    int y = tmp.get_y();

    //向八领域发展新的种子点
    for (int i = -1; i < 2; i++) {
      for (int j = -1; j < 2; j++) {
	if (i == 0 && j == 0)
	  continue;
	if (isInImage(x - i, y - j, m, n, radius) && isInCircle(x - x_src, y - y_src, radius) && markMat[x - i][y - j] == 0) {//点在图像内 && 点在划定半径内 && 点未被访问过
	  if (isEdge(mat[x][y], sobelMat[x - i][y - j])) {
	    //mark该点不可用
	    markMat[x - i][y - j] = level_2;
	  }
	  else {
	    //改点可用，放入queue的尾部；标记改点为种子点的相似点
	    q_point.push(point(x - i, y - j));
	    markMat[x - i][y - j] = level_1;
	  }
	}
      }
    }

    iter++; //迭代次数+1
  }

  saveToTxt(markMat, filename,m,n);//将结果输出到filename对应的txt文件中

  for (int i = 0; i<m; i++) {
    delete[] sobelMat[i];
    delete[] markMat[i];
  }
  delete[] sobelMat;
  delete[] markMat;
}

int main()
{
  FILE *fp_input;
  FILE *fp_output;
  const char* filename_in = "./test.txt";
  const char* filename_out = "./result.txt";
  fp_input=fopen(filename_in, "r");

  int height = 256, width = 256;//待矩阵的高度和宽度

  unsigned char** mat = new unsigned char*[height];//创建二维数组并初始化
  for (int i = 0; i<height; i++) {
    mat[i] = new unsigned char[width];
    for (int j = 0; j<width; j++)
      mat[i][j] = 0;
  }

  unsigned int tmp_i;//从txt文件中读取二维数组
  for (int i = 0; i<height; i++) {
    for (int j = 0; j<width; j++) {
      fscanf(fp_input, "%u", &tmp_i);
      mat[i][j] = (unsigned char)(tmp_i);
    }
  }
	
  point seed(210, 127);//种子点
  solution(mat, height, width, seed, 60, filename_out);//主要函数

  fclose(fp_input);
  for (int i = 0; i<height; i++) {
    delete[] mat[i];
  }
  delete[] mat;
}
