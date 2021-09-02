/*
"Disofenix 117"
Diego Esteban Suarez C.		1201689
Universidad Militar Nueva Granada
2019
*/
#include <GL/glut.h>
#include "Header.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
/*
#include "eigen-eigen-323c052e1731/Eigen/Dense"
#include "eigen-eigen-323c052e1731/Eigen/Geometry"
*/
#define PI 3.14159265

enum camera
{
	CargarMatriz,
	LookAt,
	Perspectiva,
	Ortogonal,
};
enum Tim
{
	on,
	off,
};
Tim temporizador;

camera Cam;
int     Xcu, Ycu;             // Coordenadas del cursor


GLfloat cop_x = 10;
GLfloat cop_y = 10;
GLfloat cop_z = 10;
GLfloat atx = 0.0;
GLfloat aty = 0.0;
GLfloat atz = 0.0;

using namespace Eigen;
using namespace std;

bool control; 
float n=0;
using Eigen::Matrix4f;
using Eigen::Vector4f;

int opp;															//opcion del menu de display

Vector4f cop, at;													//vectores de camara (Rotacion)
Vector3f Cop, ux, uy, uz, Pref, Np, np, up, vp;						//vectores de camara (matriz)
Matrix4f MTrans, MRotY, MRotZ, Mr;									//Matrices de transformaciones
Matrix4f Mview, Mortog, Mpers;										//Matrices de camara
GLfloat *mc, *m2, *m3, uxcop, vxcop, nxcop;							//Variables de camara (matriz)

bool Camara;														//cambio de tipo de camara

float W, H;
float near = 1;

float Arriba = 15, Abajo = 0, Derecha = 0, Izquierda = 15, Cerca = 1, Lejos = 50;	//Tubos de vista
const float asp = 1;
float ang = 45, theta = (PI / 180);									//conversion del angulo
int recur = 3;														//recursiones 
int cantPuntos = 9;
int PuntosSuperficie = 4;
float tx = 0, ty = 2, tz = 0;
Vertices Points[10],PointsSup[4][4];

GLfloat PosLuzX=0, PosLuzY=15, PosLuzZ=0;
GLfloat Amb0 = 0.5, Dif0=0.5, Spec0=0.5;


void init(void)
{
	
	//glClearColor(0.7, 0.7, 0.7, 0.0);
	glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	Cam = CargarMatriz;
	temporizador =on;

}

//////////////////////////////////////////////////////////////////////////
//ESTRUCTURAS
//////////////////////////////////////////////////////////////////////////
//---------------------------------Elefante

//Pierna tracera derecha
Vertices PTv1{ 78.3,-127,-212.8 }, PTv2{ 86.4,-253.7,-226.7 }, PTv3{ 103.3,-254.3,-207.4 },
PTv4{ 90.1,-128.7,-197.8 }, PTv5{ 83.6,-253.06,-180.1 }, PTv6{ 87.2,-127.9,-178.9 },
PTv7{ 55,-253,-179.7 }, PTv8{ 53.3,-147.5,-178.8 }, PTv9{ 43.3,-254.5,-207 },
PTv10{ 53.3,-148.3 ,-200.1 }, PTv11{ 67.6,-253.8,-229.6 }, PTv12{ 53.2,-128.6,-220.3 },
PTv13{ 83.2,-24.1,-228.9 }, PTv14{ 106,-9,-186.6 }, PTv15{ 100,-7,-141.8 },
PTv16{ 34.2,-77.3,-141.8 }, PTv17{ 34.2,-78.8,-195.2 }, PTv18{ 34.2,-44.7,-244.5 };
Vertices listaT[18] = { PTv1,PTv2,PTv3,PTv4,PTv5,PTv6,PTv7,PTv8,PTv9,PTv10,PTv11,PTv12,PTv13,PTv14,PTv15,PTv16,PTv17,PTv18 };
Caras CarasPiernaT[12] = {
{listaT[0],listaT[1],listaT[2],listaT[3]},{listaT[3],listaT[2],listaT[4],listaT[5]},{listaT[5],listaT[4],listaT[6],listaT[7]},{listaT[7],listaT[6],listaT[8],listaT[9]},
{listaT[9],listaT[8],listaT[10],listaT[11]},{listaT[11],listaT[10],listaT[1],listaT[0]},{listaT[12],listaT[0],listaT[3],listaT[13]},{listaT[13],listaT[3],listaT[5],listaT[14]},
{listaT[14],listaT[5],listaT[7],listaT[15]},{listaT[15],listaT[7],listaT[9],listaT[16]},{listaT[16],listaT[9],listaT[11],listaT[17]},{listaT[17],listaT[11],listaT[0],listaT[12]} };

//Pierna Delantera Derecha
Vertices PDv1{ 91.4,-156.3,-4 }, PDv2{ 108,-256.7,-0.9 }, PDv3{ 118.1,-256.5,19.8 },
PDv4{ 94.1,-151.7,17.9 }, PDv5{ 105.6,-256.2,43.3 }, PDv6{ 86.5,-146.5,42.8 },
PDv7{ 90,-256.5,43.9, }, PDv8{ 70.7,-169.1,43 }, PDv9{ 76.7,-256.4,19.3 },
PDv10{ 64.9,-173.5,17.7 }, PDv11{ 89.4,-256.3,-2.5 }, PDv12{ 70.5,-177.5,-4.7 },
PDv13{ 77.1,-3.4,-45.1 }, PDv14{ 71.1,11.4,-2.7 }, PDv15{ 64.3,28.3,45.1 },
PDv16{ 35,-46,45.1 }, PDv17{ 35,-61.1,-2.7 }, PDv18{ 35,-74.4,-45.1 };
Vertices listaD[18] = { PDv1,PDv2,PDv3,PDv4,PDv5,PDv6,PDv7,PDv8,PDv9,PDv10,PDv11,PDv12,PDv13,PDv14,PDv15,PDv16,PDv17,PDv18 };
Caras CarasPiernaD[12] = {
	{listaD[0],listaD[1],listaD[2],listaD[3]},{listaD[3],listaD[2],listaD[4],listaD[5]},{listaD[5],listaD[4],listaD[6],listaD[7]},{listaD[7],listaD[6],listaD[8],listaD[9]},
{listaD[9],listaD[8],listaD[10],listaD[11]},{listaD[11],listaD[10],listaD[1],listaD[0]},{listaD[12],listaD[0],listaD[3],listaD[13]},{listaD[13],listaD[3],listaD[5],listaD[14]},
{listaD[14],listaD[5],listaD[7],listaD[15]},{listaD[15],listaD[7],listaD[9],listaD[16]},{listaD[16],listaD[9],listaD[11],listaD[17]},{listaD[17],listaD[11],listaD[0],listaD[12]} };

//cuerpo
Vertices Cv1{ 0,-9.5,-261.5 }, Cv2{ 0,-44.7,-239.1 }, Cv3{ 34.2,-44.7,-244.5 }, Cv4{ 18.2,-9.6,-261.5 }, Cv5{ 0,-77.3,-141.8 }, Cv6{ 34.2,-78.8,-195.2 }, Cv7{ 0,-77.3,-141.8 }, Cv8{ 34.2,-77.3,-141.8 }, Cv9{ 0,-74.4,45.1 }, Cv10{ 35,-74.4,-45.1 },
Cv11{ 0,-61.1,-2.7 }, Cv12{ 35,-61.1,-2.7 }, Cv13{ 0,-46,45.1 }, Cv14{ 35,-46,45.1 }, Cv15{ 0,-8.3,84.7 }, Cv16{ 34.9,-8.4,84.7 }, Cv17{ 63.2,51.6,84.7 }, Cv18{ 64.3,28.3,45.1 }, Cv19{ 45.2,104.5,45.1 }, Cv20{ 48,97.6,84.7 },
Cv21{ 0,126.8,45.1 }, Cv22{ 0,125.5,84.7 }, Cv23{ 45.2,82.6,-2.7 }, Cv24{ 71.1,11.4,-2.7, }, Cv25{ 0,107.5,-2.7 }, Cv26{ 0,126.8,45.1 }, Cv27{ 45.2,69,-45.1 }, Cv28{ 77.1,-3.4,-45.1 }, Cv29{ 0,90.5,-45.1 }, Cv30{ 0,107.5,-2.7 },
Cv31{ 100.3,-7,-141.8 }, Cv32{ 44.4,63.2,-141.8 }, Cv33{ 0,86.1,-141.8 }, Cv34{ 106,-9,-186.6 }, Cv35{ 60.1,71.3,-195.2 }, Cv36{ 0,85.4,-195.2 }, Cv37{ 48,51.3,-239 }, Cv38{ 83.2,-24.1,-228.9 }, Cv39{ 0,51.3,-239 }, Cv40{ 21.6,7.4,-258.1 },
Cv41{ 16.1,28,-262 }, Cv42{ 0,23.2,-261.5 };
Vertices PuntosCuerpo[42] = { Cv1,Cv2,Cv3,Cv4,Cv5,Cv6,Cv7,Cv8,Cv9,Cv10,Cv11,Cv12,Cv13,Cv14,Cv15,Cv16,Cv17,Cv18,Cv19,Cv20,Cv21,Cv22,Cv23,Cv24,Cv25,Cv26,Cv27,Cv28,Cv29,Cv30,Cv31,Cv32,Cv33,Cv34,Cv35,Cv36,Cv37,Cv38,Cv39,Cv40,Cv41,Cv42 };
Caras Cuerpo[24] = {
{PuntosCuerpo[0],PuntosCuerpo[1],PuntosCuerpo[2],PuntosCuerpo[3]},{PuntosCuerpo[1],PuntosCuerpo[4],PuntosCuerpo[5],PuntosCuerpo[2]},{PuntosCuerpo[4],PuntosCuerpo[6],PuntosCuerpo[7],PuntosCuerpo[5]},
{PuntosCuerpo[6],PuntosCuerpo[8],PuntosCuerpo[9],PuntosCuerpo[7]},{PuntosCuerpo[8],PuntosCuerpo[10],PuntosCuerpo[11],PuntosCuerpo[9]},{PuntosCuerpo[10],PuntosCuerpo[12],PuntosCuerpo[13],PuntosCuerpo[11]},
{PuntosCuerpo[12],PuntosCuerpo[14],PuntosCuerpo[15],PuntosCuerpo[13]},{PuntosCuerpo[13],PuntosCuerpo[15],PuntosCuerpo[16],PuntosCuerpo[17]},{PuntosCuerpo[18],PuntosCuerpo[17],PuntosCuerpo[16],PuntosCuerpo[19]},
{PuntosCuerpo[20],PuntosCuerpo[18],PuntosCuerpo[19],PuntosCuerpo[21]},{PuntosCuerpo[22],PuntosCuerpo[23],PuntosCuerpo[17],PuntosCuerpo[18]},{PuntosCuerpo[24],PuntosCuerpo[22],PuntosCuerpo[18],PuntosCuerpo[25]},
{PuntosCuerpo[26],PuntosCuerpo[27],PuntosCuerpo[23],PuntosCuerpo[22]},{PuntosCuerpo[28],PuntosCuerpo[26],PuntosCuerpo[22],PuntosCuerpo[29]},{PuntosCuerpo[30],PuntosCuerpo[7],PuntosCuerpo[9],PuntosCuerpo[27]},
{PuntosCuerpo[31],PuntosCuerpo[30],PuntosCuerpo[27],PuntosCuerpo[26]},{PuntosCuerpo[32],PuntosCuerpo[31],PuntosCuerpo[26],PuntosCuerpo[28]},{PuntosCuerpo[33],PuntosCuerpo[30],PuntosCuerpo[31],PuntosCuerpo[34]},
{PuntosCuerpo[35],PuntosCuerpo[34],PuntosCuerpo[31],PuntosCuerpo[32]},{PuntosCuerpo[36],PuntosCuerpo[37],PuntosCuerpo[33],PuntosCuerpo[34]},{PuntosCuerpo[38],PuntosCuerpo[36],PuntosCuerpo[34],PuntosCuerpo[35]},
{PuntosCuerpo[39],PuntosCuerpo[37],PuntosCuerpo[36],PuntosCuerpo[40]},{PuntosCuerpo[41],PuntosCuerpo[40],PuntosCuerpo[36],PuntosCuerpo[38]},{PuntosCuerpo[3],PuntosCuerpo[2],PuntosCuerpo[37],PuntosCuerpo[39]} };

//Cabeza
Vertices kv1{ 48,97.6,84.7 }, kv2{ 63.2,51.6,84.7 }, kv3{ 128.9,55.2,67.3 }, kv4{ 112.9,117.1,73 }, kv5{ 34.9,-8.4,84.7 }, kv6{ 99.6,-47,88.6 }, kv7{ 113.3,56.9,87.3 }, kv8{ 109.5,99.2,99.3 }, kv9{ 44.6,92.8,111 }, kv10{ 34.9,-8.4,84.7 },
kv11{ 37.6,12.2,111 }, kv12{ 47.6,52.7,111 }, kv13{ 0,125.5,84.7 }, kv14{ 0,119,111 }, kv15{ 0,-8.3,84.7 }, kv16{ 0,-0.9,110.8 }, kv17{ 0,10.5,145.8 }, kv18{ 21.4,24,146 }, kv19{ 29.8,43.7,146 }, kv20{ 23.4,70.8,129.8 },
kv21{ 0,88.1,145.9 }, kv22{ 0,10.1,176.1 }, kv23{ 15.7,21.4,178.5 }, kv24{ 19.6,30.8,180.4 }, kv25{ 20.8,47.3,180.8 }, kv26{ 0,62,179.5 }, kv27{ 0,-45.9,240.7 }, kv28{ 15.2,-37.1,242.6 }, kv29{ 15.2,-29.3,244.1 }, kv30{ 10.4,-19.8,254.1 },
kv31{ 0,-20.2,253.9 }, kv32{ 0,-118,303.2 }, kv33{ 4.2,-115.9,303 }, kv34{ 6.7,-108.1,302.8 }, kv35{ 4.9,-100.1,305.9 }, kv36{ 0,-99.9,303.9 };
Vertices PuntosCabeza[36] = { kv1,kv2,kv3,kv4,kv5,kv6,kv7,kv8,kv9,kv10,kv11,kv12,kv13,kv14,kv15,kv16,kv17,kv18,kv19,kv20,kv21,kv22,kv23,kv24,kv25,kv26,kv27,kv28,kv29,kv30,kv31,kv32,kv33,kv34,kv35,kv36 };
Caras CarasCabeza[26] = {
{PuntosCabeza[0],PuntosCabeza[1],PuntosCabeza[2],PuntosCabeza[3]},{PuntosCabeza[1],PuntosCabeza[4],PuntosCabeza[5],PuntosCabeza[2]},{PuntosCabeza[3],PuntosCabeza[2],PuntosCabeza[6],PuntosCabeza[7]},{PuntosCabeza[0],PuntosCabeza[3],PuntosCabeza[7],PuntosCabeza[8]},
{PuntosCabeza[2],PuntosCabeza[5],PuntosCabeza[8],PuntosCabeza[6]},{PuntosCabeza[5],PuntosCabeza[9],PuntosCabeza[10],PuntosCabeza[8]},{PuntosCabeza[6],PuntosCabeza[8],PuntosCabeza[10],PuntosCabeza[11]},{PuntosCabeza[7],PuntosCabeza[6],PuntosCabeza[11],PuntosCabeza[8]},
{PuntosCabeza[12],PuntosCabeza[0],PuntosCabeza[8],PuntosCabeza[13]},{PuntosCabeza[10],PuntosCabeza[4],PuntosCabeza[14],PuntosCabeza[15]},{PuntosCabeza[10],PuntosCabeza[15],PuntosCabeza[16],PuntosCabeza[17]},{PuntosCabeza[11],PuntosCabeza[10],PuntosCabeza[17],PuntosCabeza[18]},
{PuntosCabeza[8],PuntosCabeza[11],PuntosCabeza[18],PuntosCabeza[19]},{PuntosCabeza[13],PuntosCabeza[8],PuntosCabeza[19],PuntosCabeza[20]},{PuntosCabeza[17],PuntosCabeza[16],PuntosCabeza[21],PuntosCabeza[22]},{PuntosCabeza[18],PuntosCabeza[17],PuntosCabeza[22],PuntosCabeza[23]},
{PuntosCabeza[19],PuntosCabeza[18],PuntosCabeza[23],PuntosCabeza[24]},{PuntosCabeza[20],PuntosCabeza[19],PuntosCabeza[24],PuntosCabeza[25]},{PuntosCabeza[22],PuntosCabeza[21],PuntosCabeza[26],PuntosCabeza[27]},{PuntosCabeza[23],PuntosCabeza[22],PuntosCabeza[27],PuntosCabeza[29]},
{PuntosCabeza[24],PuntosCabeza[23],PuntosCabeza[28],PuntosCabeza[29]},{PuntosCabeza[25],PuntosCabeza[24],PuntosCabeza[29],PuntosCabeza[30]},{PuntosCabeza[27],PuntosCabeza[26],PuntosCabeza[31],PuntosCabeza[32]},{PuntosCabeza[28],PuntosCabeza[27],PuntosCabeza[32],PuntosCabeza[33]},
{PuntosCabeza[29],PuntosCabeza[28],PuntosCabeza[33],PuntosCabeza[34]},{PuntosCabeza[30],PuntosCabeza[29],PuntosCabeza[34],PuntosCabeza[35]} };

//Cola
Vertices Cov1{ 0,-85.7,-331.8 }, Cov2{ 11.4,-87.3,-331.9 }, Cov3{ 16.1,18,-262 }, Cov4{ 0,23.2,-261.5 }, Cov5{ 13,-90.4,-330.7 }, Cov6{ 21.3,7.4,-258.1 }, Cov7{ 12.1,-95.5,-331.8 }, Cov8{ 18.2,-9.6,-261.5 }, Cov9{ 0,-95.5,-331.8 }, Cov10{ 0,-9.5,-261.5 };
Vertices PuntosCola[10] = { Cov1,Cov2,Cov3,Cov4,Cov5,Cov6,Cov7,Cov8,Cov9,Cov10 };
Caras CarasCola[4] = {
{PuntosCola[0],PuntosCola[1],PuntosCola[2],PuntosCola[3]},{PuntosCola[1],PuntosCola[4],PuntosCola[5],PuntosCola[2]},{PuntosCola[4],PuntosCola[6],PuntosCola[7],PuntosCola[5]},{PuntosCola[6],PuntosCola[8],PuntosCola[9],PuntosCola[7]} };


//---------------------------------CUBO
//coordenadas
Vertices CuboV1{ -1, 1, -1 }, CuboV2{ -1,-1,-1 }, CuboV3{ 1,-1,-1 }, CuboV4{ 1,1,-1 }, CuboV5{ 1,1,1 }, CuboV6{ 1,-1,1 }, CuboV7{ -1,1,1 }, CuboV8{ -1,-1,1 };
//vertices
Vertices PuntosCubo[8] = { CuboV1,CuboV2 ,CuboV3 ,CuboV4 ,CuboV5 ,CuboV6 ,CuboV7 ,CuboV8 };
//aristas
Aristas AristasCubo[12] = {
	{PuntosCubo[0],PuntosCubo[1]},{PuntosCubo[0],PuntosCubo[3]},{PuntosCubo[0],PuntosCubo[6]},{PuntosCubo[1],PuntosCubo[7]},
{PuntosCubo[1],PuntosCubo[2]},{PuntosCubo[2],PuntosCubo[3]},{PuntosCubo[6],PuntosCubo[7]},{PuntosCubo[7],PuntosCubo[5]},
{PuntosCubo[5],PuntosCubo[2]},{PuntosCubo[5],PuntosCubo[4]},{PuntosCubo[4],PuntosCubo[3]},{PuntosCubo[4],PuntosCubo[6]} };
//Caras
Caras CarasCubo[6] = {
{PuntosCubo[0],PuntosCubo[1],PuntosCubo[2],PuntosCubo[3]},
{PuntosCubo[0],PuntosCubo[3],PuntosCubo[4],PuntosCubo[6]},
{PuntosCubo[6],PuntosCubo[7],PuntosCubo[1],PuntosCubo[0]},
{PuntosCubo[6],PuntosCubo[4],PuntosCubo[5],PuntosCubo[7]},
{PuntosCubo[4],PuntosCubo[3],PuntosCubo[2],PuntosCubo[5]},
{PuntosCubo[1],PuntosCubo[7],PuntosCubo[5],PuntosCubo[2]} };

//---------------------------------FIGURA 2D
Vector4f F2Dv1{ -2,0,3,1 }, F2Dv2{ 2,0,3,1 }, F2Dv3{ 5,0,0,1 }, F2Dv4{ 2,0,-3,1 }, F2Dv5{ -2,0,-3,1 }, F2Dv6{ -5,0,0,1 };
Vector4f PuntosF2D[6] = { F2Dv1,F2Dv2,F2Dv3,F2Dv4,F2Dv5,F2Dv6 };

Vector3f ptsControl[9];

//////////////////////////////////////////////////////////////////////////
//FUNCIONES DE TRABAJO
//////////////////////////////////////////////////////////////////////////
//---------------------------------Funciones Operadores
Vector3f centro(int n, Caras lista[])
{
	Vector3f Pc;
	Pc[0] = lista[n].A.x + lista[n].B.x + lista[n].C.x + lista[n].D.x;
	Pc[1] = lista[n].A.y + lista[n].B.y + lista[n].C.y + lista[n].D.y;
	Pc[2] = lista[n].A.z + lista[n].B.z + lista[n].C.z + lista[n].D.z;
	Pc = Pc / 4;
	return Pc;

}
void Normal(Caras lista[], int J)
{
	//auxiliares
	Vector3f P, Q, Pc, Pf, N, n;
	float A, B, C, D;
	Vector4f aux[2], final[2];
	Matrix4f espejo;
	espejo <<
		-1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;


	for (int i = 0; i < J; i++)
	{
		//Resolviendo coheficientes para el vector normal de cada cara
		A = lista[i].A.y*(lista[i].B.z - lista[i].C.z) + lista[i].B.y*(lista[i].C.z - lista[i].A.z) + lista[i].C.y*(lista[i].A.z - lista[i].B.z);
		B = lista[i].A.z*(lista[i].B.x - lista[i].C.x) + lista[i].B.z*(lista[i].C.x - lista[i].A.x) + lista[i].C.z*(lista[i].A.x - lista[i].B.x);
		C = lista[i].A.x*(lista[i].B.y - lista[i].C.y) + lista[i].B.x*(lista[i].C.y - lista[i].A.y) + lista[i].C.x*(lista[i].A.y - lista[i].B.y);
		D = -lista[i].A.x*((lista[i].B.y*lista[i].A.z) - (lista[i].C.y*lista[i].B.z)) - lista[i].B.x*((lista[i].C.y*lista[i].A.z) - (lista[i].A.y*lista[i].C.z)) - lista[i].C.x*((lista[i].A.y*lista[i].B.z) - (lista[i].B.y*lista[i].A.z));
		//asignando Valores
		N << A, B, C;
		//normalizando
		n = N / (sqrt(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]));
		//vector central de cada cara
		Pc = centro(i, lista);
		//vector final normal
		Pf = Pc + -5 * n;
		//dibujando
		glPushMatrix();
		glLineWidth(3);
		glColor3f(1, 1, 1);
		glBegin(GL_LINE_STRIP);
		glVertex3f(Pc[0], Pc[1], Pc[2]);
		glVertex3f(Pf[0], Pf[1], Pf[2]);
		glEnd();
		glPopMatrix();
		//reflejo
		glPushMatrix();
		aux[0] << Pc[0], Pc[1], Pc[2], 1;
		aux[1] << Pf[0], Pf[1], Pf[2], 1;
		final[0] = espejo * aux[0];
		final[1] = espejo * aux[1];
		glLineWidth(3);
		glColor3f(1, 1, 1);
		glBegin(GL_LINE_STRIP);
		glVertex3f(final[0][0], final[0][1], final[0][2]);
		glVertex3f(final[1][0], final[1][1], final[1][2]);
		glEnd();
		glPopMatrix();

	}
}
void PintarPuntos(Vertices lista[], int n)
{
	Vector4f aux, final;
	Matrix4f espejo;
	espejo <<
		-1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	float r = 1, g = 0, b = 0.3;
	for (int i = 0; i < n; i++)
	{
		glPushMatrix();
		glColor3f(r, g, b);
		glPointSize(10);
		glBegin(GL_POINTS);
		glVertex3f(lista[i].x, lista[i].y, lista[i].z);

		//reflejo
		aux << lista[i].x, lista[i].y, lista[i].z, 1;
		final = espejo * aux;
		glVertex3f(final[0], final[1], final[2]);

		glEnd();
		glPopMatrix();
		r -= 0.02;
		g += 0.02;
	}
}
void PintarCaras(Caras lista[], int n)
{
	Vector4f aux[4], final[4];
	Matrix4f espejo;
	espejo <<
		-1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	float r = 1, g = 0, b = 0.5;
	for (int i = 0; i < n; i++)
	{
		glPushMatrix();
		glColor3f(r, g, b);
		glBegin(GL_QUADS);
		glVertex3f(lista[i].A.x, lista[i].A.y, lista[i].A.z);
		glVertex3f(lista[i].B.x, lista[i].B.y, lista[i].B.z);
		glVertex3f(lista[i].C.x, lista[i].C.y, lista[i].C.z);
		glVertex3f(lista[i].D.x, lista[i].D.y, lista[i].D.z);
		glEnd();
		glPopMatrix();

		//reflejo
		aux[0] << lista[i].A.x, lista[i].A.y, lista[i].A.z, 1;
		aux[1] << lista[i].B.x, lista[i].B.y, lista[i].B.z, 1;
		aux[2] << lista[i].C.x, lista[i].C.y, lista[i].C.z, 1;
		aux[3] << lista[i].D.x, lista[i].D.y, lista[i].D.z, 1;
		final[0] = espejo * aux[0];
		final[1] = espejo * aux[1];
		final[2] = espejo * aux[2];
		final[3] = espejo * aux[3];
		glPushMatrix();
		glBegin(GL_QUADS);
		glVertex3f(final[0][0], final[0][1], final[0][2]);
		glVertex3f(final[1][0], final[1][1], final[1][2]);
		glVertex3f(final[2][0], final[2][1], final[2][2]);
		glVertex3f(final[3][0], final[3][1], final[3][2]);
		glEnd();
		glPopMatrix();

		r -= 0.02;
		b += 0.02;
	}
}
void PintarPunto(Vertices P)
{
	glPushMatrix();
	glPointSize(10);
	glBegin(GL_POINTS);
	glVertex3f(P.x, P.y, P.z);
	glEnd();
	glPopMatrix();
}

void PintarLinea(Vertices PuntoI, Vertices PuntoF)
{
	glPushMatrix();
	glLineWidth(0.5);

	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(PuntoI.x, PuntoI.y, PuntoI.z);
	glVertex3f(PuntoF.x, PuntoF.y, PuntoI.z);
	glEnd();
	glPopMatrix();
}
int factorial(int n)
{
	if (n <= 1)
	{
		return(1);
	}
	else
	{
		n = n * factorial(n - 1);
	}
	return n;
}
int coeficienteBinomial(int n, int k)
{
	float res;
	res = factorial(n) / (factorial(k)*factorial(n - k));
	return res;
}


//---------------------------------Elefante
void NormalesDante()
{
	Normal(CarasPiernaT, 12);
	Normal(CarasPiernaD, 12);
	Normal(CarasCabeza, 26);
	Normal(CarasCola, 4);
	Normal(Cuerpo, 24);

}
void PuntosDante()
{
	PintarPuntos(PuntosCabeza, 36);
	PintarPuntos(PuntosCola, 10);
	PintarPuntos(PuntosCuerpo, 43);
	PintarPuntos(listaD, 18);
	PintarPuntos(listaT, 18);
}
void CarasDante()
{
	PintarCaras(CarasCabeza, 26);
	PintarCaras(CarasCola, 4);
	PintarCaras(Cuerpo, 24);
	PintarCaras(CarasPiernaD, 12);
	PintarCaras(CarasPiernaT, 12);
}
void Dante()
{
	glPushMatrix();
	PintarPuntos(PuntosCabeza, 36);
	PintarPuntos(PuntosCola, 10);
	PintarPuntos(PuntosCuerpo, 43);
	PintarPuntos(listaD, 18);
	PintarPuntos(listaT, 18);
	PintarCaras(CarasCabeza, 26);
	PintarCaras(CarasCola, 4);
	PintarCaras(Cuerpo, 24);
	PintarCaras(CarasPiernaD, 12);
	PintarCaras(CarasPiernaT, 12);
	NormalesDante();
	glPopMatrix();
}


//---------------------------------BEZIER
Vertices BezierDoble(Vertices PT, double t)
{
	Vertices P;
	P.x = 0;
	P.y = 0;
	P.z = 0;
	for (int i = 0; i < PuntosSuperficie; i++)
	{
			P.x = P.x + coeficienteBinomial((PuntosSuperficie - 1), i)*pow(t, i)*pow((1 - t), (PuntosSuperficie - 1 - i))*PT.x;
			P.y = P.y + coeficienteBinomial((PuntosSuperficie - 1), i)*pow(t, i)*pow((1 - t), (PuntosSuperficie - 1 - i))*PT.y;
			P.z = P.z + coeficienteBinomial((PuntosSuperficie - 1), i)*pow(t, i)*pow((1 - t), (PuntosSuperficie - 1 - i))*PT.z;
	}
	return P;
}

Vertices BezierGeneral(Vertices PT[], double t)
{
	Vertices P;
	P.x = 0;
	P.y = 0;
	P.z = 0;
	for (int i = 0; i < cantPuntos; i++)
	{
		P.x = P.x + coeficienteBinomial((cantPuntos - 1), i)*pow(t, i)*pow((1 - t), (cantPuntos - 1 - i))*PT[i].x;
		P.y = P.y + coeficienteBinomial((cantPuntos - 1), i)*pow(t, i)*pow((1 - t), (cantPuntos - 1 - i))*PT[i].y;
		P.z = P.z + coeficienteBinomial((cantPuntos - 1), i)*pow(t, i)*pow((1 - t), (cantPuntos - 1 - i))*PT[i].z;
	}
	return P;
}
void CurvaBezier(float o)
{
	glPushMatrix();
	glScalef(0.02, 0.02, 0.02);
	Dante();
	glPopMatrix();
	if (temporizador == on)
	{
		//puntos de control
		Points[0] = { 0.97,0,10.28 }; Points[1] = { 6.47,0.53,10.12 }; Points[2] = { 10.85,2.15,7.02 }; Points[3] = { 11.3,5.43,-7.95 }; Points[4] = { -7.14,7.27,-14.87 };
		Points[5] = { -13.56,8.8,-3.94 }; Points[6] = { -14.17,13.41,5.53 }; Points[7] = { -6.39,14.35,7.95 }; Points[8] = { -2.51,14.24,9.16 };
		for (double u = 0; u <= o; u += 0.01)
		{
			glPushMatrix();
			Vertices Pf = BezierGeneral(Points, o);
			glColor3f(0, 1, 0);
			//PintarPunto(Pf);

			cop_x = Pf.x;
			cop_y = Pf.y;
			cop_z = Pf.z;

			glPopMatrix();

		}
		//pintar puntos de control
		for (int i = 0; i < cantPuntos; i++)
		{
			glPushMatrix();
			glColor3f(1, 0, 0);
			PintarPunto(Points[i]);
			glPopMatrix();

		}
	}
	else
	{
		temporizador = off;
		n = 0;
		cop_x = 10;
		cop_y = 10;
		cop_z = 10;
		glutIdleFunc(NULL);
	}

}
void Superficie()
{
	//puntos de control
	PointsSup[0][0] = { -1.5,-1.5,4 }; PointsSup[0][1] = { -0.5,-1.5,2 }; PointsSup[0][2] = { 0.5,-1.5,-1 }; PointsSup[0][3] = { 1.5,-1.5,2 };
	PointsSup[1][0] = { -1.5,-0.5,1 }; PointsSup[1][1] = { -0.5,-0.5,3 }; PointsSup[1][2] = { 0.5,-0.5,0 }; PointsSup[1][3] = { 1.5,0 - .5,-1 };
	PointsSup[2][0] = { -1.5,0.5,4 }; PointsSup[2][1] = { -0.5,-0.5,0 }; PointsSup[2][2] = { 0.5,0.5,3 }; PointsSup[2][3] = { 1.5,0.5,4 };
	PointsSup[3][0] = { -1.5,1.5,-2 }; PointsSup[3][1] = { -0.5,-1.5,-2 }; PointsSup[3][2] = { 0.5,1.5,0 }; PointsSup[3][3] = { 1.5,1.5,-1 };
	Vector3f aux;
	aux << 0, 0, 0;
	for (int i = 0; i < PuntosSuperficie; i++)
	{
		for (int j = 0; j < PuntosSuperficie; j++)
		{
			for (double u = 0; u <= 1; u += 0.01)
			{
				for (double v = 0; v <= 1; v += 0.01)
				{
					glPushMatrix();
					Vertices p1, p2;
					p1 = BezierDoble(PointsSup[i][j], u);
					p2 = BezierDoble(PointsSup[i][j], v);

					/*
					aux[0] = PointsSup[i][j].x;
					aux[1] = PointsSup[i][j].y;
					aux[2] = PointsSup[i][j].z;
					Vector3f P1 = BezierDoble(aux, u);
					Vector3f P2 = BezierDoble(aux, v);
					Vector3f Pf;
						Pf[0] = P1[0] * P2[0];
						Pf[1] = P1[1] * P2[1];
						Pf[2] = P1[2] * P2[2];
					glColor3f(0, 1, 0);
					Vertices pf;
					pf.x = Pf[0];
					pf.y = Pf[1];
					pf.z = Pf[2];
					*/
					glColor3f(0, 1, 0);
					Vertices pf;
					pf.x = p1.x + p2.x;
					pf.y = p1.y + p2.y;
					pf.z = p1.z + p2.z;
					PintarPunto(pf);

					glPopMatrix();

				}
			}

			glPushMatrix();
			glColor3f(1, 0, 0);
			PintarPunto(PointsSup[i][j]);
			glPopMatrix();
		}
	}
	//pintar puntos de control
	for (int i = 0; i < PuntosSuperficie; i++)
	{
		for (int j = 0; j < PuntosSuperficie; j++)
		{
			glPushMatrix();
			glColor3f(1, 0, 0);
			PintarPunto(PointsSup[i][j]);
			glPopMatrix();
		}
	}

}


//---------------------------------Cubo
void CuboVertices()
{
	float r = 1, g = 0, b = 0.3;
	for (int i = 0; i < 8; i++)
	{
		glPushMatrix();
		glColor3f(r, g, b);
		glPointSize(10);
		glBegin(GL_POINTS);
		glVertex3f(PuntosCubo[i].x, PuntosCubo[i].y, PuntosCubo[i].z);
		glEnd();
		glPopMatrix();
		r -= 0.2;
		g += 0.2;
	}
}
void CuboAristas()
{
	float r = 1, g = 0, b = 0.5;
	for (int i = 0; i < 12; i++)
	{
		glPushMatrix();
		glLineWidth(3);
		glColor3f(r, g, b);
		glBegin(GL_LINE_STRIP);
		glVertex3f(AristasCubo[i].a.x, AristasCubo[i].a.y, AristasCubo[i].a.z);
		glVertex3f(AristasCubo[i].b.x, AristasCubo[i].b.y, AristasCubo[i].b.z);
		glEnd();
		glPopMatrix();
		r -= 0.05;
		g += 0.05;
	}

}
void CuboCaras()
{
	float r = 1, g = 0, b = 0.5;
	for (int i = 0; i < 6; i++)
	{
		glPushMatrix();
		glColor3f(r, g, b);
		glBegin(GL_QUADS);
		glVertex3f(CarasCubo[i].A.x, CarasCubo[i].A.y, CarasCubo[i].A.z);
		glVertex3f(CarasCubo[i].B.x, CarasCubo[i].B.y, CarasCubo[i].B.z);
		glVertex3f(CarasCubo[i].C.x, CarasCubo[i].C.y, CarasCubo[i].C.z);
		glVertex3f(CarasCubo[i].D.x, CarasCubo[i].D.y, CarasCubo[i].D.z);
		glEnd();
		glPopMatrix();
		r -= 0.2;
		b += 0.2;
	}
}
void CuboNormales()
{
	//auxiliares
	Vector3f P, Q, Pc, Pf, N, n;
	float A, B, C, D;

	for (int i = 0; i < 6; i++)
	{
		//Resolviendo coheficientes para el vector normal de cada cara
		A = CarasCubo[i].A.y*(CarasCubo[i].B.z - CarasCubo[i].C.z) + CarasCubo[i].B.y*(CarasCubo[i].C.z - CarasCubo[i].A.z) + CarasCubo[i].C.y*(CarasCubo[i].A.z - CarasCubo[i].B.z);
		B = CarasCubo[i].A.z*(CarasCubo[i].B.x - CarasCubo[i].C.x) + CarasCubo[i].B.z*(CarasCubo[i].C.x - CarasCubo[i].A.x) + CarasCubo[i].C.z*(CarasCubo[i].A.x - CarasCubo[i].B.x);
		C = CarasCubo[i].A.x*(CarasCubo[i].B.y - CarasCubo[i].C.y) + CarasCubo[i].B.x*(CarasCubo[i].C.y - CarasCubo[i].A.y) + CarasCubo[i].C.x*(CarasCubo[i].A.y - CarasCubo[i].B.y);
		D = -CarasCubo[i].A.x*((CarasCubo[i].B.y*CarasCubo[i].A.z) - (CarasCubo[i].C.y*CarasCubo[i].B.z)) - CarasCubo[i].B.x*((CarasCubo[i].C.y*CarasCubo[i].A.z) - (CarasCubo[i].A.y*CarasCubo[i].C.z)) - CarasCubo[i].C.x*((CarasCubo[i].A.y*CarasCubo[i].B.z) - (CarasCubo[i].B.y*CarasCubo[i].A.z));
		//asignando Valores
		N << A, B, C;
		//normalizando
		n = N / (sqrt(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]));
		//vector central de cada cara
		Pc = centro(i, CarasCubo);
		//vector final normal
		Pf = Pc + 3 * n;
		//dibujando
		glPushMatrix();
		glLineWidth(3);
		glColor3f(1, 1, 1);
		glBegin(GL_LINE_STRIP);
		glVertex3f(Pc[0], Pc[1], Pc[2]);
		glVertex3f(-Pf[0], -Pf[1], -Pf[2]);
		glEnd();
		glPopMatrix();
	}

}


//---------------------------------Fractales
void sierpinsky()
{
	GLfloat vertices[3][2] = { {0,0},{25,50},{50,0} };
	int i, j, k;
	GLfloat p[2] = { 7.5,5 };
	glBegin(GL_POINTS);
	for (k = 0; k < 5000; k++)
	{
		j = rand() % 3;
		p[0] = (p[0] + vertices[j][0]) / 2;
		p[1] = (p[1] + vertices[j][1]) / 2;
		glVertex2fv(p);

	}
	glEnd();
	glFlush();


}

void triangle(GLfloat *a, GLfloat *b, GLfloat *c)
{

	glVertex2fv(a);
	glVertex2fv(b);
	glVertex2fv(c);

}
void divide_triangle(GLfloat *a, GLfloat *b, GLfloat *c, int k)
{
	GLfloat ab[2], ac[2], bc[2];
	int j;
	if (k > 0)
	{
		for (j = 0; j < 2; j++) ab[j] = (a[j] + b[j]) / 2;
		for (j = 0; j < 2; j++) ac[j] = (a[j] + c[j]) / 2;
		for (j = 0; j < 2; j++) bc[j] = (b[j] + c[j]) / 2;

		divide_triangle(a, ab, ac, k - 1);
		divide_triangle(c, ac, bc, k - 1);
		divide_triangle(b, bc, ab, k - 1);
	}
	else
	{
		triangle(a, b, c);
	}
}
void triangulo()
{
	GLfloat v[3][2] = { {0,0},{25,50},{50,0} };

	glBegin(GL_TRIANGLES);
	divide_triangle(v[0], v[1], v[2], recur);
	glEnd();
	glFlush();
}

void Arbol(int n)
{
	if (n > 0)
	{
		float r = 0.3, g = 0.4, b = 0.5;
		//lado izquierdo
		glPushMatrix();
		glTranslatef(-0.5, 1.0, 0);
		glRotatef(45, 0.0, 0.0, 1.0);
		glScalef(0.707, 0.707, 0.707);
		Arbol(n - 1);
		//lado derecho
		glPopMatrix();
		glColor3f(r, g, b);
		glPushMatrix();
		glTranslatef(0.5, 1.0, 0);
		glRotatef(-45, 0.0, 0.0, 1.0);
		glScalef(0.707, 0.707, 0.707);
		Arbol(n - 1);
		glPopMatrix();

		glutSolidCube(1);
		r = r + 0.2;
		g = g + 0.1;
		b = b + 0.3;
	}
}
void Arbol_Pitagoras()
{
	Arbol(recur);
}

void GenerarCaras(Vector4f P1, Vector4f P2, Vector4f Q2, Vector4f Q1)
{
	glPushMatrix();
	glBegin(GL_QUADS);
	glVertex3f(P1[0], P1[1], P1[2]);
	glVertex3f(P2[0], P2[1], P2[2]);
	glVertex3f(Q2[0], Q2[1], Q2[2]);
	glVertex3f(Q1[0], Q1[1], Q1[2]);
	glEnd();
	glPopMatrix();
}
void GenerarTapas(Vector4f P1, Vector4f P2, Vector4f P3, Vector4f P4, Vector4f P5, Vector4f P6)
{
	glPushMatrix();
	glBegin(GL_POLYGON);
	glVertex3f(P1[0], P1[1], P1[2]);
	glVertex3f(P2[0], P2[1], P2[2]);
	glVertex3f(P3[0], P3[1], P3[2]);
	glVertex3f(P4[0], P4[1], P4[2]);
	glVertex3f(P5[0], P5[1], P5[2]);
	glVertex3f(P6[0], P6[1], P6[2]);
	glEnd();
	glPopMatrix();

}
void Prisma()
{
	//auxiliares
	Vector4f P[6], Q[6];
	Matrix4f MTrans;
	//asignando figura 2D a P auxiliar
	for (int i = 0; i < 6; i++)
	{
		P[i] = PuntosF2D[i];

	}
	// Matriz translacion
	MTrans << 1, 0, 0, tx,
		0, 1, 0, ty,
		0, 0, 1, tz,
		0, 0, 0, 1;
	float r = 1, g = 0, b = 0.5;
	glColor3f(0, 0, 0);
	GenerarTapas(P[5], P[4], P[3], P[2], P[1], P[0]);
	for (int j = 0; j < recur; j++)
	{
		for (int i = 0; i < 6; i++)
		{
			Q[i] = MTrans * P[i];
			glPushMatrix();
			glColor3f(r, g, b);
			glPointSize(10);
			glBegin(GL_POINTS);
			glVertex3f(P[i][0], P[i][1], P[i][2]);
			glVertex3f(Q[i][0], Q[i][1], Q[i][2]);
			glEnd();
			glPopMatrix();
			r -= 0.02;
			b += 0.02;

		}
		GenerarCaras(P[0], P[1], Q[1], Q[0]);
		GenerarCaras(P[1], P[2], Q[2], Q[1]);
		GenerarCaras(P[2], P[3], Q[3], Q[2]);
		GenerarCaras(P[3], P[4], Q[4], Q[3]);
		GenerarCaras(P[4], P[5], Q[5], Q[4]);
		GenerarCaras(P[5], P[0], Q[0], Q[5]);

		//reemplando
		for (int i = 0; i < 6; i++)
		{
			P[i] = Q[i];
		}
		glColor3f(0, 0, 0);
		GenerarTapas(P[0], P[1], P[2], P[3], P[4], P[5]);

	}

}

//---------------------------------rotaciones camara
void RotarCamaraY(float beta)
{
	Vector4f vectorF;


	cop[0] = cop_x;	cop[1] = cop_y;	cop[2] = cop_z; cop[3] = 1;
	at[0] = atx; at[2] = aty; at[2] = atz; at[3] = 1;

	MTrans << 1, 0, 0, -at[0],
		0, 1, 0, 0,
		0, 0, 1, -at[2],
		0, 0, 0, 1;
	MRotY << cosf(beta*theta), 0, sinf(beta*theta), 0,
		0, 1, 0, 0,
		-sinf(beta*theta), 0, cosf(beta*theta), 0,
		0, 0, 0, 1;

	vectorF = MTrans.inverse() * MRotY* MTrans* cop;
	cop_x = vectorF[0]; cop_y = vectorF[1]; cop_z = vectorF[2];


}
void RotarCamaraZ(float beta)
{
	Vector4f vectorF;


	cop[0] = cop_x;	cop[1] = cop_y;	cop[2] = cop_z; cop[3] = 1;
	at[0] = atx; at[2] = aty; at[2] = atz; at[3] = 1;

	MTrans << 1, 0, 0, -at[0],
		0, 1, 0, -at[1],
		0, 0, 1, 0,
		0, 0, 0, 1;
	MRotZ << cosf(beta*theta), -sinf(beta*theta), 0, 0,
		sinf(beta*theta), cosf(beta*theta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	vectorF = MTrans.inverse() * MRotZ* MTrans* cop;
	cop_x = vectorF[0]; cop_y = vectorF[1]; cop_z = vectorF[2];


}




//---------------------------------HUD
void camara(void)
{
	//Mariz unitaria (en vectores)
	ux << 1, 0, 0;
	uy << 0, 1, 0; //// Vp
	uz << 0, 0, 1;
	//cop
	Cop << cop_x, cop_y, cop_z;
	//Pref (At)
	Pref << atx, aty, atz;
	//N
	Np = Cop - Pref;
	//n (N normalizado)
	np = Np / sqrt(Np[0] * Np[0] + Np[1] * Np[1] + Np[2] * Np[2]);
	//U
	up = uy.cross(np);
	//u (U normalizado)
	up = up / sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
	//v
	vp = np.cross(up);

	//Producto punto del Cop
	uxcop = up.dot(Cop);
	vxcop = vp.dot(Cop);
	nxcop = np.dot(Cop);


	//matriz de vista
	Mview << up[0], up[1], up[2], -uxcop,
		vp[0], vp[1], vp[2], -vxcop,
		np[0], np[1], np[2], -nxcop,
		0, 0, 0, 1;
	//matriz Ortogonal
	Mortog << 2 / (Derecha - Izquierda), 0, 0, -(Derecha + Izquierda) / (Derecha - Izquierda),
		0, 2 / (Arriba - Abajo), 0, -(Arriba + Abajo) / (Arriba - Abajo),
		0, 0, -2 / (Lejos - Cerca), -((Lejos + Cerca) / (Lejos - Cerca)),
		0, 0, 0, 1;
	//matriz Perspectiva 
	Mpers << 1 / (asp*tan(ang / 2)), 0, 0, 0,
		0, 1 / tan(ang / 2), 0, 0,
		0, 0, -(Lejos + Cerca) / (Lejos - Cerca), -2 * (Lejos*Cerca) / (Lejos - Cerca),
		0, 0, -1, 0;

	mc = &Mview(0);
	m2 = &Mpers(0);
	m3 = &Mortog(0);



}
void malla()
{
	glLineWidth(1);

	glColor3f(0.5, 0.0, 0.5);

	for (float i = -5.0; i < 5.0; i += 0.5)
	{
		glBegin(GL_LINES);
		glVertex3f(i, 0.0f, -5.0f);
		glVertex3f(i, 00.0f, 5.0f);
		glVertex3f(-5, 0, i);
		glVertex3f(5, 0, i);
		glEnd();
	}
}
void ejes()
{
	glPushMatrix();
	glLineWidth(4);
	glBegin(GL_LINES);
	//X
	glColor3f(1.0, 0.0, 0.0);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(2.5, 0, 0);
	//Y
	glColor3f(0.0, 1.0, 0.0);
	glVertex3f(0, 0.0f, 0.0f);
	glVertex3f(0, 2.5, 0);
	//Z
	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(0, 0.0f, 0.0f);
	glVertex3f(0, 0, 2.5);
	glEnd();
	glPopMatrix();
}
void ejemplo()
{
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_QUADS);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(1, 1, 0);
	glVertex3f(0, 1, 0);
	glEnd();
}

//////////////////////////////////////////////////////////////////////////
//FUNCIONES BASICAS OPENGL
//////////////////////////////////////////////////////////////////////////
void Luz1()
{
	GLfloat light_ambient0[] = { Amb0 , Amb0 ,Amb0 , 1.0 };
	GLfloat light_diffuse0[] = { Dif0 , Dif0 ,Dif0 , 1.0 };
	GLfloat light_specular0[] = { Spec0 , Spec0, Spec0 , 1.0 };
	GLfloat light_position0[] = { PosLuzX , PosLuzY , PosLuzZ , 1 };
	GLfloat light_direction0[] = { 0,0,0,1 };
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular0);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light_direction0);

	glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 45);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);

}
void Luz2()
{
	GLfloat light_ambient1[] = { 0.15 , 0.15 ,0.15 , 1.0 };
	GLfloat light_diffuse1[] = { 1.0 , 1.0 , 1.0 , 1.0 };
	GLfloat light_specular1[] = { 1.0 , 1.0 , 1.0 , 1.0 };
	GLfloat light_position1[] = { 10 , 10 , 10, 0.0 };
	GLfloat light_direction1[] = { 0,0,0,1 };
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular1);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light_direction1);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 45);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT1);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);

}
//---------------------------------funciones de Mouse


//---------------------------------funciones de teclado
void Onkeyboard(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_UP:
	{
		RotarCamaraZ(0.5);
		break;
	}
	case GLUT_KEY_DOWN:
	{
		RotarCamaraZ(-0.5);
		break;
	}
	case GLUT_KEY_RIGHT:
	{
		RotarCamaraY(1.5);

		break;
	}
	case GLUT_KEY_LEFT:
	{
		RotarCamaraY(-1.5);

		break;
	}

	}
	glutPostRedisplay();
}
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '-':
	{
		cop_x += 0.5;
		cop_y += 0.5;
		cop_z += 0.5;
		break;
	}
	case '+':
	{
		cop_x -= 0.5;
		cop_y -= 0.5;
		cop_z -= 0.5;
		break;
	}

	case'c':
	case'C':
	{
		Camara != Camara;
		break;
	}
	case 'a':
	case 'A':
	{
		PosLuzX += 0.5;
		break;
	}
	case 'd':
	case 'D':
	{
		PosLuzX -= 0.5;
		break;
	}
	case'w':
	case'W':
	{
		PosLuzY += 0.5;
		break;
	}
	case's':
	case'S':
	{
		PosLuzY -= 0.5;
		break;
	}
	case'q':
	case'Q':
	{
		PosLuzZ -= 0.5;
		break;
	}
	case'e':
	case'E':
	{
		PosLuzZ += 0.5;
		break;
	}
	case'r':
	case'R':
	{
		if (Amb0 <= 1)
		{
			Amb0 += 0.1;
		}
		else
		{
			Amb0 = 1;
		}
		break;
	}
	case'f':
	case'F':
	{
		if (Amb0 >= 0)
		{
			Amb0 -= 0.1;
		}
		else
		{
			Amb0 = 0;
		}
		break;
	}
	case't':
	case'T':
	{
		if (Dif0 <= 1)
		{
			Dif0 += 0.1;
		}
		else
		{
			Dif0 = 1;
		}
		break;
	}
	case'g':
	case'G':
	{
		if (Dif0 >= 0)
		{
			Dif0 -= 0.1;
		}
		else
		{
			Dif0 = 0;
		}
		break;
	}
	case'y':
	case'Y':
	{
		if (Spec0 <= 1)
		{
			Spec0 += 0.1;
		}
		else
		{
			Spec0 = 1;
		}
		break;
	}
	case'h':
	case'H':
	{
		if (Spec0 >= 0)
		{
			Spec0 -= 0.1;
		}
		else
		{
			Spec0 = 0;
		}
		break;
	}
	case 27: //esc
	{
		exit(0);
		break;
	}

	}
	cout<<"PosLuzX" << PosLuzX<<endl;
	cout << "PosLuzy" << PosLuzY<<endl;
	cout << "PosLuzZ" << PosLuzZ<<endl;
	glutPostRedisplay();

}
//---------------------------------funciones de display
void OnTimerGL(void)
{
	if (n < 1)
	{
			n += 0.002;

		glutPostRedisplay();

	}
	else
	{
		n = 0;
		temporizador = off;
		glutIdleFunc(NULL);
	}
}
void menu(int op)
{
	switch (op)
	{
		//cubo
		{
	case 1: //Vertices
	{
		opp = 1;
		break;
	}

	case 2://aristas
	{
		opp = 2;
		break;
	}
	case 3://caras
	{
		opp = 3;
		break;
	}
	case 4://Completo
	{
		opp = 4;
		break;
	}
	case 5://Normales
	{
		opp = 5;
		break;
	}
		}
		//Recurrencia
		{
	case 6://Sirpinsky I.
	{
		opp = 6;
		break;
	}
	case 7://Sierpinsky R.
	{
		opp = 7;
		break;
	}
	case 8://Arbol
	{
		opp = 8;
		break;
	}
	case 13:
	{
		opp = 13;
		break;
	}
		}
		//camaras
		{
	case 9://LookAt
	{
		Cam = LookAt;
		break;
	}
	case 10://Perspectiva
	{
		Cam = Perspectiva;
		break;
	}
	case 11://ortogonal
	{
		Cam = Ortogonal;

		break;
	}
	case 12://CargarMatriz
	{
		Cam = CargarMatriz;

		break;
	}
		}
		//Dante
		{
	case 14://completo
	{
		opp = 14;
		break;
	}
	case 15://vertices
	{
		opp = 15;
		break;
	}
	case 16://Caras
	{
		opp = 16;
		break;
	}

	case 17://normales
	{
		opp = 17;
		break;
	}
		}
		//BEZIER
		{
			case 18:
			{
				opp = 18;
				temporizador = on;
				break;
			}

		}
	case 0://salir
	{
		exit(0);
		break;
	}
	default:
	{
		break;
	}
	}

}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	//glEnable(GL_CULL_FACE);
	glLoadIdentity();

	Luz1();
	//Luz2();



	//material
	GLfloat mat_d[] = { 0.1 , 0.5 , 0.8 , 1.0 };
	GLfloat mat_s[] = { 1.0 , 1.0 , 1.0 , 1.0 };
	GLfloat low_sh[] = { 5.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_s);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_d);
	glMaterialfv(GL_FRONT, GL_SHININESS, low_sh);
	glEnable(GL_COLOR_MATERIAL);

	/*
	//Defino un material Rojo
	GLfloat mat_ambient_rojo[] = { 0.05, 0.05, 0.05, 1.0f };
	GLfloat mat_diffuse_rojo[] = { 0.8, 0.0, 0.0, 1.0f };
	GLfloat mat_specular_rojo[] = { 0.9, 0.8, 0.8, 1.0f };

	//Defino un material Azul
	GLfloat mat_ambient_azul[] = { 0.1, 0.1, 0.1, 1.0f };
	GLfloat mat_diffuse_azul[] = { 0.0, 0.0, 0.8, 1.0f };
	GLfloat mat_specular_azul[] = { 0.9, 0.9, 0.9, 1.0f };

	//Defino un material Verde
	GLfloat mat_ambient_verde[] = { 0.1, 0.1, 0.1, 1.0f };
	GLfloat mat_diffuse_verde[] = { 0.0, 0.7, 0.0, 1.0f };
	GLfloat mat_specular_verde[] = { 0.8, 0.8, 0.8, 1.0f };

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient_rojo);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_ambient_rojo);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_ambient_rojo);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);

	*/
	//camara en Matrices
	camara();
	//tipo de camara
	{
		if (Cam == LookAt)
		{
			//Punto de la camara, centro de vista, vector hacia arriba
			gluLookAt(cop_x, cop_y, cop_z, atx, aty, atz, 0.0, 1.0, 0.0);
		}
		if (Cam == Perspectiva)
		{
			//glLoadMatrixf(m2);
			//			(ang , aspect, near, far)
			gluPerspective(ang, asp, Cerca, Lejos);
		}
		if (Cam == Ortogonal)
		{
			glLoadMatrixf(m3);
			//glOrtho(Izquierda, Derecha, Abajo, Arriba, Cerca, Lejos);

		}
		if (Cam == CargarMatriz)
		{
			glLoadMatrixf(mc);
		}
	}
	//menu Render
	{
		//cubo
		{
			if (opp == 1)//vertice
			{
				CuboVertices();


			}
			if (opp == 2)//aristas
			{
				CuboAristas();

			}
			if (opp == 3)//Caras
			{
				CuboCaras();

			}
			if (opp == 4)//completo
			{
				CuboVertices();
				CuboAristas();
				CuboCaras();

			}
			if (opp == 5)//normales
			{
				CuboCaras();
				CuboNormales();
			}
		}
		//Recurrencia
		{
			if (opp == 6)
			{
				sierpinsky();
			}
			if (opp == 7)
			{
				triangulo();
			}
			if (opp == 8)
			{
				Arbol_Pitagoras();
			}
			if (opp == 13)
			{
				Prisma();
			}
		}
		//Dante el Elefante
		{
			if (opp == 14)//completo
			{
				glPushMatrix();
				glScalef(0.02, 0.02, 0.02);
				Dante();
				glPopMatrix();


			}
			if (opp == 15)//Vertices
			{
				glPushMatrix();
				glScalef(0.02, 0.02, 0.02);
				PuntosDante();
				glPopMatrix();

			}
			if (opp == 16)//Caras
			{
				glPushMatrix();
				glScalef(0.02, 0.02, 0.02);
				CarasDante();
				glPopMatrix();

			}
			if (opp == 17)//Normales
			{
				glPushMatrix();
				glScalef(0.02, 0.02, 0.02);
				NormalesDante();
				CarasDante();
				glPopMatrix();


			}
		
}
		//BEZIER
		{
			if (opp == 18)
			{
				
					CurvaBezier(n);
					if(temporizador==on)
					glutIdleFunc(OnTimerGL);
					else
					glutIdleFunc(NULL);
	
			}
		}
	}
	//Superficie();
	malla();
	ejes();
	//ejemplo();
	glEnd();
	glutPostRedisplay();
	glutSwapBuffers();
}
void reshape(int w, int h)
{
	W = w; H = h;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (float)w / (float)h, near, 300.0);
	glFlush();
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GL_DOUBLEBUFFER | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1000, 800);
	glutInitWindowPosition(300, 100);
	glutCreateWindow(argv[0]);
	init();
	//menu clic derecho
	{
		int camara = glutCreateMenu(menu);//menu ->Camara
		{
			glutAddMenuEntry("LookAt", 9);
			glutAddMenuEntry("Perspectiva", 10);
			glutAddMenuEntry("Ortogonal", 11);
			glutAddMenuEntry("Cargando Matriz", 12);
		}
		int cubo = glutCreateMenu(menu);//menu ->cubo
		{
			glutAddMenuEntry("Vertices", 1);
			glutAddMenuEntry("Aristas", 2);
			glutAddMenuEntry("Caras", 3);
			glutAddMenuEntry("Normales", 5);
			glutAddMenuEntry("Completo", 4);
		}
		int Fractales = glutCreateMenu(menu);//menu ->Fractales
		{
			glutAddMenuEntry("Prisma", 13);
			glutAddMenuEntry("Sierpinsky I.", 6);
			glutAddMenuEntry("Sierpinsky R.", 7);
			glutAddMenuEntry("Arbol", 8);
		}
		int dante = glutCreateMenu(menu);//menu ->Dante
		{
			glutAddMenuEntry("Vertices", 15);
			glutAddMenuEntry("Caras", 16);
			glutAddMenuEntry("Normales", 17);
			glutAddMenuEntry("Completo", 14);
			glutAddMenuEntry("Recorrido", 18);
		}

		//menu principal
		{
			glutCreateMenu(menu);
			glutAddSubMenu("Dante", dante);
			glutAddSubMenu("Cubo", cubo);
			glutAddSubMenu("Fractales", Fractales);
			glutAddSubMenu("Camara", camara);

			glutAddMenuEntry("Salir", 0);
		}
	}
	glutAttachMenu(GLUT_RIGHT_BUTTON);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(Onkeyboard);
	glutMainLoop();
	return 0;
}