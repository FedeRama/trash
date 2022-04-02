#include <iostream>
#include <math.h>
#include <chrono>
#include <numeric>
#include "../include/frenet_optimal_trajectory.hpp"
#include "../include/cubic_spline_planner.hpp"

/*
TO-DO

GENERALE:
1) Valutare la terza funzione
2) Abusare del fatto che il nodo ros usa solamente x,y del path finale per usare due vettori grandi ed uno piccolo per fare tutti i calcoli
3) Valutare l'uso di memoria condivisa malloc managed

PRIMA FUNZIONE:
1) portare jp e js a double per sistemare gli index 48 e 60 con jp sbaglati
2) rimuovere il for e portare i thread utilizzati a 80
3) Valutare se portare i thread utilizzati a 80*15
4) Valutare se valga la pena di separare i kernel solo per il calcolo matriciale creando 9 vettori da 80 element
5) Sistemare l'ordine della memoria shared e usarla per le matrici
-) Rimuovere il casting a int dei pointer (già fatto, vedere solo se non ha creato problemi)

SECONDA FUNZIONE:
1) Valutare se portare i thread utilizzati a 80*15
2) Valutare se è possibile usare la shared memory per le spline
*/

__global__
void calcFrenetPathsKernel(double *fpD, double *fpDd, double *fpDdd, double *fpS, double *fpSd, double *fpCf, int forOne, int forTwo, int forThree, int forFour, double c_speed, double c_d, double c_d_d, double c_d_dd, double s0, double D_ROAD_W, double DT, double MAXT, double MINT, double TARGET_SPEED, double D_T_S, double N_S_SAMPLE, double MAX_ROAD_WIDTH_LEFT, double MAX_ROAD_WIDTH_RIGHT, double KJ, double KT, double KD, double KLAT, double KLON)
{
    int index = blockIdx.x*blockDim.x+threadIdx.x;
    

    //printf("%d - %d - %d\n",blockIdx.x,blockDim.x,threadIdx.x);

    __shared__ double param[23];
    param[0] = 0;
    param[1] = 0;
    param[2] = 0;
    param[3] = 0;
    param[4] = c_speed;
    param[5] = c_d;
    param[6] = c_d_d;
    param[7] = c_d_dd / 2.0;
    param[8] = s0;
    param[9] = D_ROAD_W;
    param[10] = DT;
    param[11] = MAXT;
    param[12] = MINT;
    param[13] = TARGET_SPEED;
    param[14] = D_T_S;
    param[15] = N_S_SAMPLE;
    param[16] = MAX_ROAD_WIDTH_LEFT;
    param[17] = MAX_ROAD_WIDTH_RIGHT;
    param[18] = KJ;
    param[19] = KT;
    param[20] = KD;
    param[21] = KLAT;  
    param[22] = KLON; 

    __shared__ int for_size[23];
    for_size[0] = forOne;
    for_size[1] = forTwo;
    for_size[2] = forThree;
    for_size[3] = forFour;
     
        
    __syncthreads();

    if (index < for_size[0]){

        double di = -param[16] + param[9] * index;

        int k = -1;
        for(double Ti = param[12]; Ti <= param[11] + param[10]; Ti += param[10]){
        
            k++;

            MatrixXd A1(3, 3);
            MatrixXd rA1(3, 3);
            MatrixXd B1(3, 1);
            MatrixXd X1(3, 1);

            A1 << pow(Ti, 3), pow(Ti, 4), pow(Ti, 5),
                3*pow(Ti, 2), 4*pow(Ti, 3), 5*pow(Ti, 4),
                6*Ti, 12*Ti*Ti, 20*pow(Ti, 3);

            B1 << di - param[5] - param[6]*Ti - param[7]*Ti*Ti, 
                0 - param[6] - 2*param[7]*Ti,
                0 - 2*param[7];

            double determinant = 0;
            for(int i = 0; i < 3; i++)
                determinant = determinant + (A1(0,i) * (A1(1,(i+1)%3) * A1(2,(i+2)%3) - A1(1,(i+2)%3) * A1(2,(i+1)%3)));

            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 3; j++)
                rA1(i,j) = ((A1((j+1)%3,(i+1)%3) * A1((j+2)%3,(i+2)%3)) - (A1((j+1)%3,(i+2)%3) * A1((j+2)%3,(i+1)%3)))/ determinant;
            }

            X1 = rA1*B1;

            double a3 = X1(0, 0);
            double a4 = X1(1, 0);
            double a5 = X1(2, 0);

            double fpDddd;
            int Jp = 0;
            
            for(int i = 0; i < for_size[2]; i++){
                
                double t = (( Ti  / (for_size[2] - 1)) * i);

                int ptr = (index*for_size[2]*for_size[1] + k*for_size[2] + i);

                fpS[ptr] = t;
                fpD[ptr] = param[5] + param[6]*t + param[7]*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
                fpDd[ptr] = param[6] + 2*param[7]*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
                fpDdd[ptr] = 2*param[7] + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;

                fpDddd = 6*a3 + 24*a4*t + 60*a5*t*t;

                Jp = Jp + (fpDddd * fpDddd);
            }

            //double minV = param[9] - param[10]*param[11];
            //double maxV = param[9] + param[10]*param[11];

            int tv=3;
            
            MatrixXd A2(2, 2);
            MatrixXd rA2(2, 2);
            MatrixXd B2(2, 1);
            MatrixXd X2(2, 1);

            A2 << 3*pow(Ti, 2), 4*pow(Ti, 3),
                6*Ti, 12*Ti*Ti ;

            B2 << tv - param[4] - 2*0*Ti,
                0 - 2*0;

            determinant = 1/(A2(0,0)*A2(1,1)-A2(0,1)*A2(1,0));
            rA2 << determinant * A2(1,1), -determinant * A2(0,1),
                -determinant * A2(1,0), determinant * A2(0,0);

            X2 = rA2*B2;

            a3 = X2(0, 0);
            a4 = X2(1, 0);

            double fpSddd;
            int Js = 0;
            
            for(int i=0;i<for_size[2];i++){
                
                int ptr = (index*for_size[2]*for_size[1] + k*for_size[2] + i);

                double t = fpS[ptr];
                
                fpS[ptr] = param[8] + param[4]*t + 0*t*t + a3*t*t*t + a4*t*t*t*t;
                fpSd[ptr] = param[4] + 2*0*t + 3*a3*t*t + 4*a4*t*t*t;

                fpSddd = 6*a3 + 24*a4*t;

                Js = Js + (fpSddd * fpSddd);
            }

            int ptr = (index*for_size[2]*for_size[1] + (k+1)*for_size[2] -1);

            double ds = pow((param[13] - fpSd[ptr]), 2);
            double fpCd = param[18]*Jp + param[19]*Ti + param[20]*fpD[ptr]*fpD[ptr]; //lateral generation cost
            double fpCv = param[18]*Js + param[19]*Ti + param[20]*ds; //longitudinal generation cost
            fpCf[index*for_size[1] +k] = param[21]*fpCd + param[22]*fpCv;

        }
    }
}


__global__
void calcGlobalPathsKernel(bool *fpBlackList, double *fpS, double *fpD, double *fpX, double *fpY, int fpListSize, int fpListParamSize, int splineParamSize, double lastS, double *splineXA, double *splineXB, double *splineXC, double *splineXD, double *splineXX, int splineXNx, double *splineYA, double *splineYB, double *splineYC, double *splineYD, double *splineYX, int splineYNx)
{
    int index = blockIdx.x*blockDim.x+threadIdx.x;
    
    if(index<fpListSize){
  
        for(int i=0;i<fpListParamSize;i++){
        
            double t = fpS[fpListParamSize*index + i];
            double d = fpD[fpListParamSize*index + i];

            if (t <= 0)
                t = 0;
            while (t >= lastS)
                t -= lastS;

            if(t < splineXX[0]){
                fpBlackList[index] = true;
                return;
            }
            if(t > splineXX[splineXNx - 1]){
                fpBlackList[index] = true;
                return;
            }
                
            int indX;
            for(int j=0; j<splineParamSize; j++){
                if(splineXX[j] > t){
                indX=j-1;
                break;
                }
            }

            double dxCalc = t - splineXX[indX];
            double ix= splineXA[indX] + splineXB[indX]*dxCalc + splineXC[indX] *dxCalc *dxCalc + splineXD[indX] *dxCalc *dxCalc *dxCalc;

            if(t < splineYX[0]){
                fpBlackList[index] = true;
                return;
            }
            if(t > splineYX[splineYNx - 1]){
                fpBlackList[index] = true;
                return;
            }
                
            int indY;
            for(int j=0; j<splineParamSize; j++){
                if(splineYX[j] > t){
                indY=j-1;
                break;
                }
            }

            double dyCalc = t - splineYX[indY];
            double iy= splineYA[indY] + splineYB[indY]*dyCalc + splineYC[indY] *dyCalc *dyCalc + splineYD[indY] *dyCalc *dyCalc *dyCalc;

            double dx = splineXB[indX] + 2 * splineXC[indX] * dxCalc + 3 * splineXD[indX] * dxCalc * dxCalc;
            double dy = splineYB[indY] + 2 * splineYC[indY] * dyCalc + 3 * splineYD[indY] * dyCalc * dyCalc;

            double iyaw;

            if(dx == 0)
                iyaw = 1.57*(dy > 0);
            else
                iyaw = atan2(dy, dx);

            double fx = ix + d*cos(iyaw + M_PI/2.0);
	        double fy = iy + d*sin(iyaw + M_PI/2.0);

            fpX[fpListParamSize*index + i]=fx;
            fpY[fpListParamSize*index + i]=fy;  
        }
    }
}



vector<FrenetPath> cuda_test(vector<FrenetPath> fplist, Spline2D csp, double c_speed, double c_d, double c_d_d, double c_d_dd, double s0, double const_third_for, double D_ROAD_W, double DT, double MAXT, double MINT, double TARGET_SPEED, double D_T_S, double N_S_SAMPLE, double MAX_ROAD_WIDTH_LEFT, double MAX_ROAD_WIDTH_RIGHT, double KJ, double KT, double KD, double KLAT, double KLON){

//auto timeb = std::chrono::high_resolution_clock::now();

double temp1 = ((MAX_ROAD_WIDTH_RIGHT - -MAX_ROAD_WIDTH_LEFT) / D_ROAD_W);
double temp2  = 2;
double temp3 = const_third_for;
double temp4 = 1;

int forOne = int(temp1);
int forTwo = int(temp2);
int forThree = int(temp3);
int forFour = int(temp4);

int fpListSize = forOne * forTwo * forFour;
int fpListParamSize  = const_third_for;
int splineParamSize = csp.sx.a.size();

/*
std::cout << "1: " <<temp1<<" 2: "<<temp2<< " 3: "<<temp3<<" 4: "<<temp4 << std::endl;
std::cout << "1: " <<forOne<<" 2: "<<forTwo<< " 3: "<<forThree<<" 4: "<<forFour << std::endl;
std::cout << "fpListSize: " <<fpListSize<<" fpListParamSize: "<<fpListParamSize<< " splineParamSize: "<<splineParamSize << std::endl;
std::cout << "fpListSize: " <<fplist.size()<<" fpListParamSize: "<<fplist[0].s.size()<< " splineParamSize: "<<splineParamSize << std::endl;
*/

double *fpD, *h_fpD;
double *fpDd, *h_fpDd;
double *fpDdd, *h_fpDdd;
double *fpS, *h_fpS;
double *fpSd, *h_fpSd;
double *fpX, *h_fpX;
double *fpY, *h_fpY;
double *fpCf, *h_fpCf;

double lastS = csp.get_s_last();
double *splineXA, *h_splineXA;
double *splineXB, *h_splineXB;
double *splineXC, *h_splineXC;
double *splineXD, *h_splineXD;
double *splineXX, *h_splineXX;
int splineXNx = csp.sx.nx;
double *splineYA, *h_splineYA;
double *splineYB, *h_splineYB;
double *splineYC, *h_splineYC;
double *splineYD, *h_splineYD;
double *splineYX, *h_splineYX;
int splineYNx = csp.sy.nx;



cudaMalloc((void**)&fpD, fpListSize*fpListParamSize*sizeof(double));
cudaMalloc((void**)&fpDd, fpListSize*fpListParamSize*sizeof(double));
cudaMalloc((void**)&fpDdd, fpListSize*fpListParamSize*sizeof(double));
cudaMalloc((void**)&fpS, fpListSize*fpListParamSize*sizeof(double));
cudaMalloc((void**)&fpSd, fpListSize*fpListParamSize*sizeof(double));
cudaMalloc((void**)&fpX, fpListSize*fpListParamSize*sizeof(double));
cudaMalloc((void**)&fpY, fpListSize*fpListParamSize*sizeof(double));
cudaMalloc((void**)&fpCf, fpListSize*sizeof(double));


cudaMalloc((void**)&splineXA, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineXB, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineXC, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineXD, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineXX, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineYA, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineYB, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineYC, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineYD, splineParamSize*sizeof(double));
cudaMalloc((void**)&splineYX, splineParamSize*sizeof(double));



h_fpD = (double*)malloc(fpListSize*fpListParamSize*sizeof(double));
h_fpDd = (double*)malloc(fpListSize*fpListParamSize*sizeof(double));
h_fpDdd = (double*)malloc(fpListSize*fpListParamSize*sizeof(double));
h_fpS = (double*)malloc(fpListSize*fpListParamSize*sizeof(double));
h_fpSd = (double*)malloc(fpListSize*fpListParamSize*sizeof(double));
h_fpX = (double*)malloc(fpListSize*fpListParamSize*sizeof(double));
h_fpY = (double*)malloc(fpListSize*fpListParamSize*sizeof(double));
h_fpCf = (double*)malloc(fpListSize*sizeof(double));

h_splineXA = (double*)malloc(splineParamSize*sizeof(double));
h_splineXB = (double*)malloc(splineParamSize*sizeof(double));
h_splineXC = (double*)malloc(splineParamSize*sizeof(double));
h_splineXD = (double*)malloc(splineParamSize*sizeof(double));
h_splineXX = (double*)malloc(splineParamSize*sizeof(double));
h_splineYA = (double*)malloc(splineParamSize*sizeof(double));
h_splineYB = (double*)malloc(splineParamSize*sizeof(double));
h_splineYC = (double*)malloc(splineParamSize*sizeof(double));
h_splineYD = (double*)malloc(splineParamSize*sizeof(double));
h_splineYX = (double*)malloc(splineParamSize*sizeof(double));



std::copy(csp.sx.a.begin(),csp.sx.a.end(),h_splineXA);
std::copy(csp.sx.b.begin(),csp.sx.b.end(),h_splineXB);
std::copy(csp.sx.c.begin(),csp.sx.c.end(),h_splineXC);
std::copy(csp.sx.d.begin(),csp.sx.d.end(),h_splineXD);
std::copy(csp.sx.x.begin(),csp.sx.x.end(),h_splineXX);
std::copy(csp.sy.a.begin(),csp.sy.a.end(),h_splineYA);
std::copy(csp.sy.b.begin(),csp.sy.b.end(),h_splineYB);
std::copy(csp.sy.c.begin(),csp.sy.c.end(),h_splineYC);
std::copy(csp.sy.d.begin(),csp.sy.d.end(),h_splineYD);
std::copy(csp.sy.x.begin(),csp.sy.x.end(),h_splineYX);

cudaMemcpy(splineXA, h_splineXA, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineXB, h_splineXB, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineXC, h_splineXC, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineXD, h_splineXD, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineXX, h_splineXX, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineYA, h_splineYA, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineYB, h_splineYB, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineYC, h_splineYC, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineYD, h_splineYD, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);
cudaMemcpy(splineYX, h_splineYX, splineParamSize*sizeof(double), cudaMemcpyHostToDevice);




calcFrenetPathsKernel<<<1, 40>>>(fpD, fpDd, fpDdd, fpS, fpSd, fpCf, forOne, forTwo, forThree, forFour, c_speed, c_d, c_d_d, c_d_dd, s0, D_ROAD_W, DT, MAXT, MINT, TARGET_SPEED, D_T_S, N_S_SAMPLE, MAX_ROAD_WIDTH_LEFT, MAX_ROAD_WIDTH_RIGHT, KJ, KT, KD, KLAT, KLON);

cudaDeviceSynchronize();

cudaError_t error = cudaGetLastError();
if (error != cudaSuccess) {
    fprintf(stderr, "ERROR KERNEL 1: %s \n", cudaGetErrorString(error));
}



bool *fpBlackList;
cudaMallocManaged(&fpBlackList, fpListSize*sizeof(double));

calcGlobalPathsKernel<<<1, 80>>>(fpBlackList, fpS, fpD, fpX, fpY, fpListSize, fpListParamSize, splineParamSize, lastS, splineXA, splineXB, splineXC, splineXD, splineXX, splineXNx, splineYA, splineYB, splineYC, splineYD, splineYX, splineYNx);

cudaDeviceSynchronize();

error = cudaGetLastError();
if (error != cudaSuccess) {
    fprintf(stderr, "ERROR KERNEL 2: %s \n", cudaGetErrorString(error));
}

int fpDiscarded = std::accumulate(&fpBlackList[0],&fpBlackList[fpListSize],0);

//std::chrono::duration<double> lastb= std::chrono::duration_cast<std::chrono::duration<double>>((std::chrono::high_resolution_clock::now()) - timeb);
//std::cout << "-------------------------> TEMPO UNICA - PARTE 2 -> " << lastb.count()*1000 << std::endl;



cudaMemcpy(h_fpD, fpD, fpListSize*fpListParamSize*sizeof(double), cudaMemcpyDeviceToHost);
cudaMemcpy(h_fpDd, fpDd, fpListSize*fpListParamSize*sizeof(double), cudaMemcpyDeviceToHost);
cudaMemcpy(h_fpDdd, fpDdd, fpListSize*fpListParamSize*sizeof(double), cudaMemcpyDeviceToHost);
cudaMemcpy(h_fpS, fpS, fpListSize*fpListParamSize*sizeof(double), cudaMemcpyDeviceToHost);
cudaMemcpy(h_fpSd, fpSd, fpListSize*fpListParamSize*sizeof(double), cudaMemcpyDeviceToHost);
cudaMemcpy(h_fpX, fpX, fpListSize*fpListParamSize*sizeof(double), cudaMemcpyDeviceToHost);
cudaMemcpy(h_fpY, fpY, fpListSize*fpListParamSize*sizeof(double), cudaMemcpyDeviceToHost);
cudaMemcpy(h_fpCf, fpCf, fpListSize*sizeof(double), cudaMemcpyDeviceToHost);

std::vector<FrenetPath> new_fplist;

for(int i=0; i<fpListSize;i++){

    FrenetPath fp;

    fp.d.insert(fp.d.begin(), &h_fpD[i*fpListParamSize], &h_fpD[(i+1)*fpListParamSize]);
    fp.d_d.insert(fp.d_d.begin(), &h_fpDd[i*fpListParamSize], &h_fpDd[(i+1)*fpListParamSize]);
    fp.d_dd.insert(fp.d_dd.begin(), &h_fpDdd[i*fpListParamSize], &h_fpDdd[(i+1)*fpListParamSize]);
    fp.s.insert(fp.s.begin(), &h_fpS[i*fpListParamSize], &h_fpS[(i+1)*fpListParamSize]);
    fp.s_d.insert(fp.s_d.begin(), &h_fpSd[i*fpListParamSize], &h_fpSd[(i+1)*fpListParamSize]);
    fp.x.insert(fp.x.begin(), &h_fpX[i*fpListParamSize], &h_fpX[(i+1)*fpListParamSize]);
    fp.y.insert(fp.y.begin(), &h_fpY[i*fpListParamSize], &h_fpY[(i+1)*fpListParamSize]);
    fp.cf = h_fpCf[i];
    
    new_fplist.push_back(fp);
}  



cudaFree(fpD);
cudaFree(fpDd);
cudaFree(fpDdd);
cudaFree(fpS);
cudaFree(fpSd);
cudaFree(fpCf);
cudaFree(fpX);
cudaFree(fpY);
cudaFree(splineXA);
cudaFree(splineXB);
cudaFree(splineXC);
cudaFree(splineXD);
cudaFree(splineXX);
cudaFree(splineYA);
cudaFree(splineYB);
cudaFree(splineYC);
cudaFree(splineYD);
cudaFree(splineYX);

free(h_fpD);
free(h_fpDd);
free(h_fpDdd);
free(h_fpS);
free(h_fpSd);
free(h_fpX);
free(h_fpY);
free(h_fpCf);
free(h_splineXA);
free(h_splineXB);
free(h_splineXC);
free(h_splineXD);
free(h_splineXX);
free(h_splineYA);
free(h_splineYB);
free(h_splineYC);
free(h_splineYD);
free(h_splineYX);

return new_fplist;
}