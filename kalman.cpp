// bin2asc.cpp : Defines the entry point for the console application.
//


#include <iostream>
#include "gps_type.h"
#include "function.h"
using namespace std;

#define kno1 19
#define kno2 10
#define sno 4
//kalman parameter
double FKOr[3];
double FKDV[3];
double FKDFDL[2];
double FKDh;
double FKDGyro[3];
double FKDAcc[3];
double FK_DCf[5];

double F[kno1*kno1] = {0};
double Fkper[kno1*kno1] = {0};
double H_const[kno2*kno1] = {0};
double Hsu_disk[kno2*kno1] = {0};
double R13[kno2*kno2] = {0};
double Pk[kno1*kno1] = {0};
double Kk1[kno1*kno2];
double xk[kno1] = {0};
double Zdisk[kno2] = {0};
double Ekno1[kno1*kno1] = {0};
double Q[kno1*kno1] = {0};
//xqm
void setagh(void);
void init_kf(void);
void setH(void);
void do_kalman(void);
void estim(void) ;
void get_measure(void);
void kalman_filter();

void init_kf(void)
{
	int k, k2;
	//clear
	for (k=0; k<kno1; k++)
		for (k2=0; k2<kno1; k2++) {
			Fkper[k*kno1+k2] = 0;
			Q[k*kno1+k2] = 0;
			Pk[k*kno1+k2] = 0;
			Ekno1[k*kno1+k2] = 0;
		}
	for (k=0; k<kno2; k++)
		for (k2=0; k2<kno2; k2++) {
			R13[k*kno2+k2] = 0;
		}
	for (k=0; k<kno2; k++)
		for (k2=0; k2<kno1; k2++) {
			H_const[k*kno2+k2] = 0;
		}

	for (k=0; k<3; k++)
	{
		FKOr[k] = 0;
		FKDV[k] = 0;
		FKDGyro[k] = 0;
		FKDAcc[k] = 0;
	}
	FKDFDL[0] = 0;
	FKDFDL[1] = 0;
	FKDh = 0;
	for (k=0; k<5; k++)
		FK_DCf[k] = 0;
	for (k=0; k<kno1; k++)
		xk[k] = 0;

	//initialize Fkper
	for (k=0; k<kno1; k++) {
		Fkper[k*kno1+k] = 1;
	}

	///set Q////
	Q[0*kno1+0] = pow( SigmaDrH, 2) * dT/Tz;
	Q[1*kno1+1] = pow( SigmaDrE, 2) * dT/Tz;
	Q[2*kno1+2] = pow( SigmaDrN, 2) * dT/Tz;
	Q[3*kno1+3] = pow( SigmaAksE, 2) * dT/Tz;
	Q[4*kno1+4] = pow( SigmaAksN, 2) * dT/Tz;
	Q[5*kno1+5] = pow( SigmaAksH, 2) * dT/Tz;
	Q[6*kno1+6] = pow( SigmaDrFi, 2) * dT/Tz;
	Q[7*kno1+7] = pow( SigmaDrLam, 2) * dT/Tz;
	Q[8*kno1+8] = pow( SigmaDrh, 2) * dT/Tz;
	Q[9*kno1+9] = pow( SigmadCDrXb, 2) * dT/Tz;
	Q[10*kno1+10] = pow( SigmadCDrYb, 2) * dT/Tz;
	Q[11*kno1+11] = pow( SigmadCDrZb, 2) * dT/Tz;
	Q[12*kno1+12] = pow( SigmadCAksXb, 2) * dT/Tz;
	Q[13*kno1+13] = pow( SigmadCAksYb, 2) * dT/Tz;
	Q[14*kno1+14] = pow( SigmadCAksZb, 2) * dT/Tz;

/*	double sigmaDCpo = SigmadDC1po;
	for (k=1; k<5; k++)
	{
		if (fabs(single_diff[k])>50)
			sigmaDCpo*=3;
		if (fabs(single_diff[k])>100)
			sigmaDCpo*=3;
		if (fabs(single_diff[k])>150)
			sigmaDCpo*=3;
		int l = 14+k;
		Q[l*kno1+l] = pow( sigmaDCpo, 2);
	}
*/
	Q[15*kno1+15] = pow( SigmadDC1po, 2) * dT/Tz;
	Q[16*kno1+16] = pow( SigmadDC2po, 2) * dT/Tz;
	Q[17*kno1+17] = pow( SigmadDC3po, 2) * dT/Tz;
	Q[18*kno1+18] = pow( SigmadDC4po, 2) * dT/Tz;

	//set Pk//
	Pk[0*kno1+0] = pow( SigmaAlphao, 2);
	Pk[1*kno1+1] = pow( SigmaBetao, 2);
	Pk[2*kno1+2] = pow( SigmaGammao, 2);
	Pk[3*kno1+3] = pow( SigmaDVEo, 2);
	Pk[4*kno1+4] = pow( SigmaDVNo, 2);
	Pk[5*kno1+5] = pow( SigmaDVHo, 2);
	Pk[6*kno1+6] = pow( SigmaDFio, 2);
	Pk[7*kno1+7] = pow( SigmaDLamo, 2);
	Pk[8*kno1+8] = pow( SigmaDho, 2);
	Pk[9*kno1+9] = pow( SigmaCDXbo, 2);
	Pk[10*kno1+10] = pow( SigmaCDYbo, 2);
	Pk[11*kno1+11] = pow( SigmaCDZbo, 2);
	Pk[12*kno1+12] = pow( SigmaCAksXbo, 2);
	Pk[13*kno1+13] = pow( SigmaCAksYbo, 2);
	Pk[14*kno1+14] = pow( SigmaCAksZbo, 2);

/*
	double sigmaDC = SigmaDC1po;
	for (k=1; k<5; k++)
	{
		if (fabs(single_diff[k])>50)
			sigmaDC*=3;
		if (fabs(single_diff[k])>100)
			sigmaDC*=3;
		if (fabs(single_diff[k])>150)
			sigmaDC*=3;
		if (fabs(single_diff[k])>400)
			sigmaDC*=3;
		int l = 14+k;
		Pk[l*kno1+l] = pow( sigmaDC, 2);
	}
*/
	double sigmaDC = SigmaDC1po;
	for (k=1; k<5; k++)
	{
		if (fabs(single_diff[k])>20)
			sigmaDC*=3;
		if (fabs(single_diff[k])>50)
			sigmaDC*=3;
		if (fabs(single_diff[k])>200)
			sigmaDC*=3;
		if (fabs(single_diff[k])>400)
			sigmaDC*=3;
		int l = 14+k;
		Pk[l*kno1+l] = pow( sigmaDC, 2);
	}

//	Pk[15*kno1+15] = pow( SigmaDC1po, 2);
//	Pk[16*kno1+16] = pow( SigmaDC2po, 2);
//	Pk[17*kno1+17] = pow( SigmaDC3po, 2);
//	Pk[18*kno1+18] = pow( SigmaDC4po, 2);

	//set H_const//
	for (k=0; k<6; k++)
		H_const[k*kno1+k+3] = 1;

	//set R13//

	R13[0*kno2+0] = pow( SigmaDVEGPS, 2);
	R13[1*kno2+1] = pow( SigmaDVNGPS, 2);
	R13[2*kno2+2] = pow( SigmaDVHGPS, 2);
	R13[3*kno2+3] = pow( SigmaDFiGPS, 2);
	R13[4*kno2+4] = pow( SigmaDLamGPS, 2);
	R13[5*kno2+5] = pow( SigmaDh, 2);

	for (k=6; k<kno2; k++)
		for (k2=6; k2<kno2; k2++)
			R13[k*kno2+k2] = 1 * pow( SigmaZS1_i, 2);
	for (k=6; k<kno2; k++)
		R13[k*kno2+k] = 2 * pow( SigmaZS1_i, 2);

	//set Ekno1//
	for (k=0; k<kno1; k++)
		Ekno1[k*kno1+k] = 1;

}


void setH(void)
{
	int k, k2;
	//clear
	for (k=0; k<kno1; k++)
		for (k2=0; k2<kno2; k2++) {
			Hsu_disk[k*kno2+k2] = 0;
		}

	double Soph[3],S1h[3],S2h[3],S3h[3], S4h[3], S5h[3];

	double dHS1p[kno2*kno1] = {0};
	double dHS2p[kno2*kno1] = {0};
	double dHS3p[kno2*kno1] = {0};
	double dHS4p[kno2*kno1] = {0};
	double dHS5p[kno2*kno1] = {0};

	double B[3] = {0};

	for (k=0; k<2; k++) //k=0; k<3; k++

	{
		Soph[k] = sat_vec_h[0][k];
		S1h[k] =  sat_vec_h[1][k];
		S2h[k] =  sat_vec_h[2][k];
		S3h[k] =  sat_vec_h[3][k];
		S4h[k] =  sat_vec_h[4][k];
		S5h[k] =  sat_vec_h[5][k];
	}


	matrix( B, Cbh[0], b1ort, 3, 3, 1);

		dHS1p[6*kno1+0] = (B[1]*(Soph[0]-S1h[0]) - B[0]*(Soph[1]-S1h[1]));
		dHS1p[6*kno1+1] = (B[2]*(Soph[1]-S1h[1]) - B[1]*(Soph[2]-S1h[2]));
		dHS1p[6*kno1+2] = (B[0]*(Soph[2]-S1h[2]) - B[2]*(Soph[0]-S1h[0]));
		dHS1p[6*kno1+15] = 1;
		matrix_a( Hsu_disk, H_const, dHS1p, kno2, kno1, 1);


		dHS2p[7*kno1+0] = (B[1]*(Soph[0]-S2h[0]) - B[0]*(Soph[1]-S2h[1])); 
		dHS2p[7*kno1+1] = (B[2]*(Soph[1]-S2h[1]) - B[1]*(Soph[2]-S2h[2]));
		dHS2p[7*kno1+2] = (B[0]*(Soph[2]-S2h[2]) - B[2]*(Soph[0]-S2h[0]));
		dHS2p[7*kno1+16] = 1;
		matrix_a( Hsu_disk, Hsu_disk, dHS2p, kno2, kno1, 1);

		dHS3p[8*kno1+0] = (B[1]*(Soph[0]-S3h[0]) - B[0]*(Soph[1]-S3h[1])); 
		dHS3p[8*kno1+1] = (B[2]*(Soph[1]-S3h[1]) - B[1]*(Soph[2]-S3h[2]));
		dHS3p[8*kno1+2] = (B[0]*(Soph[2]-S3h[2]) - B[2]*(Soph[0]-S3h[0]));
		dHS3p[8*kno1+17] = 1;
		matrix_a( Hsu_disk, Hsu_disk, dHS3p, kno2, kno1, 1);

		dHS4p[9*kno1+0] = (B[1]*(Soph[0]-S4h[0]) - B[0]*(Soph[1]-S4h[1])); 
		dHS4p[9*kno1+1] = (B[2]*(Soph[1]-S4h[1]) - B[1]*(Soph[2]-S4h[2]));
		dHS4p[9*kno1+2] = (B[0]*(Soph[2]-S4h[2]) - B[2]*(Soph[0]-S4h[0]));
		dHS4p[9*kno1+18] = 1;
		matrix_a( Hsu_disk, Hsu_disk, dHS4p, kno2, kno1, 1);

	/*dHS5p[10*kno1+0] = (B[1]*(Soph[0]-S5h[0]) - B[0]*(Soph[1]-S5h[1])); 
	dHS5p[10*kno1+1] = (B[2]*(Soph[1]-S5h[1]) - B[1]*(Soph[2]-S5h[2]));
	dHS5p[10*kno1+2] = (B[0]*(Soph[2]-S5h[2]) - B[2]*(Soph[0]-S5h[0]));
	dHS5p[10*kno1+19] = 1;
*/

	//matrix_a( Hsu_disk, Hsu_disk, dHS5p, kno2, kno1, 1);
}

void do_kalman(void)
{
	double Gk[kno1*kno1];
	double gkq[kno1*kno1], gkqgkt[kno1*kno1];
	double fkpk[kno1*kno1], fkpkfkt[kno1*kno1];
	double sum5[kno1*kno1], Pkkt[kno1*kno1];
	double hpkk[kno2*kno1], hpkkht[kno2*kno2];
	double sum4[kno2*kno2];
	double tempk[kno1*kno2];
	double Pkk[kno1*kno1];

	double kk1hk1[kno1*kno1], ekk1hk1[kno1*kno1];
	double ekhp[kno1*kno1], sum1[kno1*kno1];
	double kr[kno1*kno2], krkt[kno1*kno1];

	/* P(k+1,k) = Gk*Q*GkT + Fk*P(k)*FkT */
	matrix_m( Gk, Fkper, Tz, kno1, kno1);
	matrix( gkq, Gk, Q, kno1, kno1, kno1);
	matrixT( gkqgkt, gkq, Gk, kno1, kno1, kno1);
	matrix( fkpk, Fkper, Pk, kno1, kno1, kno1);
	matrixT( fkpkfkt, fkpk, Fkper, kno1, kno1, kno1);
	matrix_a( Pkk, gkqgkt, fkpkfkt, kno1, kno1, 1);

	/* K(k+1) = P(k+1,k)*HT/[H*P(k+1,k)*HT+R]*/
	matrix_trans( Pkkt, Pkk, kno1, kno1);
	matrix_a( sum5, Pkkt, Pkk, kno1, kno1, 1);
	matrix_m( sum5, sum5, 0.5, kno1, kno1);
	matrix( hpkk, Hsu_disk, sum5, kno2, kno1, kno1);
	matrixT( hpkkht, hpkk, Hsu_disk, kno2, kno1, kno2);
	matrix_a( sum4, hpkkht, R13, kno2, kno2, 1);
	brinv( sum4, kno2);
	matrixTa( tempk, Hsu_disk, sum4, kno1, kno2, kno2);
	matrix( Kk1, sum5, tempk, kno1, kno1, kno2);

	/* P(k+1) = (I-K(k)H) P(k) (I-K(k) H)' + K(k) R K(k) */
	matrix( kk1hk1, Kk1, Hsu_disk, kno1, kno2, kno1);
	matrix_a( ekk1hk1, Ekno1, kk1hk1, kno1, kno1, 0);
	matrix( ekhp, ekk1hk1, sum5, kno1, kno1, kno1);
	matrixT( sum1, ekhp, ekk1hk1, kno1, kno1, kno1);
	matrix( kr, Kk1, R13, kno1, kno2, kno2);
	matrixT( krkt, kr, Kk1, kno1, kno2, kno1);
	matrix_a( Pk, sum1, krkt, kno1, kno1, 1);
}

void estim(void) 
{
	int k;
	double kk1z[kno1];
	double xk1k[kno1];
	double xk1[kno1];

	matrix( kk1z, Kk1, Zdisk, kno1, kno2, 1); 

	matrix( xk1k, Fkper, xk, kno1, kno1, 1);
	for (k=0; k<9; k++)
		xk1k[k] = 0;
	
	matrix_a( xk1, kk1z, xk1k, kno1, 1, 1);
	for (k=0; k<kno1; k++)
		xk[k] = xk1[k];
//角速度 速度 位置 陀螺 加表 卫星
	for (k=0; k<3; k++)
		FKOr[k] = xk[k];
	for (k=0; k<3; k++)
		FKDV[k] = xk[k+3];
	for (k=0; k<2; k++)
		FKDFDL[k] = xk[k+6];
	FKDh = xk[8];
	for (k=0; k<3; k++)
		FKDGyro[k] = -xk[k+9];
	for (k=0; k<3; k++)
		FKDAcc[k] = xk[k+12];
	for (k=0; k<sno; k++)//	for (k=0; k<2; k++)
		FK_DCf[k] = xk[k+15];

}

void get_measure(void)
{
	int i;
	Zdisk[0] = Vh[0] - Vh_gps[0];
	Zdisk[1] = Vh[1] - Vh_gps[1];
	Zdisk[2] = Vh[2] - Vh_gps[2];
	Zdisk[3] = FiLa[0] - FiLa_gps[0];
	Zdisk[4] = FiLa[1] - FiLa_gps[1];
	Zdisk[5] = h - h_gps;
	for (i=0; i<sno; i++)
		Zdisk[6+i] = sat_vec_diff_b1[i]-std_double_diff[i]- FK_DCf[i];

	//20180315选星后添加
	for (i=1; i<sno+1; i++)
	{
		if (cur_sat[i]==0)
		{
			Zdisk[6+i-1] = 0;
		}
	}

}

void kalman_filter()
{
	setagh();
	if (gpsflg == 41) {

		get_measure();
		setH();
		do_kalman();
		estim();

	}
}
//c[n*r]=a[n*m]*b[m*r]
void matrix( double c[], double a[], double b[], int n, int m, int r)
{
	int i,j,k;
	double y;
	for (i=0; i<n; i++) {
		for (j=0; j<r; j++) {
			y = 0;
			for (k=0; k<m; k++) 
				y += a[i*m+k]*b[k*r+j];
			c[i*r+j] = y;
		}
	}
}

/*c[m*n]=a+b     ktrl>0:+; else:-. */
void matrix_a( double c[], double a[], double b[], int m, int n, int ktrl)
{
	int k, k2;
	for (k=0; k<m; k++)
		for (k2=0; k2<n; k2++)
			c[k*n+k2] = a[k*n+k2] + (ktrl>0 ? b[k*n+k2] : -b[k*n+k2]);
		return;
}

/* b[n*m] = a[m*n]' */
void matrix_trans( double b[], double a[], int m, int n)
{
	int k, k2;
	for (k=0; k<n; k++)
		for (k2=0; k2<m; k2++)
			b[k*m+k2] = a[k2*n+k];
}

/* c[n*r] = a[n*m]*Tran(b[r*m]) */
void matrixT( double c[], double a[], double b[], int n, int m, int r)
{
	int i,j,k;
	double y;
	for (i=0; i<n; i++)
		for (j=0; j<r; j++) {
			y=0.0;
			for (k=0; k<m; k++)
				y += a[i*m+k] * b[j*m+k];
			c[i*r+j] = y;
		}
}

/* b[m*n] = t*a[m*n] */
void matrix_m( double b[], double a[], double t, int m, int n) 
{
	int i, j;
	for (i=0; i<m; i++)
		for (j=0; j<n; j++)
			b[i*n+j] = t*a[i*n+j];
}

/**/
void brinv(double a[], int n)
{
	int i,j,k,l,u,v;
	double d,p;
	int is[10], js[10];
	
	for (k=0; k<=n-1; k++) {
		d=0.0;
		for (i=k; i<=n-1; i++)
			for (j=k; j<=n-1; j++) {
				l=i*n+j;
				p=fabs(a[l]);
				if (p>d) {
					d=p; is[k]=i; js[k]=j;
				}
			}
			if (d+1.0 == 1.0) {
				return;
			}
			if (is[k] != k) 
				for (j=0; j<=n-1; j++) {
					u=k*n+j; v=is[k]*n+j;
					p=a[u]; a[u] = a[v]; a[v]=p;
				}
				if (js[k] != k)
					for (i=0; i<=n-1; i++) {
						u=i*n+k; v=i*n+js[k];
						p=a[u]; a[u]=a[v]; a[v]=p;
					}
					l=k*n+k;
					a[l]=1.0/a[l];
					for (j=0; j<=n-1; j++)
						if (j!=k) {
							u=k*n+j; a[u]=a[u]*a[l];
						}
						for (i=0; i<=n-1; i++)
							if (i!=k)
								for (j=0; j<=n-1; j++)
									if (j!=k) {
										u=i*n+j;
										a[u] = a[u]-a[i*n+k]*a[k*n+j];
									}
									for (i=0; i<=n-1; i++)
										if (i!=k) {
											u=i*n+k; a[u] = -a[u]*a[l];
										}
	}
	for (k=n-1; k>=0; k--) {
		if (js[k]!=k)
			for (j=0; j<=n-1; j++) {
				u=k*n+j; v=js[k]*n+j;
				p=a[u]; a[u]=a[v]; a[v]=p;
			}
			if (is[k]!=k)
				for (i=0; i<=n-1; i++) {
					u=i*n+k; v=i*n+is[k];
					p=a[u]; a[u]=a[v]; a[v]=p;
				}
	}
}

/* c[n*r] = Trans{a[m*n]}*b[m*r] */
void matrixTa( double c[], double a[], double b[], int n, int m, int r)
{
	int i, j, k;
	double y;
	for (i=0; i<n; i++)
		for (j=0; j<r; j++) {
			y = 0;
			for (k=0; k<m; k++)
				y += a[k*n+i] * b[k*r+j];
			c[i*r+j] = y;
		}
}

