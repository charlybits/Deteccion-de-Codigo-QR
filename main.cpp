#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <cstdarg>

using namespace cv;
using namespace std;

const int QR_NORTE = 0;
const int QR_ESTE = 1;
const int QR_SUR = 2;
const int QR_OESTE = 3;

float distancia(Point2f P, Point2f Q);					// Distancia Entre dos puntos
float DistRectaPunt(Point2f L, Point2f M, Point2f J);		// Distancia perpendicular de un punto J de la linea formada por los puntos L y M; Solucion a la ecuacion de la line Val = ax+by+c
float Pendiente(Point2f L, Point2f M, int& alignement);	// Pendiente de una linea por dos puntos L y M sobre ella; Pendiente de la recta, S = (x1 -x2) / (y1- y2)

// Funciones necesarias

void Obtener_Vertices(vector<vector<Point> > contorno, int c_id,float slope, vector<Point2f>& X);
void Actualizar_Vertices(Point2f P, Point2f ref ,float& baseline,  Point2f& corner);
void Act_Ver_Orientacion(int orientation, vector<Point2f> IN, vector<Point2f> &OUT);
bool Punto_interseccion(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& interseccion);
float cruce(Point2f v1,Point2f v2);

int main (int argc, char **argv []){

    string msg1 = "Estas listo para comenzar \n\n\t[ENTER] Para detectar el codigo QR \n";


    cout<<"                         #################################################################\n"
        <<"                         #     Trabajo Final - Procesamiento de Señales Digitales II     #\n"
        <<"                         #                Deteccion de Codigo QR en OpenCV               #\n"
        <<"                         #                                                               #\n"
        <<"                         #                                    Alumno: Lucero Carlos Saul #\n"
        <<"                         #################################################################\n"
        <<"\n"
        <<"\n"
        << msg1;

    getchar();

	VideoCapture capture(0); //abre la camara por defecto

	Mat imagen;

	if(!capture.isOpened()) { cerr << " ERR: No se puede encontrar la fuente de entrada de vídeo." << endl;  //control de acceso a la camara web, osea se controla si la camara esta funcionando
		return -1;
	}

	vector<vector<Point> > contorno;
	vector<Vec4i> jerarquia;

	int mark,A,B,C,superior,derecha,inferior,mediana1,mediana2,outlier,im;     //outlier-parte aislada
	float AB,BC,CA, dist,slope;

	int alineacion,orientacion;

	int DBG=1;                                  // bandera de depuracion
	int key= 0;
	int key1=0;
	while (key != 'q'){

        if(im == 2){
           key1='s';
        }


        system("cls");
        cout<<"                         #################################################################\n"
            <<"                         #     Trabajo Final - Procesamiento de Señales Digitales II     #\n"
            <<"                         #                Deteccion de Codigo QR en OpenCV               #\n"
            <<"                         #                                                               #\n"
            <<"                         #                                    Alumno: Lucero Carlos Saul #\n"
            <<"                         #################################################################\n"
            <<"\n"
            <<"\n"
            <<"Coloque su codigo QR enfrente de la camara \n\n"
            <<"Luego presione [q] para salir \n";


	   	capture >> imagen;

	   	if(imagen.empty()){ cerr << "ERR: Error la imagen esta vacia.\n" << endl; // control, para ver si la imagen esta vacia
		return -1;
	   	}



	// Creacion de objetos intermedios (imagenes) necesarios para mas adelante
        Mat gris(imagen.size(), CV_MAKETYPE(imagen.depth(), 1));			// Hace una imagen de 1 canal del mismo tamaño y del mismo tipo de canal que la imagen original.
        Mat edges(imagen.size(), CV_MAKETYPE(imagen.depth(), 1));			// Igual a lo de arriba
        Mat gaus(imagen.size(), CV_MAKETYPE(imagen.depth(), 1));
        Mat trazas(imagen.size(), CV_8UC3);								    // Para depurar, matriz de trazas. Hace una imagen de 3 canales (color) del mismo tamaño que la imagen de entrada-
        Mat qr,qr_raw,qr_gray,qr_thres,qr_thres_f,trazas_f,imagen_f,kernel,qr_thres1;               //creo solo la cabecera de la matrices. osea creo matrices con esos nombres.


        trazas = Scalar(0,0,0);
		qr_raw = Mat::zeros(100, 100, CV_8UC3 );
	   	qr = Mat::zeros(100, 100, CV_8UC3 );
		qr_gray = Mat::zeros(100, 100, CV_8UC1);
	   	qr_thres = Mat::zeros(300, 300, CV_8UC1);


        cvtColor(imagen,gris,CV_RGB2GRAY);		// Convierte la imagen capturada  a escala de grises
        GaussianBlur(gris, gaus, Size(3,3), 0);
        Canny(gaus, edges, 100 , 200 ,3);		// Se aplica canny para la detecciond e bordes en la imgen a escala de grises


        //jerarquia es un vector que almacejna las relaciones jerarquicas de cada uno de los contornos.
        findContours( edges, contorno, jerarquia, RETR_TREE, CHAIN_APPROX_SIMPLE); // Busca contornos, almacena la cantidad de contornos detectados y se almacena en una vector de puntos (que es la variable contorno)
                                                                                    //tiene tantos elementos como el número de contornos.
        mark = 0;								// Reestablece todos los marcadores detectados para el fotograma

		//Obtencion de momentos para todos los contornos y centro de masas
		vector<Moments> mu(contorno.size());
  		vector<Point2f> mc(contorno.size());

		for( int i = 0; i < contorno.size(); i++ ){
            mu[i] = moments( contorno[i], false );                        // obtenemos los momentos de cada uno de los contornos previamente adquiridos.
			mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); // obtengo el centro de masa
		}

		// Proceso de los datos de contorno

		// Se encuentran tres contornos cerrados A,B,C
		// Nota: 1. Se supone que el contorno que encierra otros contornos son las 3 marcas de alineacion del codigo QR.
		// 2.La relacion de areas de los cuadros "concentricos" tambien se pueden usar para identificar los marcadodes de alineacion de base.
		// Primer metodo

        for( int i = 0; i < contorno.size(); i++ ){
			int k=i;
			int c=0;

			while(jerarquia[k][2] != -1)            //ID de contorno del contorno infantil anidado inmediato, el valor es -1 si no hay contornos de nido
			{

				k = jerarquia[k][2] ;
				c = c+1;
			}

			if(jerarquia[k][2] != -1)
			c = c+1;

            if (c >= 5)
			{
				if (mark == 0)		    A = i;
				else if  (mark == 1)	B = i;		// A ya se encontro, se asigna el contorno actual al B
				else if  (mark == 2)	C = i;		// Ya se encontraron  A y B, se asigna el contorno actual a C.
				mark = mark + 1 ;
			}
        }


        if (mark == 3)		//Para asegurarnos si ya tenemos por lo menos 3 marcadores de alineacion descubiertos
		{
			// Una vez decubierto los 3 marcadores, se necesita saber cuales son los marcadores superior, derecho e inferior.

			// Deteccion del marcador Superior
			// El vértice del triangulo no involucrado en el lado mas largo es la parte aislada
            im=1;
			AB = distancia(mc[A],mc[B]);
			BC = distancia(mc[B],mc[C]);
			CA = distancia(mc[C],mc[A]);

			if ( AB > BC && AB > CA )
			{
				outlier = C; mediana1=A; mediana2=B;
			}
			else if ( CA > AB && CA > BC )
			{
				outlier = B; mediana1=A; mediana2=C;
			}
			else if ( BC > AB && BC > CA )
			{
				outlier = A;  mediana1=B; mediana2=C;
			}
            superior = outlier;							// Eleccion mas facil

			dist = DistRectaPunt(mc[mediana1], mc[mediana2], mc[outlier]);	// Obtencion de la distancia perpendicular de la parte mas aislada desde el lado mas largo.
			slope = Pendiente(mc[mediana1], mc[mediana2],alineacion);		// calculo de la pendiente del lado mas largo.

			// Una vez obtenida la orientacion de la linea formada median1 & median2 y tambien tenemos la posicion del valor atipico w.r.t.la linea.
			// Determinacion de los marcadores derecho e inferior

            if (alineacion == 0)
			{
				inferior = mediana1;
				derecha = mediana2;
			}
			else if (slope < 0 && dist < 0 )		// Orientacion Norte
			{
				inferior= mediana1;
				derecha = mediana2;
				orientacion = QR_NORTE;
			}
			else if (slope > 0 && dist < 0 )		// Orientacion Este
			{
				derecha = mediana1;
				inferior = mediana2;
				orientacion = QR_ESTE;
			}
			else if (slope < 0 && dist > 0 )		// Orientacion Sur
			{
				derecha = mediana1;
				inferior = mediana2;
				orientacion = QR_SUR;
			}

			else if (slope > 0 && dist > 0 )		// Orientacion Oeste
			{
				inferior = mediana1;
				derecha = mediana2;
				orientacion = QR_OESTE;
			}

			// Para asegurar que los valores no deseados no se coloquen cuando el codigo qr no esta presente.
			if( superior < contorno.size() && derecha < contorno.size() && inferior < contorno.size() && contourArea(contorno[superior]) > 10 && contourArea(contorno[derecha]) > 10 && contourArea(contorno[inferior]) > 10 )
			{

                vector<Point2f> L,M,O, tempL,tempM,tempO;
				Point2f N;

				vector<Point2f> src,dst;		// src - señala los vertices del cuadrilatero imagen de origen
												// dst - Puntos de destino para transformar la imagen de superposicion.Señala las cordenadas de los vertices del cuadrilatero de imagen destino.

				Mat warp_matrix;

				Obtener_Vertices(contorno,superior,slope,tempL);
				Obtener_Vertices(contorno,derecha,slope,tempM);
				Obtener_Vertices(contorno,inferior,slope,tempO);

				Act_Ver_Orientacion(orientacion, tempL, L); 			    // reorganiza las esquinas del marcador en la orientacion segun la pocicion del codigo Qr.
				Act_Ver_Orientacion(orientacion, tempM, M);
				Act_Ver_Orientacion(orientacion, tempO, O);

				int iflag = Punto_interseccion(M[1],M[2],O[3],O[2],N);

                // src - Señala las 4 coordenadas finales de la imagen

				src.push_back(L[0]);
				src.push_back(M[1]);
				src.push_back(N);
				src.push_back(O[3]);

                // dst - Puntos de destino para transformar la imagen de superposicion.

				dst.push_back(Point2f(0,0));
				dst.push_back(Point2f(qr.cols,0));
				dst.push_back(Point2f(qr.cols, qr.rows));
				dst.push_back(Point2f(0, qr.rows));

				if (src.size() == 4 && dst.size() == 4 )			                                    // Prueba de fallas para el calculo de matriz de deformacion para tener solo 4 puntos con src y dst.
				{
					warp_matrix = getPerspectiveTransform(src, dst);                                    //calcula la transformada perspectiva a partir de 4 pares de puntos.
					warpPerspective(imagen, qr_raw, warp_matrix, Size(qr.cols, qr.rows));
					copyMakeBorder( qr_raw, qr, 10, 10, 10, 10,BORDER_CONSTANT, Scalar(255,255,255) );  //genera un borde alrededor de una imagen.

					cvtColor(qr,qr_gray,CV_RGB2GRAY);                                                   // convierte una imagen RGB a escalas de grises. (qr_gray)
					threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);                           //aplica un nivel fijo de umbral a cada elemento de la matriz.

				}

				//Dibuja contornos en la imagen

				drawContours( imagen, contorno, superior , Scalar(255,200,0), 2, 8, jerarquia, 0 );
				drawContours( imagen, contorno, derecha , Scalar(0,0,255), 2, 8, jerarquia, 0 );
				drawContours( imagen, contorno, inferior , Scalar(255,0,100), 2, 8, jerarquia, 0 );
                circle( imagen, N, 3,  Scalar(0,255,0), -1, 8, 0 );


				// Instrucciones de depuracion
				if(DBG==1)
				{
					// Impresiones de depuracion
					// Visualizaciones para facilitar la comprension
					if (slope > 5)
						circle( trazas, Point(10,20) , 5 ,  Scalar(0,0,255), -1, 8, 0 );
					else if (slope < -5)
						circle( trazas, Point(10,20) , 5 ,  Scalar(255,255,255), -1, 8, 0 );

					// Dibujo de contornos en la imagen Trazas para su analisis
					drawContours( trazas, contorno, superior , Scalar(255,0,100), 1, 8, jerarquia, 0 );
					drawContours( trazas, contorno, derecha , Scalar(255,0,100), 1, 8, jerarquia, 0 );
					drawContours( trazas, contorno, inferior , Scalar(255,0,100), 1, 8, jerarquia, 0 );

					// Dibujo de 4 esquinas en la imagen Traza para cada marcador de identificacion.
					circle( trazas, L[0], 2,  Scalar(255,255,0), -1, 8, 0 );
					circle( trazas, L[1], 2,  Scalar(0,255,0), -1, 8, 0 );
					circle( trazas, L[2], 2,  Scalar(0,0,255), -1, 8, 0 );
					circle( trazas, L[3], 2,  Scalar(128,128,128), -1, 8, 0 );

					circle( trazas, M[0], 2,  Scalar(255,255,0), -1, 8, 0 );
					circle( trazas, M[1], 2,  Scalar(0,255,0), -1, 8, 0 );
					circle( trazas, M[2], 2,  Scalar(0,0,255), -1, 8, 0 );
					circle( trazas, M[3], 2,  Scalar(128,128,128), -1, 8, 0 );

					circle( trazas, O[0], 2,  Scalar(255,255,0), -1, 8, 0 );
					circle( trazas, O[1], 2,  Scalar(0,255,0), -1, 8, 0 );
					circle( trazas, O[2], 2,  Scalar(0,0,255), -1, 8, 0 );
					circle( trazas, O[3], 2,  Scalar(128,128,128), -1, 8, 0 );

					// Dibujo del punto de la estimacion 4° esquina del codigo QR
					circle( trazas, N, 2,  Scalar(255,255,255), -1, 8, 0 );

					// Dibuja las lineas utilizadas para estimar la 4° esquina del codigo QR.
					line(trazas,M[1],N,Scalar(0,0,255),1,8,0);
					line(trazas,O[3],N,Scalar(0,0,255),1,8,0);

					// Muestra la orientacion del codigo QR en el espacio de la imagen 2D.
					int fontFace = FONT_HERSHEY_PLAIN;

					if(orientacion == QR_NORTE)
					{
						putText(trazas, "NORTE", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
					}
					else if (orientacion == QR_ESTE)
					{
						putText(trazas, "ESTE", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
					}
					else if (orientacion == QR_SUR)
					{
						putText(trazas, "SUR", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
					}
					else if (orientacion == QR_OESTE)
					{
						putText(trazas, "OESTE", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
					}
                    if(im==1){

                                imwrite("/Trabajo Final DSP II/captura_img.jpg",imagen);
                                imwrite("/Trabajo Final DSP II/captura_trazas.jpg",trazas);
                                imwrite("/Trabajo Final DSP II/captura_qr.jpg",qr_thres);
                                key1 = 0;

                                system("cls");
                                cout<<"                         #################################################################\n"
                                    <<"                         #     Trabajo Final - Procesamiento de Señales Digitales II     #\n"
                                    <<"                         #                Deteccion de Codigo QR en OpenCV               #\n"
                                    <<"                         #                                                               #\n"
                                    <<"                         #                                    Alumno: Lucero Carlos Saul #\n"
                                    <<"                         #################################################################\n"
                                    <<"\n"
                                    <<"\n"
                                    <<"Presione la letra [s] para salir \n";


                                for(;;){
                                    imagen_f  = imread("/Trabajo Final DSP II/captura_img.jpg", CV_LOAD_IMAGE_COLOR);
                                    trazas_f  = imread("/Trabajo Final DSP II/captura_trazas.jpg", CV_LOAD_IMAGE_COLOR);
                                    qr_thres_f = imread("/Trabajo Final DSP II/captura_qr.jpg", CV_LOAD_IMAGE_COLOR);

                                    imshow ( "Imagen", imagen_f );
                                    imshow ( "Trazas_f", trazas_f );
                                    imshow ( "QR codigo f", qr_thres_f );


                                    if(key1=='s')break;
                                    key1 = waitKey(20);	                // Espera un tiempo determinado antes de acceder al siguiente fotograma

                                }
                                im=im+1;
                                destroyWindow("Trazas_f");
                                destroyWindow("QR codigo f");
                    }
				}
			}

		}
        if(!imagen.empty()){            // colocado debido a un erro porque la camara toma tiempo para empezar a tomar los frame.
		imshow ( "Imagen", imagen );

        }
        key = waitKey(1);	            // OPENCV: Espera un segundo antes de acceder al siguiente fotograma
	}
    system("cls");
    cout<<"                         #################################################################\n"
        <<"                         #     Trabajo Final - Procesamiento de Señales Digitales II     #\n"
        <<"                         #                Deteccion de Codigo QR en OpenCV               #\n"
        <<"                         #                                                               #\n"
        <<"                         #                                    Alumno: Lucero Carlos Saul #\n"
        <<"                         #################################################################\n"
        <<"\n"
        <<"\n"
        <<"Presione [q] para salir \n";
	return 0;
}



// Funcion para obtener distancia entre 2 puntos

float distancia(Point2f P, Point2f Q)
{
	return sqrt(pow(abs(P.x - Q.x),2) + pow(abs(P.y - Q.y),2)) ;
}

// Funcion para la distancia perpendicular de un punto J de la linea formada pór los puntos L y M. Ecuacion de una linea ax+by+c=0
// Dado 3 puntos la funcion deriva la ecuacion de linea de los dos primeros puntos y devuelve la distancia perpendicular del
// punto 3 de la linea.

float DistRectaPunt(Point2f L, Point2f M, Point2f J)
{
	float a,b,c,pdist;

	a = -((M.y - L.y) / (M.x - L.x));
	b = 1.0;
	c = (((M.y - L.y) /(M.x - L.x)) * L.x) - L.y;
// Una vez obtenido los parametros a,b,c de la ecuacion sustituyo (x,y) por los valores del punto J.

	pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));
	return pdist;
}

// Funcion de la pendiente de una linea que pasa por dos puntos L y M. Pendiente de la recta: S = (x1 -x2) / (y1- y2)
// La funcion devuelve la pendiente de la linea formada por dos puntos dados, la badera de alineacion indica que la linea es
// vertical y la pendiente es infinito.

float Pendiente(Point2f L, Point2f M, int& alignement)
{
	float dx,dy;
	dx = (M.x - L.x);
	dy = M.y - L.y;
	if ( dy != 0)
	{
		alignement = 1;
		return (dy / dx);
	}
	else				// Para asegurarse que no estamos dividiendo por cero.
	{
		alignement = 0;
		return 0.0;
	}
}


// Función: Rutina para calcular 4 Esquinas del Marcador en el Espacio de Imagen utilizando la partición de Región
// Teoría: Los contornos de OpenCV almacenan todos los puntos que lo describen y estos puntos se encuentran en el perímetro del polígono.
// La siguiente función elige los puntos más lejanos del polígono ya que forman los vértices de ese polígono, exactamente los puntos
// que estamos buscando. Para elegir el punto más lejano, el polígono se divide en 4 regiones iguales que las regiones usando
// el cuadro delimitador. El algoritmo de distancia se aplica entre el centro de la caja delimitadora cada punto del contorno en esa región,
// el punto más lejano se considera como el vértice de esa región. Calculando para las 4 regiones obtenemos las 4 esquinas del polígono (- cuadrilátero).

void Obtener_Vertices (vector<vector<Point> > contorno, int c_id, float slope, vector<Point2f>& quad){
	Rect box;
	box = boundingRect( contorno[c_id]);   //La función calcula y devuelve el rectángulo delimitador arriba-derecha
                                           //mínimo para el conjunto de puntos especificado.
	Point2f M0,M1,M2,M3;
	Point2f A, B, C, D, W, X, Y, Z;
	A =  box.tl();
	B.x = box.br().x;
	B.y = box.tl().y;
	C = box.br();
	D.x = box.tl().x;
	D.y = box.br().y;
	W.x = (A.x + B.x) / 2;
	W.y = A.y;
	X.x = B.x;
	X.y = (B.y + C.y) / 2;
	Y.x = (C.x + D.x) / 2;
	Y.y = C.y;
	Z.x = D.x;
	Z.y = (D.y + A.y) / 2;
	float dmax[4];
	dmax[0]=0.0;
	dmax[1]=0.0;
	dmax[2]=0.0;
	dmax[3]=0.0;
		for( int i = 0; i < contorno[c_id].size(); i++ )		{
			if((contorno[c_id][i].x < W.x) && (contorno[c_id][i].y <= Z.y))
			{
			    Actualizar_Vertices(contorno[c_id][i],C,dmax[2],M0);
			}
			else if((contorno[c_id][i].x >= W.x) && (contorno[c_id][i].y < Z.y))
			{
			    Actualizar_Vertices(contorno[c_id][i],D,dmax[3],M1);
			}
			else if((contorno[c_id][i].x > W.x) && (contorno[c_id][i].y >= Z.y))
			{
			    Actualizar_Vertices(contorno[c_id][i],A,dmax[0],M2);
			}
			else if((contorno[c_id][i].x <= W.x) && (contorno[c_id][i].y > Z.y))
			{
			    Actualizar_Vertices(contorno[c_id][i],B,dmax[1],M3);
			}
	}
	quad.push_back(M0);
	quad.push_back(M1);
	quad.push_back(M2);
	quad.push_back(M3);
}

// Funcion para comparar si un punto esta mas lejado que la distancia mas lejana registrada
// Detecta el punto mas lejado usando el punto de referencia y la distancia de referencia.
void Actualizar_Vertices(Point2f P, Point2f ref , float& baseline,  Point2f& corner){
    float temp_dist;
    temp_dist = distancia(P,ref);

    if(temp_dist > baseline)
    {
        baseline = temp_dist;			// La distancia mas lejana es la nueva linea de base.
        corner = P;						//P es ahora el punto mas lejano.
    }

}


//localización de las 4 esquinas de cada patrón y orientarlas segun la orientacion del codigo Qr.
void Act_Ver_Orientacion(int orientacion, vector<Point2f> IN,vector<Point2f> &OUT){
	Point2f M0,M1,M2,M3;
    if(orientacion == QR_NORTE){
		M0 = IN[0];
		M1 = IN[1];
	 	M2 = IN[2];
		M3 = IN[3];
	}
	else if (orientacion == QR_ESTE){
		M0 = IN[1];
		M1 = IN[2];
	 	M2 = IN[3];
		M3 = IN[0];
	}
	else if (orientacion == QR_SUR)	{
		M0 = IN[2];
		M1 = IN[3];
	 	M2 = IN[0];
		M3 = IN[1];
	}
	else if (orientacion == QR_OESTE){
		M0 = IN[3];
		M1 = IN[0];
	 	M2 = IN[1];
		M3 = IN[2];
	}
	OUT.push_back(M0);
	OUT.push_back(M1);
	OUT.push_back(M2);
	OUT.push_back(M3);
}

// Funcion para obtener el punto de interseccion de la lineas formadas por conjuntos de puntos
bool Punto_interseccion (Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& interseccion){
    Point2f p = a1;
    Point2f q = b1;
    Point2f r(a2-a1);
    Point2f s(b2-b1);

    if(cruce(r,s) == 0) {return false;}

    float t = cruce(q-p,s)/cruce(r,s);

    interseccion = p + t*r;
    return true;
}

float cruce(Point2f v1,Point2f v2){
    return v1.x*v2.y - v1.y*v2.x;
}


